#include "pcl/pcl_adsr.h"

#include "core/pcl_octave.h"

/*
 * Control-rate ADSR. At each control point (every PCL_CONTROL_DIV samples) the gate edges seen
 * since the last point are applied, the running stage is advanced one period, and the level at the
 * END of the period is computed; the samples in between are a linear ramp from the previous end
 * level. A stage of N periods moves its progress x from 0 to 1 in N equal steps and lands on its
 * end level exactly.
 *
 * Internal level format (pcl_coef_t): float in [0, 1], or Q2.30 in fixed-point builds with 1.0 held
 * as the largest value below it (2^30 - 1), so the Q15 and Q31 outputs never overflow. The stage
 * curve is d(x) = (2^(-m x) - 2^(-m)) / (1 - 2^(-m)), m = 16 * curve octaves, evaluated as the
 * square of 2^(-m x / 2) with the shared 2^x table (core/pcl_octave.h); all 64-bit products are at
 * control rate.
 */

#define ADSR_CFG_ATTACK 0
#define ADSR_CFG_DECAY 1
#define ADSR_CFG_RELEASE 2

#define ADSR_MS_PER_SECOND 1000u

/* Curves below this fraction are exactly linear: the table exponential is less accurate than a
   straight line there (docs/modules/adsr.md). 1/1024. */
#define ADSR_CURVE_LINEAR_BELOW_NUM 1
#define ADSR_CURVE_LINEAR_BELOW_DEN 1024

PCL_STATIC_ASSERT(PCL_CONTROL_DIV >= 1 && PCL_CONTROL_DIV <= 255, adsr_control_div_fits_uint8);

/* Schmitt trigger thresholds on the gate buffer: on when rising through HIGH, off below LOW. */
#if PCL_SAMPLE_TYPE == PCL_FLOAT
#define ADSR_GATE_HIGH 0.5f
#define ADSR_GATE_LOW 0.25f
#elif PCL_SAMPLE_TYPE == PCL_Q15
#define ADSR_GATE_HIGH PCL_CONST_Q15(0.5)
#define ADSR_GATE_LOW PCL_CONST_Q15(0.25)
#else
#define ADSR_GATE_HIGH PCL_CONST_Q31(0.5)
#define ADSR_GATE_LOW PCL_CONST_Q31(0.25)
#endif

/* ---- Level maths (one place per sample type) ---- */

#if PCL_SAMPLE_TYPE == PCL_FLOAT

#define ADSR_ONE 1.0f
#define ADSR_X_TO_FLOAT (1.0f / 4294967296.0f)
typedef float adsr_unit_t; /* a curve value in [0, 1] */

static pcl_coef_t adsr_clamp(pcl_coef_t v)
{
    return v < 0.0f ? 0.0f : (v > ADSR_ONE ? ADSR_ONE : v);
}

/* d(x) for the stage constants `c`: 1 at x = 0, 0 at x = 1. */
static adsr_unit_t adsr_decay(const pcl_adsr_stage_cfg_t *c, uint32_t x)
{
    const float xf = (float)x * ADSR_X_TO_FLOAT;
    float       s, e, d;

    if (c->h == 0.0f)
        return 1.0f - xf;
    s = pcl_octave_scale_f(1.0f, -c->h * xf);
    e = s * s;
    d = (e - c->e1) * c->invden;
    return d < 0.0f ? 0.0f : (d > 1.0f ? 1.0f : d);
}

static adsr_unit_t  adsr_one_minus(adsr_unit_t d) { return 1.0f - d; }
static pcl_coef_t   adsr_scale(pcl_coef_t v, adsr_unit_t d) { return v * d; }
static pcl_sample_t adsr_out(pcl_coef_t v) { return v; }
/*
 * Float ramps are NOT accumulated (cur += step): fifteen rounded additions can drift past the end
 * point by more than the ramp's own margin when the two end levels are a few ulps apart (the flat
 * top of a steep attack), which overshot 1.0 and broke monotonicity. Instead each sample is v0 +
 * diff * t with t = j / PCL_CONTROL_DIV: monotone in j, and never beyond v0 + diff because t < 1.
 * Fixed point adds exactly, so it keeps the accumulator. In float builds `step` holds the period's
 * whole difference.
 */
static pcl_coef_t adsr_step(pcl_coef_t diff) { return diff; }

#else

#define ADSR_UNIT_SHIFT 30
#define ADSR_UNIT (1 << ADSR_UNIT_SHIFT)
#define ADSR_ONE (ADSR_UNIT - 1)
#define ADSR_X_TO_UNIT_SHIFT 2 /* x is Q32; a curve value is Q30 */
#define ADSR_INVDEN_SHIFT 24
typedef int32_t adsr_unit_t; /* a curve value in [0, 2^30] */

static pcl_coef_t adsr_clamp(pcl_coef_t v) { return v < 0 ? 0 : (v > ADSR_ONE ? ADSR_ONE : v); }

static adsr_unit_t adsr_decay(const pcl_adsr_stage_cfg_t *c, uint32_t x)
{
    int32_t  oct;
    uint32_t s, e;
    uint64_t d;

    if (c->h == 0u)
        return ADSR_UNIT - (int32_t)(x >> ADSR_X_TO_UNIT_SHIFT);
    /* 2^(-h x), h x < 8 octaves (Q4.28), then squared: 2^(-2 h x) = 2^(-m x). */
    oct = -(int32_t)(((uint64_t)c->h * x) >> 32);
    s = pcl_octave_scale_inc((uint32_t)ADSR_UNIT, oct, 0xFFFFFFFFu);
    e = (uint32_t)(((uint64_t)s * s) >> ADSR_UNIT_SHIFT);
    if (e <= c->e1)
        return 0;
    d = ((uint64_t)(e - c->e1) * c->invden) >> ADSR_INVDEN_SHIFT;
    return d > (uint64_t)ADSR_UNIT ? ADSR_UNIT : (adsr_unit_t)d;
}

static adsr_unit_t adsr_one_minus(adsr_unit_t d) { return ADSR_UNIT - d; }

/* v * d, both Q30. */
static pcl_coef_t adsr_scale(pcl_coef_t v, adsr_unit_t d)
{
    return (pcl_coef_t)(((int64_t)v * (int64_t)d) >> ADSR_UNIT_SHIFT);
}

static pcl_sample_t adsr_out(pcl_coef_t v)
{
#if PCL_SAMPLE_TYPE == PCL_Q15
    return (pcl_sample_t)(v >> 15);
#else
    return v * 2;
#endif
}

/* Truncated toward zero, so a ramp can never overshoot its end point. */
static pcl_coef_t adsr_step(pcl_coef_t diff) { return diff / (int32_t)PCL_CONTROL_DIV; }

#endif

/* ---- Stage constants (setters; may divide) ---- */

static uint32_t adsr_ticks(uint32_t ms, uint32_t sample_rate_hz)
{
    const uint64_t period = (uint64_t)ADSR_MS_PER_SECOND * PCL_CONTROL_DIV;
    const uint64_t n = ((uint64_t)ms * sample_rate_hz + period / 2u) / period;
    return n > 0u ? (uint32_t)n : 1u;
}

static int adsr_unit_valid(pcl_coef_t v)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    /* Written positively so that NaN fails both comparisons. */
    return v >= 0.0f && v <= 1.0f;
#else
    return v >= 0 && v <= PCL_COEF(1.0);
#endif
}

static int adsr_curve_is_linear(pcl_coef_t curve)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return curve < (float)ADSR_CURVE_LINEAR_BELOW_NUM / (float)ADSR_CURVE_LINEAR_BELOW_DEN;
#else
    return curve < (pcl_coef_t)(PCL_COEF(1.0) / ADSR_CURVE_LINEAR_BELOW_DEN);
#endif
}

/* Builds the stage constants for `ms` and `curve` at the sample rate. Both are already validated.
 */
static void adsr_make_cfg(
    pcl_adsr_stage_cfg_t *c, uint32_t ms, pcl_coef_t curve, uint32_t sample_rate_hz)
{
    c->ticks = adsr_ticks(ms, sample_rate_hz);
    c->inc = c->ticks == 1u ? 0xFFFFFFFFu : (uint32_t)(4294967296ull / c->ticks);

    if (adsr_curve_is_linear(curve))
    {
        c->h = 0;
        c->e1 = 0;
        c->invden = 0;
        return;
    }
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    c->h = 8.0f * curve; /* half of m = 16 * curve octaves */
    {
        const float s1 = pcl_octave_scale_f(1.0f, -c->h);
        c->e1 = s1 * s1;
        c->invden = 1.0f / (1.0f - c->e1);
    }
#else
    {
        /* h in Q4.28: 8 * curve octaves = curve(Q30) * 2^31 / 2^30 = 2 * curve. */
        uint32_t       s1, den;
        const uint32_t h = (uint32_t)curve * 2u;

        c->h = h;
        s1 = pcl_octave_scale_inc((uint32_t)ADSR_UNIT, (int32_t)(-(int64_t)h), 0xFFFFFFFFu);
        c->e1 = (uint32_t)(((uint64_t)s1 * s1) >> ADSR_UNIT_SHIFT);
        den = (uint32_t)ADSR_UNIT - c->e1;
        c->invden
            = (uint32_t)((((uint64_t)1 << (ADSR_UNIT_SHIFT + ADSR_INVDEN_SHIFT)) + den / 2u) / den);
    }
#endif
}

/* ---- State machine ---- */

/* Starts a stage at the current level. */
static void adsr_enter(pcl_adsr_t *a, pcl_adsr_stage_t stage)
{
    a->stage = (uint8_t)stage;
    a->start = a->v0;
    a->x = 0u;
    if (stage == PCL_ADSR_ATTACK)
        a->act = a->cfg[ADSR_CFG_ATTACK];
    else if (stage == PCL_ADSR_DECAY)
        a->act = a->cfg[ADSR_CFG_DECAY];
    else if (stage == PCL_ADSR_RELEASE)
        a->act = a->cfg[ADSR_CFG_RELEASE];
    a->ticks_left = a->act.ticks;
}

static void adsr_gate_edge(pcl_adsr_t *a, int on)
{
    if (on && !a->gate)
    {
        a->gate = 1u;
        a->rose = 1u;
    }
    else if (!on && a->gate)
    {
        a->gate = 0u;
        a->fell = 1u;
    }
}

static void adsr_on(pcl_adsr_t *a)
{
    a->release_pending = 0u;
    if (a->stage == PCL_ADSR_IDLE || a->stage == PCL_ADSR_RELEASE)
        adsr_enter(a, PCL_ADSR_ATTACK);
}

static void adsr_off(pcl_adsr_t *a)
{
    switch (a->stage)
    {
    case PCL_ADSR_ATTACK:
    case PCL_ADSR_DECAY:
        if (a->run_through)
            a->release_pending = 1u;
        else
            adsr_enter(a, PCL_ADSR_RELEASE);
        break;
    case PCL_ADSR_SUSTAIN:
        adsr_enter(a, PCL_ADSR_RELEASE);
        break;
    default:
        break;
    }
}

/* The running stage has delivered its last period: move on. */
static void adsr_advance(pcl_adsr_t *a, pcl_coef_t sus)
{
    switch (a->stage)
    {
    case PCL_ADSR_ATTACK:
        if (sus >= ADSR_ONE) /* nothing to decay */
        {
            a->stage = PCL_ADSR_SUSTAIN;
            if (a->release_pending)
                adsr_enter(a, PCL_ADSR_RELEASE);
        }
        else
        {
            adsr_enter(a, PCL_ADSR_DECAY);
        }
        break;
    case PCL_ADSR_DECAY:
        a->stage = PCL_ADSR_SUSTAIN;
        if (a->release_pending)
            adsr_enter(a, PCL_ADSR_RELEASE);
        break;
    default: /* RELEASE */
        a->stage = PCL_ADSR_IDLE;
        break;
    }
}

/* Starts a new control period: applies the gate edges, advances the stage, sets the ramp. */
static void adsr_tick(pcl_adsr_t *a)
{
    const pcl_coef_t sus = adsr_clamp(pcl_smooth_next(&a->sustain));
    pcl_coef_t       v1 = 0;
    int              last = 0;

    if ((a->stage == PCL_ADSR_ATTACK || a->stage == PCL_ADSR_DECAY || a->stage == PCL_ADSR_RELEASE)
        && a->ticks_left == 0u)
        adsr_advance(a, sus);

    if (a->rose)
        adsr_on(a);
    if (!a->gate && a->fell)
        adsr_off(a);
    a->rose = 0u;
    a->fell = 0u;

    if (a->stage == PCL_ADSR_ATTACK || a->stage == PCL_ADSR_DECAY || a->stage == PCL_ADSR_RELEASE)
    {
        a->ticks_left--;
        last = a->ticks_left == 0u;
        if (!last)
            a->x += a->act.inc;
    }

    switch (a->stage)
    {
    case PCL_ADSR_ATTACK:
        v1 = last
            ? ADSR_ONE
            : a->start + adsr_scale(ADSR_ONE - a->start, adsr_one_minus(adsr_decay(&a->act, a->x)));
        break;
    case PCL_ADSR_DECAY:
        v1 = last ? sus : sus + adsr_scale(a->start - sus, adsr_decay(&a->act, a->x));
        break;
    case PCL_ADSR_SUSTAIN:
        v1 = sus;
        break;
    case PCL_ADSR_RELEASE:
        v1 = last ? 0 : adsr_scale(a->start, adsr_decay(&a->act, a->x));
        break;
    default:
        v1 = 0;
        break;
    }
    v1 = adsr_clamp(v1);

    a->cur = a->v0;
    a->step = adsr_step(v1 - a->v0);
    a->v0 = v1;
}

/* ---- Public API ---- */

static uint32_t adsr_ramp_ticks(uint32_t sample_rate_hz)
{
    const uint32_t ticks = pcl_smooth_ramp_samples(sample_rate_hz) / (uint32_t)PCL_CONTROL_DIV;
    return ticks > 0u ? ticks : 1u;
}

static void adsr_restart(pcl_adsr_t *a)
{
    pcl_smooth_reset(&a->sustain);
    a->stage = PCL_ADSR_IDLE;
    a->x = 0u;
    a->ticks_left = 0u;
    a->v0 = 0;
    a->start = 0;
    a->cur = 0;
    a->step = 0;
    a->pos = 0u;
    a->gate = 0u;
    a->schmitt = 0u;
    a->rose = 0u;
    a->fell = 0u;
    a->release_pending = 0u;
}

pcl_status_t pcl_adsr_init(pcl_adsr_t *adsr, const pcl_adsr_config_t *cfg)
{
    if (adsr == NULL || cfg == NULL)
        return PCL_ERR_NULL_POINTER;
    if (cfg->sample_rate_hz < PCL_SAMPLE_RATE_MIN_HZ || cfg->sample_rate_hz > PCL_SAMPLE_RATE_MAX_HZ
        || cfg->attack_ms > PCL_ADSR_TIME_MAX_MS || cfg->decay_ms > PCL_ADSR_TIME_MAX_MS
        || cfg->release_ms > PCL_ADSR_TIME_MAX_MS || !adsr_unit_valid(cfg->sustain)
        || !adsr_unit_valid(cfg->attack_curve) || !adsr_unit_valid(cfg->decay_curve)
        || !adsr_unit_valid(cfg->release_curve))
        return PCL_ERR_OUT_OF_RANGE;

    adsr->sample_rate_hz = cfg->sample_rate_hz;
    adsr->run_through = cfg->run_through != 0 ? 1u : 0u;
    adsr_make_cfg(
        &adsr->cfg[ADSR_CFG_ATTACK], cfg->attack_ms, cfg->attack_curve, cfg->sample_rate_hz);
    adsr_make_cfg(&adsr->cfg[ADSR_CFG_DECAY], cfg->decay_ms, cfg->decay_curve, cfg->sample_rate_hz);
    adsr_make_cfg(
        &adsr->cfg[ADSR_CFG_RELEASE], cfg->release_ms, cfg->release_curve, cfg->sample_rate_hz);
    adsr->act = adsr->cfg[ADSR_CFG_ATTACK];
    pcl_smooth_init(&adsr->sustain, cfg->sustain, adsr_ramp_ticks(cfg->sample_rate_hz));
    adsr_restart(adsr);
    return PCL_OK;
}

void pcl_adsr_reset(pcl_adsr_t *adsr) { adsr_restart(adsr); }

static pcl_status_t adsr_set_time(pcl_adsr_t *a, int which, uint32_t ms)
{
    if (a == NULL)
        return PCL_ERR_NULL_POINTER;
    if (ms > PCL_ADSR_TIME_MAX_MS)
        return PCL_ERR_OUT_OF_RANGE;
    a->cfg[which].ticks = adsr_ticks(ms, a->sample_rate_hz);
    a->cfg[which].inc
        = a->cfg[which].ticks == 1u ? 0xFFFFFFFFu : (uint32_t)(4294967296ull / a->cfg[which].ticks);
    return PCL_OK;
}

static pcl_status_t adsr_set_curve(pcl_adsr_t *a, int which, pcl_coef_t curve)
{
    pcl_adsr_stage_cfg_t c;

    if (a == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!adsr_unit_valid(curve))
        return PCL_ERR_OUT_OF_RANGE;
    adsr_make_cfg(&c, 0u, curve, a->sample_rate_hz); /* the time fields are replaced below */
    c.ticks = a->cfg[which].ticks;
    c.inc = a->cfg[which].inc;
    a->cfg[which] = c;
    return PCL_OK;
}

pcl_status_t pcl_adsr_set_attack_ms(pcl_adsr_t *adsr, uint32_t ms)
{
    return adsr_set_time(adsr, ADSR_CFG_ATTACK, ms);
}

pcl_status_t pcl_adsr_set_decay_ms(pcl_adsr_t *adsr, uint32_t ms)
{
    return adsr_set_time(adsr, ADSR_CFG_DECAY, ms);
}

pcl_status_t pcl_adsr_set_release_ms(pcl_adsr_t *adsr, uint32_t ms)
{
    return adsr_set_time(adsr, ADSR_CFG_RELEASE, ms);
}

pcl_status_t pcl_adsr_set_sustain(pcl_adsr_t *adsr, pcl_coef_t sustain)
{
    if (adsr == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!adsr_unit_valid(sustain))
        return PCL_ERR_OUT_OF_RANGE;
    pcl_smooth_set_target(&adsr->sustain, sustain);
    return PCL_OK;
}

pcl_status_t pcl_adsr_set_attack_curve(pcl_adsr_t *adsr, pcl_coef_t curve)
{
    return adsr_set_curve(adsr, ADSR_CFG_ATTACK, curve);
}

pcl_status_t pcl_adsr_set_decay_curve(pcl_adsr_t *adsr, pcl_coef_t curve)
{
    return adsr_set_curve(adsr, ADSR_CFG_DECAY, curve);
}

pcl_status_t pcl_adsr_set_release_curve(pcl_adsr_t *adsr, pcl_coef_t curve)
{
    return adsr_set_curve(adsr, ADSR_CFG_RELEASE, curve);
}

void pcl_adsr_set_run_through(pcl_adsr_t *adsr, int run_through)
{
    adsr->run_through = run_through != 0 ? 1u : 0u;
}

void pcl_adsr_gate(pcl_adsr_t *adsr, int on) { adsr_gate_edge(adsr, on); }

pcl_adsr_stage_t pcl_adsr_stage(const pcl_adsr_t *adsr) { return (pcl_adsr_stage_t)adsr->stage; }

void pcl_adsr_process(pcl_adsr_t *adsr, pcl_sample_t *out, const pcl_sample_t *gate,
    const pcl_event_t *events, size_t num_events, size_t frames)
{
    size_t next_event = 0;
    size_t i = 0;

    while (i < frames)
    {
        size_t     n;
        size_t     k;
        pcl_coef_t cur;
        pcl_coef_t step;
#if PCL_SAMPLE_TYPE == PCL_FLOAT
        float t;
#endif

        if (adsr->pos == 0u)
            adsr_tick(adsr);

        n = (size_t)PCL_CONTROL_DIV - adsr->pos;
        if (n > frames - i)
            n = frames - i;

        cur = adsr->cur;
        step = adsr->step;
#if PCL_SAMPLE_TYPE == PCL_FLOAT
        t = (float)adsr->pos * (1.0f / (float)PCL_CONTROL_DIV);
#endif
        for (k = 0; k < n; k++)
        {
            const size_t idx = i + k;

            /* Events first, then the buffer; edges are latched for the next control point. */
            while (next_event < num_events && events[next_event].offset <= idx)
            {
                if (events[next_event].offset == idx)
                {
                    if (events[next_event].type == PCL_EVT_NOTE_ON)
                        adsr_gate_edge(adsr, 1);
                    else if (events[next_event].type == PCL_EVT_NOTE_OFF)
                        adsr_gate_edge(adsr, 0);
                }
                next_event++;
            }
            if (gate != NULL)
            {
                const pcl_sample_t g = gate[idx];
                if (g >= ADSR_GATE_HIGH)
                {
                    if (adsr->schmitt == 0u)
                    {
                        adsr->schmitt = 1u;
                        adsr_gate_edge(adsr, 1);
                    }
                }
                else if (g < ADSR_GATE_LOW)
                {
                    if (adsr->schmitt != 0u)
                    {
                        adsr->schmitt = 0u;
                        adsr_gate_edge(adsr, 0);
                    }
                }
            }
#if PCL_SAMPLE_TYPE == PCL_FLOAT
            out[idx] = adsr_out(cur + step * t);
            t += 1.0f / (float)PCL_CONTROL_DIV; /* exact for a power-of-two PCL_CONTROL_DIV */
#else
            out[idx] = adsr_out(cur);
            cur += step;
#endif
        }
        adsr->cur = cur;
        adsr->pos = (uint8_t)((adsr->pos + n) % (size_t)PCL_CONTROL_DIV);
        i += n;
    }
}
