#include "pcl/pcl_lfo.h"

#include "core/pcl_octave.h"
#include "core/pcl_rng.h"
#include "core/pcl_wave.h"

/*
 * Control-rate LFO. At each control point (every PCL_CONTROL_DIV samples) the shape is
 * evaluated one period ahead, at the phase the period ends on, and the samples in between are a
 * linear ramp from the previous value to that one. Because the shape is a pure function of
 * phase this look-ahead costs nothing and there is no lag.
 *
 * Internal value format ("lfo value", pcl_coef_t): float in [-1, 1], or Q2.30 in the fixed-point
 * builds, where every shape stays within [-2^30, 2^30 - 1]. That leaves one bit of headroom so
 * the difference of two values fits an int32_t, and Q15 and Q31 builds share all the code.
 */

#define LFO_POINTS_PER_CYCLE_MIN 16u
#define LFO_RATE_DIV (PCL_CONTROL_DIV * LFO_POINTS_PER_CYCLE_MIN)

/* Phase advance per control period at the maximum rate: 16 control points per cycle. */
#define LFO_ADV_MAX (1u << 28)
#define LFO_ADV_SHIFT 2 /* Q2.30 (frequency / maximum) to phase advance: * 2^28 / 2^30 */
#define LFO_HZ_Q16_FRAC_BITS 16
#define LFO_FN_FRAC_BITS 30
#define LFO_WIDTH_MIN PCL_COEF(0.01)
#define LFO_WIDTH_MAX PCL_COEF(0.99)

#define LFO_Q30_ONE (1 << 30)
#define LFO_Q30_TO_Q15_SHIFT 15
#define LFO_PICK_SHIFT 1

PCL_STATIC_ASSERT(PCL_CONTROL_DIV >= 1 && PCL_CONTROL_DIV <= 255, lfo_control_div_fits_uint8);

/* ---- Value helpers (one place per sample type) ---- */

#if PCL_SAMPLE_TYPE == PCL_FLOAT
#define LFO_ONE 1.0f
#define LFO_MINUS_ONE (-1.0f)
#define LFO_Q31_TO_FLOAT (1.0f / 2147483648.0f)
#define LFO_Q30_TO_FLOAT (1.0f / 1073741824.0f)
#define LFO_PHASE_TO_FLOAT (1.0f / 4294967296.0f)
#else
#define LFO_ONE (LFO_Q30_ONE - 1) /* the largest Q2.30 value below +1 */
#define LFO_MINUS_ONE (-LFO_Q30_ONE)
#endif

/* A Q1.31 value (the ramps from pcl_wave.h) as an lfo value. */
static pcl_coef_t lfo_from_w31(int32_t w)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return (float)w * LFO_Q31_TO_FLOAT;
#else
    return w >> 1;
#endif
}

/* A table sample (the sine) as an lfo value. */
static pcl_coef_t lfo_from_sample(pcl_sample_t s)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return s;
#elif PCL_SAMPLE_TYPE == PCL_Q15
    return (int32_t)s * (1 << LFO_Q30_TO_Q15_SHIFT);
#else
    return s >> 1;
#endif
}

/* A new random value, uniform in [-1, 1). */
static pcl_coef_t lfo_pick(uint32_t *rng)
{
    const int32_t v = pcl_rng_s32(pcl_rng_next(rng)) >> LFO_PICK_SHIFT;
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return (float)v * LFO_Q30_TO_FLOAT;
#else
    return v;
#endif
}

/* Bipolar value to the selected polarity. */
static pcl_coef_t lfo_polar(const pcl_lfo_t *l, pcl_coef_t v)
{
    if (!l->unipolar)
        return v;
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return 0.5f * v + 0.5f;
#else
    return (v + LFO_Q30_ONE) >> 1;
#endif
}

/* The per-sample increment of a linear ramp of `diff` over one control period. */
static pcl_coef_t lfo_ramp_step(pcl_coef_t diff)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    /* Float ramps are evaluated as v0 + diff * t, not accumulated: repeated rounded additions could
       overshoot the end point by an ulp or two where the two end levels are nearly equal (the flat
       top of a slow sine went to 1.0000002). `step` holds the whole period's difference in float
       builds. */
    return diff;
#else
    /* C division truncates toward zero, so the ramp can never overshoot its end point. */
    return diff / (int32_t)PCL_CONTROL_DIV;
#endif
}

static pcl_sample_t lfo_out(pcl_coef_t v)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return v;
#elif PCL_SAMPLE_TYPE == PCL_Q15
    return (pcl_sample_t)(v >> LFO_Q30_TO_Q15_SHIFT);
#else
    return v * 2;
#endif
}

/* ---- Shapes ---- */

/* Pulse threshold as a phase: square is high below it. */
static uint32_t lfo_width_thr(pcl_coef_t w)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return (uint32_t)(w * 4294967296.0f);
#else
    return (uint32_t)w << (32 - LFO_FN_FRAC_BITS);
#endif
}

static pcl_coef_t lfo_random_smooth(const pcl_lfo_t *l, uint32_t phase)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return l->r_prev + (l->r_cur - l->r_prev) * ((float)phase * LFO_PHASE_TO_FLOAT);
#else
    const int64_t diff = (int64_t)l->r_cur - (int64_t)l->r_prev;
    return l->r_prev + (int32_t)((diff * (int64_t)phase) >> 32);
#endif
}

/* The bipolar shape at `phase`. */
static pcl_coef_t lfo_shape(const pcl_lfo_t *l, uint32_t phase, uint32_t width_thr)
{
    switch (l->shape)
    {
    case PCL_LFO_TRIANGLE:
        /* The oscillator's triangle starts at -1; a quarter cycle later it is the zero-crossing
         * one. */
        return lfo_from_w31(pcl_wave_tri_w31(phase + PCL_WAVE_PHASE_QUARTER));
    case PCL_LFO_SAW_UP:
        /* The phase read as a signed value is 2 * frac(phase + 0.5) - 1. */
        return lfo_from_w31((int32_t)phase);
    case PCL_LFO_SAW_DOWN:
        return lfo_from_w31(-1 - (int32_t)phase); /* ones' complement: no overflow at INT32_MIN */
    case PCL_LFO_SQUARE:
        return phase < width_thr ? LFO_ONE : LFO_MINUS_ONE;
    case PCL_LFO_RANDOM_STEP:
        return l->r_cur;
    case PCL_LFO_RANDOM_SMOOTH:
        return lfo_random_smooth(l, phase);
    default:
        return lfo_from_sample(pcl_wave_sine(phase));
    }
}

/* ---- Control point ---- */

/* Phase advance over one control period for the smoothed frequency `u`, bent by `mod` octaves. */
static uint32_t lfo_advance(const pcl_lfo_t *l, pcl_coef_t u, pcl_sample_t mod)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    float f = u;

    if (mod != 0.0f && l->rate_depth != 0.0f)
        f = pcl_octave_scale_f(f, l->rate_depth * mod);
    if (f > 1.0f)
        f = 1.0f;
    return (
        uint32_t)(f * (float)LFO_ADV_MAX + 0.5f); /* rounded to the nearest count of 2^-32 cycles */
#else
    uint32_t adv = ((uint32_t)u + (1u << (LFO_ADV_SHIFT - 1))) >> LFO_ADV_SHIFT; /* rounded */

    if (mod != 0 && l->rate_depth != 0)
    {
#if PCL_SAMPLE_TYPE == PCL_Q15
        const int32_t oct = pcl_mul_coef_q15(l->rate_depth, mod); /* Q4.28 */
#else
        const int32_t oct = pcl_mul_coef_q31(l->rate_depth, mod);
#endif
        adv = pcl_octave_scale_inc(adv, oct, LFO_ADV_MAX);
    }
    return adv > LFO_ADV_MAX ? LFO_ADV_MAX : adv;
#endif
}

/* Starts a new control period: sets the ramp from the old end point to the shape one period ahead.
 */
static void lfo_tick(pcl_lfo_t *l, pcl_sample_t mod)
{
    const pcl_coef_t u = pcl_smooth_next(&l->freq);
    const uint32_t   thr = lfo_width_thr(pcl_smooth_next(&l->width));
    const uint32_t   adv = lfo_advance(l, u, mod);
    pcl_coef_t       beg, end;
    uint32_t         p1;

    if (l->jump)
    {
        l->v_next = lfo_shape(l, l->phase, thr);
        l->jump = 0u;
    }
    p1 = l->phase + adv;
    if (p1 < l->phase) /* wrapped: a new random value for the new cycle */
    {
        l->r_prev = l->r_cur;
        l->r_cur = lfo_pick(&l->rng);
    }
    beg = lfo_polar(l, l->v_next);
    l->v_next = lfo_shape(l, p1, thr);
    end = lfo_polar(l, l->v_next);
    l->cur = beg;
    l->step = lfo_ramp_step(end - beg);
    l->phase = p1;
}

/* ---- Public API ---- */

static uint32_t lfo_ramp_ticks(uint32_t sample_rate_hz)
{
    const uint32_t ticks = pcl_smooth_ramp_samples(sample_rate_hz) / (uint32_t)PCL_CONTROL_DIV;
    return ticks > 0u ? ticks : 1u;
}

static int lfo_hz_valid(pcl_hz_t hz, uint32_t sample_rate_hz)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    /* Written positively so that NaN fails both comparisons. */
    return hz >= 0.0f && hz <= (float)sample_rate_hz / (float)LFO_RATE_DIV;
#else
    return (uint64_t)hz * LFO_RATE_DIV <= ((uint64_t)sample_rate_hz << LFO_HZ_Q16_FRAC_BITS);
#endif
}

/* Hz to frequency / maximum frequency (1.0 = the maximum), as a pcl_coef_t; setter only. */
static pcl_coef_t lfo_hz_to_u(pcl_hz_t hz, uint32_t sample_rate_hz)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return hz * (float)LFO_RATE_DIV / (float)sample_rate_hz;
#else
    /* u(Q2.30) = hz(Q16.16) * LFO_RATE_DIV * 2^30 / (sample_rate * 2^16). 64-bit divide, setter
     * only. */
    const uint64_t scaled = ((uint64_t)hz * LFO_RATE_DIV)
        << (LFO_FN_FRAC_BITS - LFO_HZ_Q16_FRAC_BITS);
    return (pcl_coef_t)((scaled + (sample_rate_hz / 2u)) / sample_rate_hz);
#endif
}

static int lfo_width_valid(pcl_coef_t w) { return w >= LFO_WIDTH_MIN && w <= LFO_WIDTH_MAX; }

static int lfo_depth_valid(pcl_oct_t d)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return d >= 0.0f && d < 8.0f;
#else
    return d >= 0; /* Q4.28 cannot represent 8 or more */
#endif
}

static void lfo_restart(pcl_lfo_t *l)
{
    pcl_smooth_reset(&l->freq);
    pcl_smooth_reset(&l->width);
    l->rng = pcl_rng_seed(l->seed);
    l->r_prev = lfo_pick(&l->rng);
    l->r_cur = lfo_pick(&l->rng);
    l->phase = l->start_phase;
    l->jump = 0u;
    l->pos = 0u;
    l->cur = 0;
    l->step = 0;
    l->v_next = lfo_shape(l, l->phase, lfo_width_thr(l->width.current));
}

pcl_status_t pcl_lfo_init(pcl_lfo_t *lfo, const pcl_lfo_config_t *cfg)
{
    uint32_t ramp;

    if (lfo == NULL || cfg == NULL)
        return PCL_ERR_NULL_POINTER;
    if (cfg->sample_rate_hz < PCL_SAMPLE_RATE_MIN_HZ || cfg->sample_rate_hz > PCL_SAMPLE_RATE_MAX_HZ
        || (unsigned)cfg->shape >= (unsigned)PCL_LFO_SHAPE_COUNT
        || !lfo_hz_valid(cfg->frequency, cfg->sample_rate_hz)
        || !lfo_depth_valid(cfg->rate_depth_oct) || !lfo_width_valid(cfg->pulse_width))
        return PCL_ERR_OUT_OF_RANGE;

    ramp = lfo_ramp_ticks(cfg->sample_rate_hz);
    lfo->sample_rate_hz = cfg->sample_rate_hz;
    lfo->shape = (uint8_t)cfg->shape;
    lfo->unipolar = cfg->unipolar != 0 ? 1u : 0u;
    lfo->rate_depth = cfg->rate_depth_oct;
    lfo->start_phase = cfg->start_phase;
    lfo->seed = cfg->seed;
    pcl_smooth_init(&lfo->freq, lfo_hz_to_u(cfg->frequency, cfg->sample_rate_hz), ramp);
    pcl_smooth_init(&lfo->width, cfg->pulse_width, ramp);
    lfo_restart(lfo);
    return PCL_OK;
}

void pcl_lfo_reset(pcl_lfo_t *lfo) { lfo_restart(lfo); }

pcl_status_t pcl_lfo_set_shape(pcl_lfo_t *lfo, pcl_lfo_shape_t shape)
{
    if (lfo == NULL)
        return PCL_ERR_NULL_POINTER;
    if ((unsigned)shape >= (unsigned)PCL_LFO_SHAPE_COUNT)
        return PCL_ERR_OUT_OF_RANGE;
    lfo->shape = (uint8_t)shape;
    return PCL_OK;
}

pcl_status_t pcl_lfo_set_frequency(pcl_lfo_t *lfo, pcl_hz_t frequency)
{
    if (lfo == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!lfo_hz_valid(frequency, lfo->sample_rate_hz))
        return PCL_ERR_OUT_OF_RANGE;
    pcl_smooth_set_target(&lfo->freq, lfo_hz_to_u(frequency, lfo->sample_rate_hz));
    return PCL_OK;
}

pcl_status_t pcl_lfo_set_rate_depth(pcl_lfo_t *lfo, pcl_oct_t octaves)
{
    if (lfo == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!lfo_depth_valid(octaves))
        return PCL_ERR_OUT_OF_RANGE;
    lfo->rate_depth = octaves;
    return PCL_OK;
}

pcl_status_t pcl_lfo_set_pulse_width(pcl_lfo_t *lfo, pcl_coef_t width)
{
    if (lfo == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!lfo_width_valid(width))
        return PCL_ERR_OUT_OF_RANGE;
    pcl_smooth_set_target(&lfo->width, width);
    return PCL_OK;
}

void pcl_lfo_set_unipolar(pcl_lfo_t *lfo, int unipolar) { lfo->unipolar = unipolar != 0 ? 1u : 0u; }

void pcl_lfo_set_phase(pcl_lfo_t *lfo, uint32_t phase)
{
    lfo->phase = phase;
    lfo->jump = 1u;
}

void pcl_lfo_retrigger(pcl_lfo_t *lfo) { pcl_lfo_set_phase(lfo, lfo->start_phase); }

void pcl_lfo_set_seed(pcl_lfo_t *lfo, uint32_t seed)
{
    lfo->seed = seed;
    lfo_restart(lfo);
}

uint32_t pcl_lfo_phase(const pcl_lfo_t *lfo) { return lfo->phase; }

void pcl_lfo_process(pcl_lfo_t *lfo, pcl_sample_t *out, const pcl_sample_t *rate_mod, size_t frames)
{
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

        if (lfo->pos == 0u)
            lfo_tick(lfo, rate_mod != NULL ? rate_mod[i] : (pcl_sample_t)0);

        n = (size_t)PCL_CONTROL_DIV - lfo->pos;
        if (n > frames - i)
            n = frames - i;

        cur = lfo->cur;
        step = lfo->step;
#if PCL_SAMPLE_TYPE == PCL_FLOAT
        t = (float)lfo->pos * (1.0f / (float)PCL_CONTROL_DIV);
#endif
        for (k = 0; k < n; k++)
        {
#if PCL_SAMPLE_TYPE == PCL_FLOAT
            out[i + k] = lfo_out(cur + step * t);
            t += 1.0f / (float)PCL_CONTROL_DIV; /* exact for a power-of-two PCL_CONTROL_DIV */
#else
            out[i + k] = lfo_out(cur);
            cur += step;
#endif
        }
        lfo->cur = cur;
        lfo->pos = (uint8_t)((lfo->pos + n) % (size_t)PCL_CONTROL_DIV);
        i += n;
    }
}
