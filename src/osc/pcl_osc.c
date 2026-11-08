/*
 * Band-limited oscillator. See docs/modules/oscillator.md for the design.
 *
 * References: PolyBLEP / PolyBLAMP as in V. Valimaki et al., "Perceptually informed
 * synthesis of bandlimited classical waveforms using integrated polynomial
 * interpolation", JASA 131(1), 2012, and F. Esqueda, V. Valimaki, S. Bilbao,
 * "Rounding corners with BLAMP", DAFx-16.
 *
 * Correction maths. For an edge, let e = distance to the edge / window (0 <= e < 1,
 * window = one sample's phase increment) and q = 1 - e. A step of height J is corrected
 * by adding (J/2) * (+q^2) to the sample before the edge and (J/2) * (-q^2) to the sample
 * after it (this is the 2-point PolyBLEP residual). A slope change of M per cycle is
 * corrected by adding M * dt/6 * q^3 on both sides (its integral, the PolyBLAMP).
 *
 * Hard sync needs no division: the event is `delta` samples before the current sample,
 * so e = delta for the sample after it and e = 1 - delta for the sample before it.
 */
#include "pcl/pcl_osc.h"

#include "core/pcl_octave.h"
#include "core/pcl_tables.h"
#include "core/pcl_wave.h"

/* ---- Constants ---- */

#define OSC_PHASE_HALF 0x80000000u
#define OSC_PHASE_QUARTER 0x40000000u
#define OSC_INC_MAX 1932735283u /* 0.45 * 2^32: keeps PolyBLEP windows below half a cycle */
#define OSC_HZ_Q16_FRAC_BITS 16
#define OSC_FN_FRAC_BITS 30 /* normalized frequency is stored in Q2.30 in fixed builds */
#define OSC_MAX_FREQ_PERCENT 45u

/* K - 2 for K = 2*pi/3, as Q0.31: lets (2*pi/3) * x be formed as 2*x + x*(K-2). */
#define OSC_K_2PI_3_MINUS_2_Q31 202711939

#define OSC_WIDTH_MIN PCL_COEF(0.01)
#define OSC_WIDTH_MAX PCL_COEF(0.99)
#define OSC_PWM_DEPTH_MAX PCL_COEF(0.49)

/* Smallest non-zero sync fraction: 2^-15 in every build. */
#if PCL_SAMPLE_TYPE == PCL_FLOAT
#define OSC_SYNC_MIN (1.0f / 32768.0f)
#elif PCL_SAMPLE_TYPE == PCL_Q15
#define OSC_SYNC_MIN ((pcl_sample_t)1)
#else
#define OSC_SYNC_MIN ((pcl_sample_t)65536)
#endif

PCL_STATIC_ASSERT(PCL_CONTROL_DIV >= 1 && PCL_CONTROL_DIV <= 255, control_div_fits_uint8);

/* ---- Numeric layer: everything below is written once for all sample types ---- */

static pcl_sample_t osc_add(pcl_sample_t a, pcl_sample_t b)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return a + b;
#elif PCL_SAMPLE_TYPE == PCL_Q15
    return pcl_sat_q15((int32_t)a + (int32_t)b);
#else
    return pcl_sat_q31((int64_t)a + (int64_t)b);
#endif
}

static pcl_sample_t osc_sub(pcl_sample_t a, pcl_sample_t b)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return a - b;
#elif PCL_SAMPLE_TYPE == PCL_Q15
    return pcl_sat_q15((int32_t)a - (int32_t)b);
#else
    return pcl_sat_q31((int64_t)a - (int64_t)b);
#endif
}

static pcl_sample_t osc_mul(pcl_sample_t a, pcl_sample_t b)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return a * b;
#elif PCL_SAMPLE_TYPE == PCL_Q15
    return pcl_mul_q15(a, b);
#else
    return pcl_mul_q31(a, b);
#endif
}

static pcl_sample_t osc_half(pcl_sample_t a)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return 0.5f * a;
#elif PCL_SAMPLE_TYPE == PCL_Q15
    return (pcl_sample_t)(a >> 1);
#else
    return a >> 1;
#endif
}

/* 1 - x for a fraction x in [0, 1] (fixed point: the largest value stands in for 1). */
static pcl_sample_t osc_comp(pcl_sample_t x)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return 1.0f - x;
#elif PCL_SAMPLE_TYPE == PCL_Q15
    return (pcl_sample_t)(INT16_MAX - x);
#else
    return INT32_MAX - x;
#endif
}

static pcl_sample_t osc_sample_max(void)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return 1.0f;
#elif PCL_SAMPLE_TYPE == PCL_Q15
    return INT16_MAX;
#else
    return INT32_MAX;
#endif
}

static pcl_sample_t osc_sample_min(void)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return -1.0f;
#elif PCL_SAMPLE_TYPE == PCL_Q15
    return INT16_MIN;
#else
    return INT32_MIN;
#endif
}

/* A Q1.31 value to the sample format. */
static pcl_sample_t osc_from_w31(int32_t w)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return (float)w * (1.0f / 2147483648.0f);
#elif PCL_SAMPLE_TYPE == PCL_Q15
    return (pcl_sample_t)(w >> 16);
#else
    return w;
#endif
}

#if PCL_SAMPLE_TYPE != PCL_FLOAT
/* Leading zero count, for normalizing the window reciprocal. */
static uint32_t osc_clz32(uint32_t x)
{
    uint32_t n = 0;
    if (x <= 0x0000FFFFu)
    {
        n += 16u;
        x <<= 16;
    }
    if (x <= 0x00FFFFFFu)
    {
        n += 8u;
        x <<= 8;
    }
    if (x <= 0x0FFFFFFFu)
    {
        n += 4u;
        x <<= 4;
    }
    if (x <= 0x3FFFFFFFu)
    {
        n += 2u;
        x <<= 2;
    }
    if (x <= 0x7FFFFFFFu)
        n += 1u;
    return n;
}
#endif

/*
 * Window reciprocal. All band-limiting needs x = d / win for a distance d < win, so the
 * reciprocal of win is computed once when the window changes and the per-sample path has
 * no division. Fixed point keeps it as a normalized mantissa: with s = clz(win) and
 * wn = win << s in [2^31, 2^32), x = (d << s) / wn.
 *   Q15: recip = 2^31 / (wn >> 16)   (16 bits), x(Q0.31) = ((d << s) >> 16) * recip
 *   Q31: recip = 2^63 / wn           (32 bits), x(Q0.31) = ((d << s) * recip) >> 32
 */
static void osc_window_update(pcl_osc_t *o, uint32_t win)
{
    o->win = win;
    o->recip_shift = 0u;
    if (win == 0u)
    {
        o->recip = 0;
        o->k_tri = 0;
        o->k_sine = 0;
        return;
    }

#if PCL_SAMPLE_TYPE == PCL_FLOAT
    {
        const float dt = (float)win * (1.0f / 4294967296.0f);
        o->recip = 1.0f / (float)win;
        o->k_tri = (4.0f / 3.0f) * dt;
        o->k_sine = 2.09439510f * dt; /* 2*pi/3 */
    }
#else
    {
        const uint32_t shift = osc_clz32(win);
        const uint32_t wn = win << shift;
        const int32_t  dt31 = (int32_t)(win >> 1); /* window as Q0.31, <= 0.45 */
        const int32_t  k_tri31 = dt31 + dt31 / 3;
        const int32_t  k_sine31 = (dt31 << 1) + pcl_mul_q31(dt31, OSC_K_2PI_3_MINUS_2_Q31);

        o->recip_shift = shift;
#if PCL_SAMPLE_TYPE == PCL_Q15
        o->recip = 0x80000000u / (wn >> 16);
        o->k_tri = (pcl_sample_t)(k_tri31 >> 16);
        o->k_sine = (pcl_sample_t)(k_sine31 >> 16);
#else
        {
            const uint64_t r = ((uint64_t)1 << 63) / wn;
            o->recip = r > 0xFFFFFFFFu ? 0xFFFFFFFFu : (uint32_t)r;
        }
        o->k_tri = k_tri31;
        o->k_sine = k_sine31;
#endif
    }
#endif
}

/* d / win as a sample-format fraction in [0, 1], for 0 <= d < win. */
static pcl_sample_t osc_frac(const pcl_osc_t *o, uint32_t d)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    const float x = (float)d * o->recip;
    return x < 1.0f ? x : 1.0f;
#elif PCL_SAMPLE_TYPE == PCL_Q15
    const uint32_t x = (((d << o->recip_shift) >> 16) * o->recip) >> 16;
    return (pcl_sample_t)(x > (uint32_t)INT16_MAX ? (uint32_t)INT16_MAX : x);
#else
    const uint32_t x = (uint32_t)(((uint64_t)(d << o->recip_shift) * o->recip) >> 32);
    return (pcl_sample_t)(x > (uint32_t)INT32_MAX ? (uint32_t)INT32_MAX : x);
#endif
}

/* inc * delta for a fraction delta in [0, 1): how far the phase moves in `delta` samples. */
static uint32_t osc_inc_times(uint32_t inc, pcl_sample_t delta)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return (uint32_t)((float)inc * delta);
#elif PCL_SAMPLE_TYPE == PCL_Q15
    const uint32_t hi = inc >> 16;
    const uint32_t lo = inc & 0xFFFFu;
    const uint32_t d = (uint32_t)(int32_t)delta; /* 0 .. 32767 */
    return ((hi * d) << 1) + ((lo * d) >> 15);
#else
    return (uint32_t)(((uint64_t)inc * (uint32_t)delta) >> 31);
#endif
}

/* ---- Waveforms ---- */

/* Quarter-wave sine with linear interpolation; phase 2^32 = one cycle (shared with the LFO). */
static pcl_sample_t osc_sine(uint32_t phase) { return pcl_wave_sine(phase); }

/* Rising ramp -1 .. 1 as Q1.31, and the triangle (-1 at phase 0, +1 at 0.5) as Q1.31. */
static int32_t osc_saw_w31(uint32_t phase) { return pcl_wave_saw_w31(phase); }

static int32_t osc_tri_w31(uint32_t phase) { return pcl_wave_tri_w31(phase); }

/* The uncorrected waveform at an arbitrary phase. */
static pcl_sample_t osc_naive(const pcl_osc_t *o, uint32_t phase)
{
    switch (o->waveform)
    {
    case PCL_OSC_SINE:
        return osc_sine(phase);
    case PCL_OSC_TRIANGLE:
        return osc_from_w31(osc_tri_w31(phase));
    case PCL_OSC_SAW:
        return osc_from_w31(osc_saw_w31(phase));
    default: /* PCL_OSC_PULSE */
        return phase < (o->thr31 << 1) ? osc_sample_max() : osc_sample_min();
    }
}

/* q = 1 - d/win for the window sample at distance d from an edge. */
static pcl_sample_t osc_q(const pcl_osc_t *o, uint32_t d) { return osc_comp(osc_frac(o, d)); }

/*
 * Adds a step correction for an edge at phase `edge`. `half_j` is +1 or -1 for an upward
 * or downward step of height 2 (J/2). Samples after the edge get half_j * (-q^2), samples
 * before it half_j * (+q^2). `skip_post` suppresses the after-edge half (used right
 * after a sync reset, whose own correction replaces it).
 */
static pcl_sample_t osc_step_edge(
    const pcl_osc_t *o, pcl_sample_t y, uint32_t edge, int half_j, int skip_post)
{
    uint32_t d = o->phase - edge;
    if (d < o->win)
    {
        if (!skip_post)
        {
            const pcl_sample_t q = osc_q(o, d);
            const pcl_sample_t q2 = osc_mul(q, q);
            y = half_j > 0 ? osc_sub(y, q2) : osc_add(y, q2);
        }
        return y;
    }
    d = edge - o->phase;
    if (d < o->win)
    {
        const pcl_sample_t q = osc_q(o, d);
        const pcl_sample_t q2 = osc_mul(q, q);
        y = half_j > 0 ? osc_add(y, q2) : osc_sub(y, q2);
    }
    return y;
}

/* Adds a corner (slope change) correction: sign * k_tri * q^3, both sides of the corner. */
static pcl_sample_t osc_ramp_edge(
    const pcl_osc_t *o, pcl_sample_t y, uint32_t edge, int sign, int skip_post)
{
    uint32_t d = o->phase - edge;
    if (d < o->win)
    {
        if (skip_post)
            return y;
    }
    else
    {
        d = edge - o->phase;
        if (d >= o->win)
            return y;
    }
    {
        const pcl_sample_t q = osc_q(o, d);
        const pcl_sample_t q3 = osc_mul(osc_mul(q, q), q);
        const pcl_sample_t corr = osc_mul(o->k_tri, q3);
        return sign > 0 ? osc_add(y, corr) : osc_sub(y, corr);
    }
}

/* The band-limited waveform at the oscillator's current phase. */
static pcl_sample_t osc_wave(const pcl_osc_t *o, int skip_edge0_post)
{
    pcl_sample_t y = osc_naive(o, o->phase);

    switch (o->waveform)
    {
    case PCL_OSC_TRIANGLE:
        y = osc_ramp_edge(o, y, 0u, +1, skip_edge0_post);
        y = osc_ramp_edge(o, y, OSC_PHASE_HALF, -1, 0);
        break;
    case PCL_OSC_SAW:
        y = osc_step_edge(o, y, 0u, -1, skip_edge0_post);
        break;
    case PCL_OSC_PULSE:
        y = osc_step_edge(o, y, 0u, +1, skip_edge0_post);
        y = osc_step_edge(o, y, o->thr31 << 1, -1, 0);
        break;
    default: /* PCL_OSC_SINE needs no correction */
        break;
    }
    return y;
}

/* ---- Hard sync ---- */

/* (1 - cos) / 2 at a phase, in [0, 1], from the sine table. */
static pcl_sample_t osc_half_one_minus_cos(uint32_t phase)
{
    const pcl_sample_t c = osc_sine(phase + OSC_PHASE_QUARTER);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return 0.5f * (1.0f - c);
#elif PCL_SAMPLE_TYPE == PCL_Q15
    return (pcl_sample_t)((INT16_MAX >> 1) - (c >> 1));
#else
    return (INT32_MAX >> 1) - (c >> 1);
#endif
}

/*
 * Applies a sync event that happened `delta` samples before the current sample: the slave
 * restarted from phase 0 then. Resets the phase to where it has got to since, and returns
 * the corrections for the current sample (`y_adj`) and the previous, still-held one
 * (`held_adj`).
 */
static void osc_sync(pcl_osc_t *o, pcl_sample_t delta, pcl_sample_t *y_adj, pcl_sample_t *held_adj)
{
    const uint32_t     advanced = osc_inc_times(o->inc, delta);
    const uint32_t     before_phase = o->phase - advanced; /* where the phase was at the event */
    const pcl_sample_t before = osc_naive(o, before_phase);
    const pcl_sample_t after = osc_naive(o, 0u);
    const pcl_sample_t half_j = osc_sub(osc_half(after), osc_half(before));
    const pcl_sample_t q_post = osc_comp(delta); /* current sample: e = delta       */
    const pcl_sample_t q_pre = delta;            /* previous sample: e = 1 - delta  */
    const pcl_sample_t q2_post = osc_mul(q_post, q_post);
    const pcl_sample_t q2_pre = osc_mul(q_pre, q_pre);
    pcl_sample_t       slope = 0;

    *y_adj = osc_sub((pcl_sample_t)0, osc_mul(half_j, q2_post));
    *held_adj = osc_mul(half_j, q2_pre);

    /* Slope change at the event: only the triangle and the sine have one. */
    if (o->waveform == PCL_OSC_TRIANGLE)
    {
        if (before_phase & OSC_PHASE_HALF) /* was falling, now rising */
            slope = o->k_tri;
    }
    else if (o->waveform == PCL_OSC_SINE)
    {
        slope = osc_mul(osc_half_one_minus_cos(before_phase), o->k_sine);
    }
    if (slope != 0)
    {
        const pcl_sample_t q3_post = osc_mul(q2_post, q_post);
        const pcl_sample_t q3_pre = osc_mul(q2_pre, q_pre);
        *y_adj = osc_add(*y_adj, osc_mul(slope, q3_post));
        *held_adj = osc_add(*held_adj, osc_mul(slope, q3_pre));
    }

    o->phase = advanced;
}

/* ---- Control rate: pitch, width, window ---- */

static uint32_t osc_clamp_inc(uint32_t inc) { return inc > OSC_INC_MAX ? OSC_INC_MAX : inc; }

/* Phase increment for the smoothed frequency `fn` (cycles/sample) bent by `mod` octaves. */
static uint32_t osc_pitch_inc(const pcl_osc_t *o, pcl_coef_t fn, pcl_sample_t mod)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    float f = fn;

    if (mod != 0.0f && o->pitch_depth != 0.0f)
        f = pcl_octave_scale_f(f, o->pitch_depth * mod);
    if (f > 0.45f)
        f = 0.45f;
    return osc_clamp_inc((uint32_t)(f * 4294967296.0f));
#else
    const uint32_t base = (uint32_t)fn << (32 - OSC_FN_FRAC_BITS);

    if (mod == 0 || o->pitch_depth == 0)
        return osc_clamp_inc(base);

    {
#if PCL_SAMPLE_TYPE == PCL_Q15
        const int32_t oct = pcl_mul_coef_q15(o->pitch_depth, mod); /* Q4.28 */
#else
        const int32_t oct = pcl_mul_coef_q31(o->pitch_depth, mod);
#endif
        return osc_clamp_inc(pcl_octave_scale_inc(base, oct, OSC_INC_MAX));
    }
#endif
}

/*
 * Pulse threshold (Q0.31 fraction of a cycle) for the smoothed width `w` plus `mod` times
 * the PWM depth, clamped so that the two edges' windows (win each side) cannot overlap.
 */
static uint32_t osc_width_thr(const pcl_osc_t *o, pcl_coef_t w, pcl_sample_t mod, uint32_t win)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    float pw = w;
    float lo;
    float hi;

    if (mod != 0.0f && o->pwm_depth != 0.0f)
        pw += o->pwm_depth * mod;
    lo = (float)win * (2.0f / 4294967296.0f); /* 2 * dt */
    if (lo < 0.01f)
        lo = 0.01f;
    hi = 1.0f - lo;
    pw = (lo > hi) ? 0.5f : (pw < lo ? lo : (pw > hi ? hi : pw));
    return (uint32_t)(pw * 2147483648.0f);
#else
    int32_t pw = w; /* Q2.30 */
    int32_t lo;
    int32_t hi;

    if (mod != 0 && o->pwm_depth != 0)
    {
#if PCL_SAMPLE_TYPE == PCL_Q15
        pw += pcl_mul_coef_q15(o->pwm_depth, mod);
#else
        pw += pcl_mul_coef_q31(o->pwm_depth, mod);
#endif
    }
    lo = (int32_t)(win >> 1); /* 2 * dt in Q2.30 */
    if (lo < OSC_WIDTH_MIN)
        lo = OSC_WIDTH_MIN;
    hi = (1 << PCL_Q30_SHIFT) - lo;
    pw = (lo > hi) ? (1 << (PCL_Q30_SHIFT - 1)) : (pw < lo ? lo : (pw > hi ? hi : pw));
    return (uint32_t)pw << 1;
#endif
}

/* Computes the steady-state targets (no modulation) and settles on them. */
static void osc_prepare(pcl_osc_t *o)
{
    const uint32_t inc = osc_pitch_inc(o, o->freq.current, 0);

    o->inc = inc;
    o->inc_target = inc;
    o->inc_step = 0;
    osc_window_update(o, inc);
    o->thr31 = osc_width_thr(o, o->width.current, 0, inc);
    o->thr_target = o->thr31;
    o->thr_step = 0;
    o->held = 0;
    o->sync_pending = 0;
    o->ctrl_count = 0u;
}

/* One control period begins: sample the modulation and aim the interpolators at it. */
static void osc_tick(pcl_osc_t *o, pcl_sample_t pitch_mod, pcl_sample_t pwm_mod)
{
    const pcl_coef_t fn = pcl_smooth_next(&o->freq);
    const pcl_coef_t w = pcl_smooth_next(&o->width);
    uint32_t         inc_next;
    uint32_t         win;
    uint32_t         thr_next;

    /* The previous period was heading for these; land on them exactly. */
    o->inc = o->inc_target;
    o->thr31 = o->thr_target;

    inc_next = osc_pitch_inc(o, fn, pitch_mod);
    win = inc_next > o->inc ? inc_next : o->inc;
    if (win != o->win)
        osc_window_update(o, win);
    thr_next = osc_width_thr(o, w, pwm_mod, win);

    o->inc_target = inc_next;
    o->inc_step = ((int32_t)inc_next - (int32_t)o->inc) / (int32_t)PCL_CONTROL_DIV;
    o->thr_target = thr_next;
    o->thr_step = ((int32_t)thr_next - (int32_t)o->thr31) / (int32_t)PCL_CONTROL_DIV;
    o->ctrl_count = (uint8_t)PCL_CONTROL_DIV;
}

/* Moves the phase on one sample and notes whether it wrapped. */
static void osc_advance(pcl_osc_t *o, int want_sync_out)
{
    const uint32_t old = o->phase;

    o->phase = old + o->inc;
    o->sync_pending = 0;
    if (want_sync_out && o->phase < old)
    {
        /* The wrap happened phase/inc of a sample ago. */
        const pcl_sample_t d = osc_frac(o, o->phase);
        o->sync_pending = d < OSC_SYNC_MIN ? OSC_SYNC_MIN : d;
    }
    o->inc += (uint32_t)o->inc_step;
    o->thr31 += (uint32_t)o->thr_step;
}

/* ---- Public API ---- */

static uint32_t osc_ramp_ticks(uint32_t sample_rate_hz)
{
    const uint32_t ticks = pcl_smooth_ramp_samples(sample_rate_hz) / (uint32_t)PCL_CONTROL_DIV;
    return ticks > 0u ? ticks : 1u;
}

static int osc_hz_valid(pcl_hz_t hz, uint32_t sample_rate_hz)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    /* Written positively so that NaN fails both comparisons. */
    return hz >= 0.0f && hz <= 0.45f * (float)sample_rate_hz;
#else
    return (uint64_t)hz * 100u <= ((uint64_t)sample_rate_hz * OSC_MAX_FREQ_PERCENT)
        << OSC_HZ_Q16_FRAC_BITS;
#endif
}

/* Hz to cycles/sample, in pcl_coef_t. */
static pcl_coef_t osc_hz_to_fn(pcl_hz_t hz, uint32_t sample_rate_hz)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return hz / (float)sample_rate_hz;
#else
    /* hz is Q16.16: fn(Q2.30) = hz * 2^14 / sample_rate. 64-bit integer divide, setter only. */
    return (pcl_coef_t)((((uint64_t)hz << (OSC_FN_FRAC_BITS - OSC_HZ_Q16_FRAC_BITS))
                            + (sample_rate_hz / 2u))
        / sample_rate_hz);
#endif
}

static int osc_width_valid(pcl_coef_t w) { return w >= OSC_WIDTH_MIN && w <= OSC_WIDTH_MAX; }

static int osc_pwm_depth_valid(pcl_coef_t d)
{
    return d >= PCL_COEF(0.0) && d <= OSC_PWM_DEPTH_MAX;
}

static int osc_pitch_depth_valid(pcl_oct_t d)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return d >= 0.0f && d < 8.0f;
#else
    return d >= 0; /* Q4.28 cannot represent 8 or more */
#endif
}

pcl_status_t pcl_osc_init(pcl_osc_t *osc, const pcl_osc_config_t *cfg)
{
    uint32_t ramp;

    if (osc == NULL || cfg == NULL)
        return PCL_ERR_NULL_POINTER;
    if (cfg->sample_rate_hz < PCL_SAMPLE_RATE_MIN_HZ || cfg->sample_rate_hz > PCL_SAMPLE_RATE_MAX_HZ
        || (unsigned)cfg->waveform >= (unsigned)PCL_OSC_WAVEFORM_COUNT
        || !osc_hz_valid(cfg->frequency, cfg->sample_rate_hz)
        || !osc_pitch_depth_valid(cfg->pitch_depth_oct) || !osc_width_valid(cfg->pulse_width)
        || !osc_pwm_depth_valid(cfg->pwm_depth))
        return PCL_ERR_OUT_OF_RANGE;

    ramp = osc_ramp_ticks(cfg->sample_rate_hz);
    osc->sample_rate_hz = cfg->sample_rate_hz;
    osc->waveform = (uint8_t)cfg->waveform;
    osc->pitch_depth = cfg->pitch_depth_oct;
    osc->pwm_depth = cfg->pwm_depth;
    pcl_smooth_init(&osc->freq, osc_hz_to_fn(cfg->frequency, cfg->sample_rate_hz), ramp);
    pcl_smooth_init(&osc->width, cfg->pulse_width, ramp);
    osc->phase = 0u;
    osc_prepare(osc);
    return PCL_OK;
}

void pcl_osc_reset(pcl_osc_t *osc)
{
    pcl_smooth_reset(&osc->freq);
    pcl_smooth_reset(&osc->width);
    osc->phase = 0u;
    osc_prepare(osc);
}

pcl_status_t pcl_osc_set_frequency(pcl_osc_t *osc, pcl_hz_t hz)
{
    if (osc == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!osc_hz_valid(hz, osc->sample_rate_hz))
        return PCL_ERR_OUT_OF_RANGE;
    pcl_smooth_set_target(&osc->freq, osc_hz_to_fn(hz, osc->sample_rate_hz));
    return PCL_OK;
}

pcl_status_t pcl_osc_set_waveform(pcl_osc_t *osc, pcl_osc_waveform_t waveform)
{
    if (osc == NULL)
        return PCL_ERR_NULL_POINTER;
    if ((unsigned)waveform >= (unsigned)PCL_OSC_WAVEFORM_COUNT)
        return PCL_ERR_OUT_OF_RANGE;
    osc->waveform = (uint8_t)waveform;
    return PCL_OK;
}

pcl_status_t pcl_osc_set_pitch_depth(pcl_osc_t *osc, pcl_oct_t octaves)
{
    if (osc == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!osc_pitch_depth_valid(octaves))
        return PCL_ERR_OUT_OF_RANGE;
    osc->pitch_depth = octaves;
    return PCL_OK;
}

pcl_status_t pcl_osc_set_pulse_width(pcl_osc_t *osc, pcl_coef_t width)
{
    if (osc == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!osc_width_valid(width))
        return PCL_ERR_OUT_OF_RANGE;
    pcl_smooth_set_target(&osc->width, width);
    return PCL_OK;
}

pcl_status_t pcl_osc_set_pwm_depth(pcl_osc_t *osc, pcl_coef_t depth)
{
    if (osc == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!osc_pwm_depth_valid(depth))
        return PCL_ERR_OUT_OF_RANGE;
    osc->pwm_depth = depth;
    return PCL_OK;
}

void pcl_osc_set_phase(pcl_osc_t *osc, uint32_t phase) { osc->phase = phase; }

void pcl_osc_process(pcl_osc_t *osc, pcl_sample_t *out, const pcl_sample_t *pitch_mod,
    const pcl_sample_t *pwm_mod, const pcl_sample_t *sync_in, pcl_sample_t *sync_out, size_t frames)
{
    /* Work on a local copy so stores to the buffers cannot force state reloads. */
    pcl_osc_t o = *osc;
    size_t    i;

    for (i = 0; i < frames; i++)
    {
        pcl_sample_t y_adj = 0;
        pcl_sample_t held_adj = 0;
        pcl_sample_t sync_report;
        int          synced = 0;

        if (o.ctrl_count == 0u)
            osc_tick(&o, pitch_mod != NULL ? pitch_mod[i] : (pcl_sample_t)0,
                pwm_mod != NULL ? pwm_mod[i] : (pcl_sample_t)0);
        o.ctrl_count = (uint8_t)(o.ctrl_count - 1u);

        sync_report = o.sync_pending;
        if (sync_in != NULL && sync_in[i] != 0)
        {
            osc_sync(&o, sync_in[i], &y_adj, &held_adj);
            sync_report = sync_in[i]; /* a forced restart is a phase return too */
            synced = 1;
        }
        if (sync_out != NULL)
            sync_out[i] = sync_report;

        out[i] = osc_add(o.held, held_adj);
        o.held = osc_add(osc_wave(&o, synced), y_adj);
        osc_advance(&o, sync_out != NULL);
    }

    *osc = o;
}
