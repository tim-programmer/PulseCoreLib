/*
 * pcl_unison.c - unison oscillator built from pcl_osc_t voices. Design note:
 * docs/modules/unison.md.
 *
 * Detune spread: linear in octaves (equal pitch steps). Pan: constant power,
 * g_L = cos((pan + 1) * pi/4), g_R = sin((pan + 1) * pi/4) (standard practice, e.g. C. Roads,
 * "The Computer Music Tutorial", 1996). Mix: 1 / sqrt(N), so uncorrelated voices keep the level
 * of one voice. Supersaw background: A. Szabo, "How to Emulate the Super Saw", 2010.
 */
#include "pcl/pcl_unison.h"

#include "core/pcl_octave.h"
#include "core/pcl_rng.h"
#include "pcl/pcl_math.h"
#include "pcl/pcl_smooth.h"

PCL_STATIC_ASSERT(PCL_UNISON_MAX_VOICES >= 1 && PCL_UNISON_MAX_VOICES <= 16, unison_max_voices);
PCL_STATIC_ASSERT(PCL_UNISON_CHUNK >= 1 && PCL_UNISON_CHUNK <= 256, unison_chunk);

#define UNISON_MAX_FREQ_PERCENT 45u /* voices are held at 0.45 * sample rate, as pcl_osc  */
#define UNISON_HZ_FRAC_BITS 16      /* pcl_hz_t is Q16.16 in fixed-point builds          */
#define UNISON_PAN_PHASE_SHIFT 1    /* (pan + 1) in Q2.30 -> phase (pan + 1) * 2^29      */
#define UNISON_Q15_TO_Q30_SHIFT 15

/* 1 / sqrt(n) for n = 1 .. 16. */
static const pcl_coef_t unison_mix_tab[16] = {
    PCL_COEF(1.0),
    PCL_COEF(0.70710678118654752),
    PCL_COEF(0.57735026918962576),
    PCL_COEF(0.5),
    PCL_COEF(0.44721359549995794),
    PCL_COEF(0.40824829046386302),
    PCL_COEF(0.37796447300922723),
    PCL_COEF(0.35355339059327376),
    PCL_COEF(0.33333333333333333),
    PCL_COEF(0.31622776601683793),
    PCL_COEF(0.30151134457776362),
    PCL_COEF(0.28867513459481288),
    PCL_COEF(0.27735009811261456),
    PCL_COEF(0.26726124191242440),
    PCL_COEF(0.25819888974716112),
    PCL_COEF(0.25),
};

#if PCL_SAMPLE_TYPE == PCL_FLOAT
typedef float unison_acc_t;
#elif PCL_SAMPLE_TYPE == PCL_Q15
typedef int32_t unison_acc_t;
#else
typedef int64_t unison_acc_t;
#endif

/* ---- Helpers ---- */

/* Sample times gain (pcl_coef_t), rounded and saturated in fixed point. */
static unison_acc_t unison_scale(pcl_sample_t x, pcl_coef_t g)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return x * g;
#elif PCL_SAMPLE_TYPE == PCL_Q15
    return (unison_acc_t)pcl_mul_q15_q30(x, g);
#else
    return (unison_acc_t)pcl_mul_q31_q30(x, g);
#endif
}

static pcl_sample_t unison_saturate(unison_acc_t acc)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return acc;
#elif PCL_SAMPLE_TYPE == PCL_Q15
    return pcl_sat_q15(acc);
#else
    return pcl_sat_q31(acc);
#endif
}

/* Coefficient product a * b (both pcl_coef_t). Setter only. */
static pcl_coef_t unison_coef_mul(pcl_coef_t a, pcl_coef_t b)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return a * b;
#else
    return pcl_sat_q31(
        ((int64_t)a * (int64_t)b + ((int64_t)1 << (PCL_Q30_SHIFT - 1))) >> PCL_Q30_SHIFT);
#endif
}

/* A full-scale sample (pcl_sin/pcl_cos result) as a pcl_coef_t. */
static pcl_coef_t unison_sample_to_coef(pcl_sample_t s)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return s;
#elif PCL_SAMPLE_TYPE == PCL_Q15
    return (pcl_coef_t)s * (1 << UNISON_Q15_TO_Q30_SHIFT);
#else
    return s >> 1; /* Q1.31 -> Q2.30; |s| <= 1, so the shift cannot lose the sign */
#endif
}

static int unison_hz_valid(pcl_hz_t hz, uint32_t sample_rate_hz)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return hz >= 0.0f && hz <= 0.45f * (float)sample_rate_hz; /* NaN fails both */
#else
    return (uint64_t)hz * 100u <= ((uint64_t)sample_rate_hz * UNISON_MAX_FREQ_PERCENT)
        << UNISON_HZ_FRAC_BITS;
#endif
}

static int unison_detune_valid(pcl_oct_t d)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return d >= 0.0f && d <= 1.0f;
#else
    return d >= 0 && d <= PCL_OCT(1.0);
#endif
}

static int unison_width_valid(pcl_coef_t w) { return w >= PCL_COEF(0.0) && w <= PCL_COEF(1.0); }

static int unison_voices_valid(size_t n) { return n >= 1u && n <= (size_t)PCL_UNISON_MAX_VOICES; }

/*
 * Position of voice i of n as a fraction of the spread, times `amount`: amount * (2i - (n-1))
 * / den, with den = 2 (n-1) for the detune (+/- amount/2) and (n-1) for the pan (+/- amount).
 * n == 1 gives 0.
 */
#if PCL_SAMPLE_TYPE == PCL_FLOAT
static float unison_spread(float amount, size_t i, size_t n, int half)
{
    const int k = 2 * (int)i - ((int)n - 1);
    const int den = half ? 2 * ((int)n - 1) : (int)n - 1;

    return n > 1u ? amount * (float)k / (float)den : 0.0f;
}
#else
static int32_t unison_spread(int32_t amount, size_t i, size_t n, int half)
{
    const int64_t k = 2 * (int64_t)i - ((int64_t)n - 1);
    const int64_t den = half ? 2 * ((int64_t)n - 1) : (int64_t)n - 1;

    /* Division truncates toward zero, so voices i and n-1-i get exactly opposite values. */
    return n > 1u ? (int32_t)((int64_t)amount * k / den) : 0;
}
#endif

/* Base frequency times 2^oct (|oct| <= 1/2), held at 0.45 * sample rate. Setter only. */
static pcl_hz_t unison_voice_hz(pcl_hz_t base, pcl_oct_t oct, uint32_t sample_rate_hz)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    const float hz = pcl_octave_scale_f(base, oct);
    const float hz_max = 0.45f * (float)sample_rate_hz;

    return hz > hz_max ? hz_max : hz;
#else
    /* oct in Q4.28: integer part k (-1 or 0 here), fraction f in [0, 1). 64-bit, setter only:
       pcl_octave_scale_inc would overflow for bases above 32768 Hz. */
    const int32_t  k = oct >> PCL_Q28_SHIFT;
    const uint32_t mant = pcl_exp2m1_q16((uint32_t)oct & ((1u << PCL_Q28_SHIFT) - 1u));
    uint64_t       hz = ((uint64_t)base * (65536u + mant) + 0x8000u) >> 16;
    uint64_t       hz_max
        = (((uint64_t)sample_rate_hz * UNISON_MAX_FREQ_PERCENT) << UNISON_HZ_FRAC_BITS) / 100u;

    if (k < 0)
        hz >>= -k;
    else
        hz <<= k;
    if (hz_max > UINT32_MAX)
        hz_max = UINT32_MAX;
    return (pcl_hz_t)(hz > hz_max ? hz_max : hz);
#endif
}

/* Pushes the detuned frequency of every voice (active or not) to its oscillator. */
static void unison_apply_frequencies(pcl_unison_t *u)
{
    size_t i;

    for (i = 0; i < (size_t)PCL_UNISON_MAX_VOICES; i++)
    {
        const size_t    n = u->voices;
        const pcl_oct_t off = i < n ? unison_spread(u->detune, i, n, 1) : 0;
        /* Valid by construction: unison_voice_hz holds it within the oscillator's range. */
        (void)pcl_osc_set_frequency(
            &u->voice[i], unison_voice_hz(u->base_hz, off, u->voice[i].sample_rate_hz));
    }
}

/* Constant-power left gain cos((pan + 1) * pi/4) for pan in [-1, 1]. Setter only. */
static pcl_coef_t unison_pan_gain(pcl_coef_t pan)
{
    uint32_t phase;

#if PCL_SAMPLE_TYPE == PCL_FLOAT
    phase = (uint32_t)((pan + 1.0f) * 536870912.0f); /* (pan + 1) / 8 of a cycle */
#else
    /* pan + 1 reaches 2^31 at pan = +1, so add in unsigned arithmetic. */
    phase = ((uint32_t)pan + (uint32_t)PCL_COEF(1.0)) >> UNISON_PAN_PHASE_SHIFT;
#endif
    return unison_sample_to_coef(pcl_cos(phase));
}

/* Target pan gains of the active voices for the current width and count. */
static void unison_compute_targets(pcl_unison_t *u)
{
    const size_t     n = u->voices;
    const pcl_coef_t mix = unison_mix_tab[n - 1u];
    size_t           i;

    for (i = 0; i < n; i++)
    {
        const pcl_coef_t pan = unison_spread(u->width, i, n, 0); /* -1 .. 1 */

        /* g_R(pan) = g_L(-pan) (sin x = cos(pi/2 - x)), so both channels come from the same
           table points: width 0 gives L == R exactly, and the field is mirror-symmetric. */
        u->target_l[i] = unison_coef_mul(mix, unison_pan_gain(pan));
        u->target_r[i] = unison_coef_mul(mix, unison_pan_gain(-pan));
    }
}

static void unison_snap_gains(pcl_unison_t *u)
{
    size_t i;

    for (i = 0; i < (size_t)u->voices; i++)
    {
        u->gain_l[i] = u->target_l[i];
        u->gain_r[i] = u->target_r[i];
        u->step_l[i] = 0;
        u->step_r[i] = 0;
    }
    u->ramp_left = 0u;
}

static void unison_start_ramp(pcl_unison_t *u)
{
    const uint32_t r = u->ramp_samples;
    size_t         i;

    for (i = 0; i < (size_t)u->voices; i++)
    {
#if PCL_SAMPLE_TYPE == PCL_FLOAT
        u->step_l[i] = (u->target_l[i] - u->gain_l[i]) / (float)r;
        u->step_r[i] = (u->target_r[i] - u->gain_r[i]) / (float)r;
#else
        u->step_l[i] = (u->target_l[i] - u->gain_l[i]) / (int32_t)r;
        u->step_r[i] = (u->target_r[i] - u->gain_r[i]) / (int32_t)r;
#endif
    }
    u->ramp_left = r;
}

/* Seeded start phases of every voice. */
static void unison_set_phases(pcl_unison_t *u)
{
    uint32_t state = pcl_rng_seed(u->seed);
    size_t   i;

    for (i = 0; i < (size_t)PCL_UNISON_MAX_VOICES; i++)
        pcl_osc_set_phase(&u->voice[i], pcl_rng_next(&state));
}

/* Advances the width ramp by `frames` samples without rendering (mono calls). */
static void unison_skip_ramp(pcl_unison_t *u, uint32_t frames)
{
    size_t i;

    if (u->ramp_left == 0u)
        return;
    if (frames >= u->ramp_left)
    {
        unison_snap_gains(u);
        return;
    }
    for (i = 0; i < (size_t)u->voices; i++)
    {
#if PCL_SAMPLE_TYPE == PCL_FLOAT
        u->gain_l[i] += u->step_l[i] * (float)frames;
        u->gain_r[i] += u->step_r[i] * (float)frames;
#else
        u->gain_l[i] += u->step_l[i] * (int32_t)frames;
        u->gain_r[i] += u->step_r[i] * (int32_t)frames;
#endif
    }
    u->ramp_left -= frames;
}

/* ---- Public API ---- */

pcl_status_t pcl_unison_init(pcl_unison_t *u, const pcl_unison_config_t *cfg)
{
    pcl_status_t st;
    size_t       i;

    if (u == NULL || cfg == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!unison_voices_valid(cfg->voices) || !unison_detune_valid(cfg->detune_oct)
        || !unison_width_valid(cfg->stereo_width))
        return PCL_ERR_OUT_OF_RANGE;

    /* Validate the oscillator config on voice 0 before touching anything else. */
    {
        pcl_osc_t probe;
        st = pcl_osc_init(&probe, &cfg->osc);
        if (st != PCL_OK)
            return st;
    }

    for (i = 0; i < (size_t)PCL_UNISON_MAX_VOICES; i++)
        (void)pcl_osc_init(&u->voice[i], &cfg->osc);
    u->base_hz = cfg->osc.frequency;
    u->detune = cfg->detune_oct;
    u->width = cfg->stereo_width;
    u->seed = cfg->seed;
    u->voices = (uint8_t)cfg->voices;
    u->gain_mono = unison_mix_tab[cfg->voices - 1u];
    u->ramp_samples = pcl_smooth_ramp_samples(cfg->osc.sample_rate_hz);

    unison_apply_frequencies(u);
    unison_compute_targets(u);
    pcl_unison_reset(u);
    return PCL_OK;
}

void pcl_unison_reset(pcl_unison_t *u)
{
    size_t i;

    for (i = 0; i < (size_t)PCL_UNISON_MAX_VOICES; i++)
        pcl_osc_reset(&u->voice[i]);
    unison_set_phases(u);
    unison_snap_gains(u);
}

pcl_status_t pcl_unison_set_frequency(pcl_unison_t *u, pcl_hz_t hz)
{
    if (u == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!unison_hz_valid(hz, u->voice[0].sample_rate_hz))
        return PCL_ERR_OUT_OF_RANGE;
    u->base_hz = hz;
    unison_apply_frequencies(u);
    return PCL_OK;
}

pcl_status_t pcl_unison_set_detune(pcl_unison_t *u, pcl_oct_t octaves)
{
    if (u == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!unison_detune_valid(octaves))
        return PCL_ERR_OUT_OF_RANGE;
    u->detune = octaves;
    unison_apply_frequencies(u);
    return PCL_OK;
}

pcl_status_t pcl_unison_set_voices(pcl_unison_t *u, size_t voices)
{
    if (u == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!unison_voices_valid(voices))
        return PCL_ERR_OUT_OF_RANGE;
    u->voices = (uint8_t)voices;
    u->gain_mono = unison_mix_tab[voices - 1u];
    unison_apply_frequencies(u);
    unison_compute_targets(u);
    unison_snap_gains(u);
    return PCL_OK;
}

pcl_status_t pcl_unison_set_stereo_width(pcl_unison_t *u, pcl_coef_t width)
{
    if (u == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!unison_width_valid(width))
        return PCL_ERR_OUT_OF_RANGE;
    u->width = width;
    unison_compute_targets(u);
    unison_start_ramp(u);
    return PCL_OK;
}

/* Validates on voice 0 (which leaves it unchanged on error), then applies to the rest. */
#define UNISON_FORWARD(call_on_voice)                                                              \
    do                                                                                             \
    {                                                                                              \
        pcl_status_t st_;                                                                          \
        size_t       i_;                                                                           \
        if (u == NULL)                                                                             \
            return PCL_ERR_NULL_POINTER;                                                           \
        st_ = call_on_voice(&u->voice[0]);                                                         \
        if (st_ != PCL_OK)                                                                         \
            return st_;                                                                            \
        for (i_ = 1; i_ < (size_t)PCL_UNISON_MAX_VOICES; i_++)                                     \
            (void)call_on_voice(&u->voice[i_]);                                                    \
        return PCL_OK;                                                                             \
    } while (0)

pcl_status_t pcl_unison_set_waveform(pcl_unison_t *u, pcl_osc_waveform_t waveform)
{
#define UNISON_CALL_(v) pcl_osc_set_waveform((v), waveform)
    UNISON_FORWARD(UNISON_CALL_);
#undef UNISON_CALL_
}

pcl_status_t pcl_unison_set_pitch_depth(pcl_unison_t *u, pcl_oct_t octaves)
{
#define UNISON_CALL_(v) pcl_osc_set_pitch_depth((v), octaves)
    UNISON_FORWARD(UNISON_CALL_);
#undef UNISON_CALL_
}

pcl_status_t pcl_unison_set_pulse_width(pcl_unison_t *u, pcl_coef_t width)
{
#define UNISON_CALL_(v) pcl_osc_set_pulse_width((v), width)
    UNISON_FORWARD(UNISON_CALL_);
#undef UNISON_CALL_
}

pcl_status_t pcl_unison_set_pwm_depth(pcl_unison_t *u, pcl_coef_t depth)
{
#define UNISON_CALL_(v) pcl_osc_set_pwm_depth((v), depth)
    UNISON_FORWARD(UNISON_CALL_);
#undef UNISON_CALL_
}

void pcl_unison_process(pcl_unison_t *u, pcl_sample_t *out_l, pcl_sample_t *out_r,
    const pcl_sample_t *pitch_mod, const pcl_sample_t *pwm_mod, const pcl_sample_t *sync_in,
    size_t frames)
{
    const size_t n_voices = u->voices;
    size_t       pos = 0;

    while (pos < frames)
    {
        const size_t n
            = frames - pos < (size_t)PCL_UNISON_CHUNK ? frames - pos : (size_t)PCL_UNISON_CHUNK;
        pcl_sample_t buf[PCL_UNISON_CHUNK];
        unison_acc_t acc_l[PCL_UNISON_CHUNK];
        unison_acc_t acc_r[PCL_UNISON_CHUNK];
        uint32_t     ramp_after = u->ramp_left;
        size_t       v, k;

        for (k = 0; k < n; k++)
        {
            acc_l[k] = 0;
            acc_r[k] = 0;
        }

        for (v = 0; v < n_voices; v++)
        {
            pcl_osc_process(&u->voice[v], buf, pitch_mod != NULL ? pitch_mod + pos : NULL,
                pwm_mod != NULL ? pwm_mod + pos : NULL, sync_in != NULL ? sync_in + pos : NULL,
                NULL, n);

            if (out_r == NULL)
            {
                const pcl_coef_t g = u->gain_mono;
                for (k = 0; k < n; k++)
                    acc_l[k] += unison_scale(buf[k], g);
            }
            else if (u->ramp_left == 0u)
            {
                const pcl_coef_t gl = u->gain_l[v];
                const pcl_coef_t gr = u->gain_r[v];
                for (k = 0; k < n; k++)
                {
                    acc_l[k] += unison_scale(buf[k], gl);
                    acc_r[k] += unison_scale(buf[k], gr);
                }
            }
            else
            {
                /* Width ramp: each sample steps the gains, the last one lands on the target. */
                pcl_coef_t gl = u->gain_l[v];
                pcl_coef_t gr = u->gain_r[v];
                uint32_t   left = u->ramp_left;

                for (k = 0; k < n; k++)
                {
                    if (left > 0u)
                    {
                        left--;
                        if (left > 0u)
                        {
                            gl += u->step_l[v];
                            gr += u->step_r[v];
                        }
                        else
                        {
                            gl = u->target_l[v];
                            gr = u->target_r[v];
                        }
                    }
                    acc_l[k] += unison_scale(buf[k], gl);
                    acc_r[k] += unison_scale(buf[k], gr);
                }
                u->gain_l[v] = gl;
                u->gain_r[v] = gr;
                ramp_after = left;
            }
        }

        if (out_r == NULL)
        {
            unison_skip_ramp(u, (uint32_t)n);
            for (k = 0; k < n; k++)
                out_l[pos + k] = unison_saturate(acc_l[k]);
        }
        else
        {
            if (ramp_after == 0u && u->ramp_left != 0u)
                unison_snap_gains(u);
            u->ramp_left = ramp_after;
            for (k = 0; k < n; k++)
            {
                out_l[pos + k] = unison_saturate(acc_l[k]);
                out_r[pos + k] = unison_saturate(acc_r[k]);
            }
        }
        pos += n;
    }
}
