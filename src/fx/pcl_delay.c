/*
 * pcl_delay.c - see include/pcl/pcl_delay.h and docs/modules/delay.md.
 *
 * Per sample: read y at the fractional delay d (src/core/pcl_delayline.h), form the feedback signal
 * fb * tone(sat(y)), write x + feedback into the line, output x + mix (y - x). The line is read
 * before it is written, so the minimum delay is one sample (two with Hermite interpolation).
 *
 * The delay d is uint32_t Q20.12 samples in every build. The base time glides linearly (one step
 * per control period) toward its target; at each control point the modulation is added and d moves
 * linearly to the result over the period (tape-style: a moving delay bends the pitch). The tone
 * control is the TPT one-pole of the ladder (V. Zavalishin, "The Art of VA Filter Design").
 */
#include "pcl/pcl_delay.h"

#include <string.h>

#include "core/pcl_cutoff.h"
#include "core/pcl_delayline.h"

PCL_STATIC_ASSERT(
    (int)PCL_DELAY_STORE_NATIVE == PCL_DL_NATIVE && (int)PCL_DELAY_STORE_Q15 == PCL_DL_Q15,
    delay_storage_values);

/* Line slack beyond the longest delay: the Hermite taps on either side and the read-before-write.
 */
#define DELAY_LINE_SLACK 4u
#define DELAY_MAX_SAMPLES (PCL_DL_MAX_LENGTH - 8u)
#define DELAY_MS_PER_S 1000u
#define DELAY_TONE_MAX_PERCENT 45u

#if PCL_SAMPLE_TYPE == PCL_FLOAT

/* Anti-denormal guard (as in the filters): -480 dBFS added to the line and the tone state. */
#define DELAY_DENORMAL_GUARD_F 1.0e-24f
/* Below 1/64, tanh(v) = v within 1.3e-6: pass through (see docs/modules/ladder.md). */
#define DELAY_TANH_LINEAR_F (1.0f / 64.0f)

static int delay_ms_valid(pcl_ms_t ms, uint32_t max_ms)
{
    return ms >= 0.0f && ms <= (float)max_ms;
}

static uint32_t delay_ms_to_q12(pcl_ms_t ms, uint32_t fs)
{
    return (uint32_t)((ms * (float)fs / (float)DELAY_MS_PER_S) * (float)PCL_DL_ONE + 0.5f);
}

static int delay_feedback_valid(pcl_coef_t f)
{
    return f >= -(float)PCL_DELAY_FEEDBACK_MAX && f <= (float)PCL_DELAY_FEEDBACK_MAX;
}

static int delay_mix_valid(pcl_coef_t m) { return m >= 0.0f && m <= 1.0f; }

static int delay_tone_valid(pcl_hz_t f, uint32_t fs)
{
    return f == 0.0f
        || (f >= (float)PCL_DELAY_TONE_MIN_HZ
            && f <= (float)DELAY_TONE_MAX_PERCENT / 100.0f * (float)fs);
}

static int delay_tone_off(pcl_hz_t f) { return f == 0.0f; }

static pcl_coef_t delay_tone_g(pcl_hz_t f, uint32_t fs)
{
    const float g = pcl_tan_hz(f, fs);
    return g / (1.0f + g);
}

/* Modulation in Q20.12 samples. */
static int64_t delay_mod_q12(uint32_t depth, pcl_sample_t m) { return (int64_t)((float)depth * m); }

#else /* fixed point */

#define DELAY_TANH_LINEAR ((pcl_wide_t)1 << (PCL_Q28_SHIFT - 6))
#define DELAY_HZ_FRAC_BITS 16

static int delay_ms_valid(pcl_ms_t ms, uint32_t max_ms)
{
    return (uint64_t)ms <= ((uint64_t)max_ms << DELAY_HZ_FRAC_BITS);
}

/* Q16.16 ms -> Q20.12 samples: ms * fs / 1000 / 16, rounded. */
static uint32_t delay_ms_to_q12(pcl_ms_t ms, uint32_t fs)
{
    const uint64_t den = (uint64_t)DELAY_MS_PER_S << (DELAY_HZ_FRAC_BITS - PCL_DL_FRAC_BITS);
    return (uint32_t)(((uint64_t)ms * fs + den / 2u) / den);
}

static int delay_feedback_valid(pcl_coef_t f)
{
    return f >= PCL_COEF(-PCL_DELAY_FEEDBACK_MAX) && f <= PCL_COEF(PCL_DELAY_FEEDBACK_MAX);
}

static int delay_mix_valid(pcl_coef_t m) { return m >= 0 && m <= PCL_COEF(1.0); }

static int delay_tone_valid(pcl_hz_t f, uint32_t fs)
{
    return f == 0u
        || (f >= PCL_HZ(PCL_DELAY_TONE_MIN_HZ)
            && (uint64_t)f * 100u <= ((uint64_t)fs * DELAY_TONE_MAX_PERCENT) << DELAY_HZ_FRAC_BITS);
}

static int delay_tone_off(pcl_hz_t f) { return f == 0u; }

static pcl_coef_t delay_tone_g(pcl_hz_t f, uint32_t fs)
{
    const pcl_wide_t g = pcl_tan_hz(f, fs);                            /* Q4.28 */
    const int64_t    den = (int64_t)g + ((int64_t)1 << PCL_Q28_SHIFT); /* 1 + g */
    return (pcl_coef_t)((((int64_t)g << PCL_Q30_SHIFT) + den / 2) / den);
}

static int64_t delay_mod_q12(uint32_t depth, pcl_sample_t m)
{
#if PCL_SAMPLE_TYPE == PCL_Q15
    return ((int64_t)depth * m) >> PCL_Q15_SHIFT;
#else
    return ((int64_t)depth * m) >> PCL_Q31_SHIFT;
#endif
}

#endif

static uint32_t delay_max_samples(uint32_t max_ms, uint32_t fs)
{
    return (uint32_t)(((uint64_t)max_ms * fs + DELAY_MS_PER_S - 1u) / DELAY_MS_PER_S);
}

static uint32_t delay_min_q12(unsigned interp)
{
    return interp == PCL_DELAY_HERMITE ? 2u * PCL_DL_ONE : PCL_DL_ONE;
}

static uint32_t delay_clamp(int64_t d, uint32_t lo, uint32_t hi)
{
    return d < (int64_t)lo ? lo : (d > (int64_t)hi ? hi : (uint32_t)d);
}

static uint32_t delay_glide_ticks(uint32_t glide_ms, uint32_t fs)
{
    return (uint32_t)((uint64_t)glide_ms * fs / ((uint64_t)DELAY_MS_PER_S * PCL_CONTROL_DIV));
}

static int delay_cfg_valid(const pcl_delay_config_t *cfg)
{
    if (cfg->sample_rate_hz < PCL_SAMPLE_RATE_MIN_HZ || cfg->sample_rate_hz > PCL_SAMPLE_RATE_MAX_HZ
        || cfg->max_delay_ms < 1u || cfg->max_delay_ms > PCL_DELAY_MAX_MS
        || delay_max_samples(cfg->max_delay_ms, cfg->sample_rate_hz) > DELAY_MAX_SAMPLES)
        return 0;
    return delay_ms_valid(cfg->time_ms, cfg->max_delay_ms)
        && delay_ms_valid(cfg->mod_depth_ms, cfg->max_delay_ms)
        && cfg->glide_ms <= PCL_DELAY_MAX_GLIDE_MS && delay_feedback_valid(cfg->feedback)
        && delay_tone_valid(cfg->tone_hz, cfg->sample_rate_hz) && delay_mix_valid(cfg->mix)
        && (unsigned)cfg->interp < (unsigned)PCL_DELAY_INTERP_COUNT
        && (unsigned)cfg->storage < (unsigned)PCL_DELAY_STORE_COUNT;
}

static uint32_t delay_line_length(const pcl_delay_config_t *cfg)
{
    return delay_max_samples(cfg->max_delay_ms, cfg->sample_rate_hz) + DELAY_LINE_SLACK;
}

size_t pcl_delay_required_bytes(const pcl_delay_config_t *cfg)
{
    size_t elem;

    if (cfg == NULL || !delay_cfg_valid(cfg))
        return 0;
    elem = pcl_dl_elem_bytes((unsigned)cfg->storage);
    return (size_t)delay_line_length(cfg) * elem + (elem - 1u);
}

static void delay_set_tone_fields(pcl_delay_t *dl, pcl_hz_t tone_hz)
{
    dl->tone_on = !delay_tone_off(tone_hz);
    dl->tone_g = dl->tone_on ? delay_tone_g(tone_hz, dl->sample_rate_hz) : 0;
}

pcl_status_t pcl_delay_init(
    pcl_delay_t *dl, const pcl_delay_config_t *cfg, void *buffer, size_t bytes)
{
    pcl_delay_t tmp;
    size_t      elem, offset;
    uint32_t    ramp;

    if (dl == NULL || cfg == NULL || buffer == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!delay_cfg_valid(cfg))
        return PCL_ERR_OUT_OF_RANGE;
    if (bytes < pcl_delay_required_bytes(cfg))
        return PCL_ERR_BUFFER_TOO_SMALL;

    elem = pcl_dl_elem_bytes((unsigned)cfg->storage);
    offset = (elem - (size_t)((uintptr_t)buffer % elem)) % elem;
    tmp.line = (unsigned char *)buffer + offset;
    tmp.length = delay_line_length(cfg);
    tmp.sample_rate_hz = cfg->sample_rate_hz;
    tmp.storage = (uint8_t)cfg->storage;
    tmp.interp = (uint8_t)cfg->interp;
    tmp.saturate = cfg->saturate != 0;
    tmp.max_ms = cfg->max_delay_ms;
    tmp.d_min = delay_min_q12(tmp.interp);
    tmp.d_max = (uint32_t)(((uint64_t)cfg->max_delay_ms * cfg->sample_rate_hz * PCL_DL_ONE)
        / DELAY_MS_PER_S);
    tmp.time_target = delay_ms_to_q12(cfg->time_ms, cfg->sample_rate_hz);
    tmp.time_cur = tmp.time_target;
    tmp.time_step = 0;
    tmp.glide_ticks = delay_glide_ticks(cfg->glide_ms, cfg->sample_rate_hz);
    tmp.mod_depth = delay_ms_to_q12(cfg->mod_depth_ms, cfg->sample_rate_hz);
    ramp = pcl_smooth_ramp_samples(cfg->sample_rate_hz);
    pcl_smooth_init(&tmp.feedback, cfg->feedback, ramp);
    pcl_smooth_init(&tmp.mix, cfg->mix, ramp);
    delay_set_tone_fields(&tmp, cfg->tone_hz);
    pcl_delay_reset(&tmp);
    *dl = tmp;
    return PCL_OK;
}

void pcl_delay_reset(pcl_delay_t *dl)
{
    memset(dl->line, 0, (size_t)dl->length * pcl_dl_elem_bytes(dl->storage));
    dl->w = 0u;
    dl->time_cur = dl->time_target;
    dl->glide_left = 0u;
    dl->d = delay_clamp(dl->time_cur, dl->d_min, dl->d_max);
    dl->d_end = dl->d;
    dl->d_step = 0;
    dl->pos = 0u;
    dl->tone_s = 0;
    pcl_smooth_reset(&dl->feedback);
    pcl_smooth_reset(&dl->mix);
}

pcl_status_t pcl_delay_set_time(pcl_delay_t *dl, pcl_ms_t time_ms)
{
    if (dl == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!delay_ms_valid(time_ms, dl->max_ms))
        return PCL_ERR_OUT_OF_RANGE;
    dl->time_target = delay_ms_to_q12(time_ms, dl->sample_rate_hz);
    if (dl->time_target > dl->d_max)
        dl->time_target = dl->d_max;
    if (dl->glide_ticks == 0u)
    {
        dl->time_cur = dl->time_target;
        dl->glide_left = 0u;
    }
    else
    {
        dl->time_step = ((int64_t)dl->time_target - dl->time_cur) / (int64_t)dl->glide_ticks;
        dl->glide_left = dl->glide_ticks;
    }
    return PCL_OK;
}

pcl_status_t pcl_delay_set_glide(pcl_delay_t *dl, uint32_t glide_ms)
{
    if (dl == NULL)
        return PCL_ERR_NULL_POINTER;
    if (glide_ms > PCL_DELAY_MAX_GLIDE_MS)
        return PCL_ERR_OUT_OF_RANGE;
    dl->glide_ticks = delay_glide_ticks(glide_ms, dl->sample_rate_hz);
    return PCL_OK;
}

pcl_status_t pcl_delay_set_mod_depth(pcl_delay_t *dl, pcl_ms_t depth_ms)
{
    if (dl == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!delay_ms_valid(depth_ms, dl->max_ms))
        return PCL_ERR_OUT_OF_RANGE;
    dl->mod_depth = delay_ms_to_q12(depth_ms, dl->sample_rate_hz);
    return PCL_OK;
}

pcl_status_t pcl_delay_set_feedback(pcl_delay_t *dl, pcl_coef_t feedback)
{
    if (dl == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!delay_feedback_valid(feedback))
        return PCL_ERR_OUT_OF_RANGE;
    pcl_smooth_set_target(&dl->feedback, feedback);
    return PCL_OK;
}

pcl_status_t pcl_delay_set_tone(pcl_delay_t *dl, pcl_hz_t tone_hz)
{
    if (dl == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!delay_tone_valid(tone_hz, dl->sample_rate_hz))
        return PCL_ERR_OUT_OF_RANGE;
    delay_set_tone_fields(dl, tone_hz);
    return PCL_OK;
}

pcl_status_t pcl_delay_set_mix(pcl_delay_t *dl, pcl_coef_t mix)
{
    if (dl == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!delay_mix_valid(mix))
        return PCL_ERR_OUT_OF_RANGE;
    pcl_smooth_set_target(&dl->mix, mix);
    return PCL_OK;
}

pcl_status_t pcl_delay_set_interp(pcl_delay_t *dl, pcl_delay_interp_t interp)
{
    if (dl == NULL)
        return PCL_ERR_NULL_POINTER;
    if ((unsigned)interp >= (unsigned)PCL_DELAY_INTERP_COUNT)
        return PCL_ERR_OUT_OF_RANGE;
    dl->interp = (uint8_t)interp;
    dl->d_min = delay_min_q12(dl->interp);
    /* Hermite needs two samples: lift a shorter delay now rather than read an unwritten slot. */
    if (dl->d < dl->d_min || dl->d_end < dl->d_min)
    {
        dl->d = dl->d < dl->d_min ? dl->d_min : dl->d;
        dl->d_end = dl->d_end < dl->d_min ? dl->d_min : dl->d_end;
        dl->d_step = (int32_t)(((int64_t)dl->d_end - dl->d) / PCL_CONTROL_DIV);
    }
    return PCL_OK;
}

void pcl_delay_set_saturate(pcl_delay_t *dl, int saturate) { dl->saturate = saturate != 0; }

/* ---- Processing ---- */

/* At the start of every control period: advance the glide, add the modulation, aim d at it. */
static void delay_tick(pcl_delay_t *dl, int have_mod, pcl_sample_t mod)
{
    int64_t target;

    if (dl->glide_left != 0u)
    {
        dl->glide_left--;
        dl->time_cur = dl->glide_left != 0u ? (uint32_t)((int64_t)dl->time_cur + dl->time_step)
                                            : dl->time_target;
    }
    target = dl->time_cur;
    if (have_mod)
        target += delay_mod_q12(dl->mod_depth, mod);
    dl->d_end = delay_clamp(target, dl->d_min, dl->d_max);
    dl->d_step = (int32_t)(((int64_t)dl->d_end - dl->d) / PCL_CONTROL_DIV);
}

static void delay_chunk(pcl_delay_t *dl, const pcl_sample_t *in, pcl_sample_t *out, size_t n)
{
    void *const      line = dl->line;
    const uint32_t   length = dl->length;
    const unsigned   storage = dl->storage;
    const int        hermite = dl->interp == PCL_DELAY_HERMITE;
    const int        saturate = dl->saturate;
    const int        tone_on = dl->tone_on;
    const pcl_coef_t tg = dl->tone_g;
    const int32_t    d_step = dl->d_step;
    pcl_smooth_t     fb_s = dl->feedback;
    pcl_smooth_t     mix_s = dl->mix;
    pcl_wide_t       ts = dl->tone_s;
    uint32_t         w = dl->w;
    uint32_t         d = dl->d;
    size_t           i;

    for (i = 0; i < n; i++)
    {
        const pcl_sample_t x = in[i];
        const pcl_coef_t   fb = pcl_smooth_next(&fb_s);
        const pcl_coef_t   mix = pcl_smooth_next(&mix_s);
        pcl_sample_t       y, wv;

        d = (uint32_t)((int64_t)d + d_step);
        y = hermite ? pcl_dl_read_hermite(line, storage, length, w, d)
                    : pcl_dl_read_linear(line, storage, length, w, d);

#if PCL_SAMPLE_TYPE == PCL_FLOAT
        {
            float v = y;
            if (saturate && !(v > -DELAY_TANH_LINEAR_F && v < DELAY_TANH_LINEAR_F))
                v = pcl_tanh(v);
            if (tone_on)
            {
                const float yy = tg * v + (1.0f - tg) * ts;
                ts = 2.0f * yy - ts + DELAY_DENORMAL_GUARD_F;
                v = yy;
            }
            wv = x + fb * v + DELAY_DENORMAL_GUARD_F;
            out[i] = x + mix * (y - x);
        }
#else
        {
            pcl_wide_t v = pcl_sample_to_wide(y);
            if (saturate && (v >= DELAY_TANH_LINEAR || v <= -DELAY_TANH_LINEAR))
                v = pcl_sample_to_wide(pcl_tanh(v));
            if (tone_on)
            {
                const int32_t    th = ((int32_t)1 << PCL_Q30_SHIFT) - tg;
                const pcl_wide_t yy
                    = (pcl_wide_t)(((int64_t)tg * v + (int64_t)th * ts + PCL_CUTOFF_ROUND30)
                        >> PCL_Q30_SHIFT);
                ts = pcl_wide_sat(2 * (int64_t)yy - ts);
                v = yy;
            }
            /* x keeps its full precision: only the feedback goes through Q4.28. */
            wv = pcl_dl_saturate((int64_t)x
                + pcl_wide64_to_sample(((int64_t)fb * v + PCL_CUTOFF_ROUND30) >> PCL_Q30_SHIFT));
            out[i] = pcl_dl_saturate((int64_t)x
                + (((int64_t)mix * ((int64_t)y - x) + PCL_CUTOFF_ROUND30) >> PCL_Q30_SHIFT));
        }
#endif
        pcl_dl_store(line, storage, w, wv);
        w = w + 1u == length ? 0u : w + 1u;
    }

    dl->feedback = fb_s;
    dl->mix = mix_s;
    dl->tone_s = ts;
    dl->w = w;
    dl->d = d;
}

void pcl_delay_process(pcl_delay_t *dl, const pcl_sample_t *in, pcl_sample_t *out,
    const pcl_sample_t *time_mod, size_t frames)
{
    size_t n = 0;

    while (n < frames)
    {
        size_t chunk = (size_t)PCL_CONTROL_DIV - dl->pos;

        if (chunk > frames - n)
            chunk = frames - n;
        if (dl->pos == 0u)
            delay_tick(dl, time_mod != NULL, time_mod != NULL ? time_mod[n] : (pcl_sample_t)0);
        delay_chunk(dl, in + n, out + n, chunk);
        dl->pos = (uint8_t)(dl->pos + chunk);
        if (dl->pos >= PCL_CONTROL_DIV)
        {
            dl->pos = 0u;
            dl->d = dl->d_end; /* land exactly, whatever the rounding of the steps */
        }
        n += chunk;
    }
}
