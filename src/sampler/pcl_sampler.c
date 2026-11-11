/*
 * pcl_sampler.c - see include/pcl/pcl_sampler.h and docs/modules/sampler.md.
 *
 * Positions are int64_t frames in Q.24 and steps uint32_t Q8.24 in every build; the pitch scaling
 * is integer (a 12-entry semitone table and the shared 2^x table), so all builds play from the same
 * positions when unmodulated. Reads use the kernels of src/core/pcl_interp.h with the top 16 bits
 * of the fraction. Loops wrap by subtracting the loop length, so the fraction is kept and a loop's
 * length is exact. Background: D. C. Massie, "Wavetable sampling synthesis" (loops, pitch
 * shifting).
 */
#include "pcl/pcl_sampler.h"

#include "core/pcl_delayline.h"
#include "core/pcl_interp.h"
#include "core/pcl_octave.h"

#define SMP_POS_BITS 24
#define SMP_ONE ((int64_t)1 << SMP_POS_BITS)
#define SMP_INTERP_BITS 16u     /* fraction bits passed to the kernels */
#define SMP_STEP_MAX UINT32_MAX /* 256 frames per output sample */
#define SMP_SEMITONES 12
#define SMP_MS_PER_S 1000u
#define SMP_MAX_NOTE 127u
#define SMP_SEMITONE_SHIFT 30 /* the semitone table is Q2.30 */

/* 2^(i/12), i = 0 .. 11, as Q2.30. */
static const uint32_t smp_semitone_tab[SMP_SEMITONES] = {
    (uint32_t)PCL_CONST_Q30(1.0),
    (uint32_t)PCL_CONST_Q30(1.0594630943592953),
    (uint32_t)PCL_CONST_Q30(1.1224620483093730),
    (uint32_t)PCL_CONST_Q30(1.1892071150027210),
    (uint32_t)PCL_CONST_Q30(1.2599210498948732),
    (uint32_t)PCL_CONST_Q30(1.3348398541700344),
    (uint32_t)PCL_CONST_Q30(1.4142135623730951),
    (uint32_t)PCL_CONST_Q30(1.4983070768766815),
    (uint32_t)PCL_CONST_Q30(1.5874010519681994),
    (uint32_t)PCL_CONST_Q30(1.6817928305074290),
    (uint32_t)PCL_CONST_Q30(1.7817974362806785),
    (uint32_t)PCL_CONST_Q30(1.8877486253633868),
};

#if PCL_SAMPLE_TYPE == PCL_FLOAT
#define SMP_COEF_ONE 1.0f
#else
#define SMP_COEF_ONE ((pcl_coef_t)1 << PCL_Q30_SHIFT)
#endif

/* ---- Helpers ---- */

static uint32_t smp_sat_step(uint64_t v) { return v > SMP_STEP_MAX ? SMP_STEP_MAX : (uint32_t)v; }

/* v * 2^k, saturating at the step limit. */
static uint32_t smp_shift(uint64_t v, int k)
{
    if (k >= 0)
        return v > (SMP_STEP_MAX >> k) ? SMP_STEP_MAX : smp_sat_step(v << k);
    return smp_sat_step(v >> -k);
}

/* The step for `note` given the sample's base step and root note. */
static uint32_t smp_note_step(uint32_t base, uint8_t note, uint8_t root)
{
    const int diff = (int)note - (int)root;
    const int k = diff >= 0 ? diff / SMP_SEMITONES : -((SMP_SEMITONES - 1 - diff) / SMP_SEMITONES);
    const int r = diff - k * SMP_SEMITONES; /* 0 .. 11 */
    const uint64_t v
        = ((uint64_t)base * smp_semitone_tab[r] + ((uint64_t)1 << (SMP_SEMITONE_SHIFT - 1)))
        >> SMP_SEMITONE_SHIFT;

    return smp_shift(v, k);
}

/* step * 2^oct, oct in Q4.28 (-8 .. 8), with the shared 2^x table. */
static uint32_t smp_scale_step(uint32_t step, int32_t oct)
{
    const int32_t  k = oct >> PCL_Q28_SHIFT;
    const uint32_t mant = pcl_exp2m1_q16((uint32_t)oct & ((1u << PCL_Q28_SHIFT) - 1u));
    const uint64_t v = ((uint64_t)step * (65536u + mant) + 0x8000u) >> 16;

    return smp_shift(v, (int)k);
}

static int32_t smp_mod_oct(pcl_oct_t depth, pcl_sample_t mod)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    /* |mod * depth| < 8, so the Q4.28 value fits int32. */
    return (int32_t)(mod * depth * 268435456.0f);
#elif PCL_SAMPLE_TYPE == PCL_Q15
    return pcl_mul_coef_q15(depth, mod);
#else
    return pcl_mul_coef_q31(depth, mod);
#endif
}

static pcl_coef_t smp_coef_mul(pcl_coef_t a, pcl_coef_t b)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return a * b;
#else
    return (pcl_coef_t)(((int64_t)a * b + ((int64_t)1 << (PCL_Q30_SHIFT - 1))) >> PCL_Q30_SHIFT);
#endif
}

static int smp_sample_valid(const pcl_sampler_sample_t *x)
{
    if (x->data == NULL || x->frames < 1u
        || (unsigned)x->format >= (unsigned)PCL_SAMPLER_FORMAT_COUNT
        || x->sample_rate_hz < PCL_SAMPLE_RATE_MIN_HZ || x->sample_rate_hz > PCL_SAMPLE_RATE_MAX_HZ
        || x->root_note > SMP_MAX_NOTE
        || (unsigned)x->loop_mode >= (unsigned)PCL_SAMPLER_LOOP_COUNT)
        return 0;
    if (x->loop_mode == PCL_SAMPLER_ONE_SHOT)
        return 1;
    return x->loop_start < x->loop_end && x->loop_end <= x->frames
        && x->loop_end - x->loop_start >= 2u;
}

static int smp_depth_valid(pcl_oct_t d)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return d >= 0.0f && d < 8.0f;
#else
    return d >= 0;
#endif
}

static int smp_gain_valid(pcl_coef_t g)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return g >= 0.0f && g < 2.0f;
#else
    return g >= 0; /* Q2.30 holds anything below 2 */
#endif
}

static int smp_sens_valid(pcl_coef_t v) { return v >= (pcl_coef_t)0 && v <= SMP_COEF_ONE; }

/* Sample rate / output rate in Q8.24 (at most 96: 768 kHz played at 8 kHz). */
static uint32_t smp_base_step(uint32_t sample_rate_hz, uint32_t out_rate_hz)
{
    return (
        uint32_t)((((uint64_t)sample_rate_hz << SMP_POS_BITS) + out_rate_hz / 2u) / out_rate_hz);
}

static uint32_t smp_declick_len(uint32_t ms, uint32_t fs)
{
    return (uint32_t)((uint64_t)ms * fs / SMP_MS_PER_S);
}

static unsigned smp_storage(pcl_sampler_format_t f)
{
    return f == PCL_SAMPLER_Q15 ? PCL_DL_Q15 : PCL_DL_NATIVE;
}

/* ---- Playheads ---- */

static void smp_fade_out(const pcl_sampler_t *s, pcl_sampler_head_t *h)
{
    h->stopping = 1;
    if (s->declick_len == 0u)
    {
        h->active = 0;
        return;
    }
    h->fade_left = s->declick_len;
    h->fade_step = (pcl_coef_t)((0 - h->fade) / (pcl_coef_t)s->declick_len);
}

static void smp_note_on(pcl_sampler_t *s, uint8_t note, uint8_t velocity)
{
    pcl_sampler_head_t *const         h = &s->head[0];
    const pcl_sampler_sample_t *const x = s->sample;
    uint32_t                          offset = s->start_offset;

    if (h->active)
    {
        /* The sounding note becomes the tail and fades out from where it is. */
        s->head[1] = *h;
        smp_fade_out(s, &s->head[1]);
    }
    if (offset > x->frames - 1u)
        offset = x->frames - 1u;
    h->smp = x;
    h->dir = s->reverse ? -1 : 1;
    h->pos = (int64_t)(s->reverse ? x->frames - 1u - offset : offset) * SMP_ONE;
    h->looping = x->loop_mode != PCL_SAMPLER_ONE_SHOT;
    h->step_note = smp_note_step(s->base_step, note, x->root_note);
    h->step = smp_scale_step(h->step_note, s->mod_oct);
    h->step_end = h->step;
    h->step_inc = 0;
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    h->amp = 1.0f - s->velocity_sens + s->velocity_sens * (float)velocity / (float)SMP_MAX_NOTE;
#else
    h->amp = (pcl_coef_t)(SMP_COEF_ONE - s->velocity_sens
        + ((int64_t)s->velocity_sens * velocity) / (int64_t)SMP_MAX_NOTE);
#endif
    h->stopping = 0;
    h->active = 1;
    if (s->declick_len == 0u)
    {
        h->fade = SMP_COEF_ONE;
        h->fade_left = 0u;
    }
    else
    {
        h->fade = 0;
        h->fade_step = (pcl_coef_t)(SMP_COEF_ONE / (pcl_coef_t)s->declick_len);
        h->fade_left = s->declick_len;
    }
}

static void smp_note_off(pcl_sampler_t *s)
{
    pcl_sampler_head_t *const h = &s->head[0];

    if (!h->active || h->stopping)
        return;
    switch (h->smp->loop_mode)
    {
    case PCL_SAMPLER_SUSTAIN:
        h->looping = 0; /* play on through the release part of the recording */
        break;
    case PCL_SAMPLER_FORWARD:
    case PCL_SAMPLER_PING_PONG:
        smp_fade_out(s, h);
        break;
    default: /* ONE_SHOT ignores NOTE_OFF */
        break;
    }
}

/* Frame k of the head's sample, wrapping across the loop seam while the loop is in effect. */
static pcl_sample_t smp_tap(const pcl_sampler_head_t *h, int64_t k, int wrap)
{
    const pcl_sampler_sample_t *const x = h->smp;

    if (wrap)
    {
        const int64_t len = (int64_t)x->loop_end - x->loop_start;
        if (k >= (int64_t)x->loop_end)
            k -= len;
        else if (k < (int64_t)x->loop_start)
            k += len;
    }
    if (k < 0)
        k = 0;
    else if (k >= (int64_t)x->frames)
        k = (int64_t)x->frames - 1;
    return pcl_dl_load(x->data, smp_storage(x->format), (uint32_t)k);
}

PCL_FORCE_INLINE pcl_sample_t smp_read(const pcl_sampler_head_t *h, const int hermite)
{
    const pcl_sampler_sample_t *const x = h->smp;
    const int64_t                     idx = h->pos >> SMP_POS_BITS;
    const uint32_t f = (uint32_t)(h->pos >> (SMP_POS_BITS - SMP_INTERP_BITS)) & 0xFFFFu;
    const int      in_loop = h->looping && x->loop_mode != PCL_SAMPLER_PING_PONG
        && idx >= (int64_t)x->loop_start && idx < (int64_t)x->loop_end;
    const int64_t  lo = in_loop ? (int64_t)x->loop_start : 0;
    const int64_t  hi = in_loop ? (int64_t)x->loop_end : (int64_t)x->frames; /* exclusive */
    const unsigned storage = smp_storage(x->format);

    if (!hermite)
    {
        if (idx + 1 < hi)
            return pcl_interp_linear(pcl_dl_load(x->data, storage, (uint32_t)idx),
                pcl_dl_load(x->data, storage, (uint32_t)idx + 1u), f, SMP_INTERP_BITS);
        return pcl_interp_linear(
            smp_tap(h, idx, in_loop), smp_tap(h, idx + 1, in_loop), f, SMP_INTERP_BITS);
    }
    if (idx - 1 >= lo && idx + 2 < hi)
        return pcl_interp_hermite(pcl_dl_load(x->data, storage, (uint32_t)idx - 1u),
            pcl_dl_load(x->data, storage, (uint32_t)idx),
            pcl_dl_load(x->data, storage, (uint32_t)idx + 1u),
            pcl_dl_load(x->data, storage, (uint32_t)idx + 2u), f, SMP_INTERP_BITS);
    return pcl_interp_hermite(smp_tap(h, idx - 1, in_loop), smp_tap(h, idx, in_loop),
        smp_tap(h, idx + 1, in_loop), smp_tap(h, idx + 2, in_loop), f, SMP_INTERP_BITS);
}

/* Moves the head by one step and handles loops and the sample's ends. */
static void smp_advance(const pcl_sampler_t *s, pcl_sampler_head_t *h)
{
    const pcl_sampler_sample_t *const x = h->smp;
    const int64_t                     prev = h->pos;
    const int64_t                     last = ((int64_t)x->frames - 1) * SMP_ONE;

    h->pos = h->dir > 0 ? prev + (int64_t)h->step : prev - (int64_t)h->step;

    if (h->looping)
    {
        const int64_t start = (int64_t)x->loop_start * SMP_ONE;
        const int64_t end = (int64_t)x->loop_end * SMP_ONE; /* exclusive */
        const int64_t len = end - start;

        if (x->loop_mode == PCL_SAMPLER_PING_PONG)
        {
            const int64_t top = end - SMP_ONE; /* the last loop frame: the upper turning point */
            if (h->dir > 0 && prev <= top && h->pos > top)
            {
                h->pos = 2 * top - h->pos;
                h->dir = -1;
                if (h->pos < start)
                    h->pos = start;
            }
            else if (h->dir < 0 && prev >= start && h->pos < start)
            {
                h->pos = 2 * start - h->pos;
                h->dir = 1;
                if (h->pos > top)
                    h->pos = top;
            }
        }
        else if (h->dir > 0 && prev < end && h->pos >= end)
        {
            int64_t excess = h->pos - end;
            if (excess >= len)
                excess %= len;
            h->pos = start + excess;
        }
        else if (h->dir < 0 && prev >= start && h->pos < start)
        {
            int64_t deficit = start - h->pos;
            if (deficit > len)
                deficit %= len;
            h->pos = end - deficit;
        }
    }

    /* Past either end of the data: hold the edge and fade out. Inside an active forward or sustain
       loop that ends at the last frame, positions between it and loop_end are valid (they
       interpolate across the seam), so they are left alone. */
    if ((h->pos > last || h->pos < 0)
        && !(h->looping && x->loop_mode != PCL_SAMPLER_PING_PONG
            && h->pos >= (int64_t)x->loop_start * SMP_ONE
            && h->pos < (int64_t)x->loop_end * SMP_ONE))
    {
        h->pos = h->pos < 0 ? 0 : last;
        h->step = 0u;
        h->step_end = 0u;
        h->step_inc = 0;
        h->step_note = 0u;
        if (!h->stopping)
            smp_fade_out(s, h);
    }
}

/* ---- Public API ---- */

pcl_status_t pcl_sampler_init(pcl_sampler_t *s, const pcl_sampler_config_t *cfg)
{
    pcl_sampler_t tmp;

    if (s == NULL || cfg == NULL || cfg->sample == NULL)
        return PCL_ERR_NULL_POINTER;
    if (cfg->sample_rate_hz < PCL_SAMPLE_RATE_MIN_HZ || cfg->sample_rate_hz > PCL_SAMPLE_RATE_MAX_HZ
        || !smp_sample_valid(cfg->sample)
        || (unsigned)cfg->interp >= (unsigned)PCL_SAMPLER_INTERP_COUNT
        || !smp_depth_valid(cfg->pitch_depth_oct) || !smp_gain_valid(cfg->gain)
        || !smp_sens_valid(cfg->velocity_sens) || cfg->start_offset >= cfg->sample->frames
        || cfg->declick_ms > PCL_SAMPLER_MAX_DECLICK_MS)
        return PCL_ERR_OUT_OF_RANGE;

    tmp.sample_rate_hz = cfg->sample_rate_hz;
    tmp.sample = cfg->sample;
    tmp.base_step = smp_base_step(cfg->sample->sample_rate_hz, cfg->sample_rate_hz);
    tmp.interp = (uint8_t)cfg->interp;
    tmp.pitch_depth = cfg->pitch_depth_oct;
    tmp.velocity_sens = cfg->velocity_sens;
    tmp.start_offset = cfg->start_offset;
    tmp.reverse = cfg->reverse != 0;
    tmp.declick_len = smp_declick_len(cfg->declick_ms, cfg->sample_rate_hz);
    pcl_smooth_init(&tmp.gain, cfg->gain, pcl_smooth_ramp_samples(cfg->sample_rate_hz));
    pcl_sampler_reset(&tmp);
    *s = tmp;
    return PCL_OK;
}

void pcl_sampler_reset(pcl_sampler_t *s)
{
    unsigned i;

    for (i = 0; i < 2u; i++)
    {
        s->head[i].active = 0;
        s->head[i].stopping = 0;
        s->head[i].smp = s->sample;
    }
    pcl_smooth_reset(&s->gain);
    s->mod_oct = 0;
    s->pos = 0u;
}

pcl_status_t pcl_sampler_set_sample(pcl_sampler_t *s, const pcl_sampler_sample_t *sample)
{
    if (s == NULL || sample == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!smp_sample_valid(sample))
        return PCL_ERR_OUT_OF_RANGE;
    s->sample = sample;
    s->base_step = smp_base_step(sample->sample_rate_hz, s->sample_rate_hz);
    return PCL_OK;
}

pcl_status_t pcl_sampler_set_interp(pcl_sampler_t *s, pcl_sampler_interp_t interp)
{
    if (s == NULL)
        return PCL_ERR_NULL_POINTER;
    if ((unsigned)interp >= (unsigned)PCL_SAMPLER_INTERP_COUNT)
        return PCL_ERR_OUT_OF_RANGE;
    s->interp = (uint8_t)interp;
    return PCL_OK;
}

pcl_status_t pcl_sampler_set_pitch_depth(pcl_sampler_t *s, pcl_oct_t octaves)
{
    if (s == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!smp_depth_valid(octaves))
        return PCL_ERR_OUT_OF_RANGE;
    s->pitch_depth = octaves;
    return PCL_OK;
}

pcl_status_t pcl_sampler_set_gain(pcl_sampler_t *s, pcl_coef_t gain)
{
    if (s == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!smp_gain_valid(gain))
        return PCL_ERR_OUT_OF_RANGE;
    pcl_smooth_set_target(&s->gain, gain);
    return PCL_OK;
}

pcl_status_t pcl_sampler_set_velocity_sens(pcl_sampler_t *s, pcl_coef_t sens)
{
    if (s == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!smp_sens_valid(sens))
        return PCL_ERR_OUT_OF_RANGE;
    s->velocity_sens = sens;
    return PCL_OK;
}

pcl_status_t pcl_sampler_set_start_offset(pcl_sampler_t *s, uint32_t frames)
{
    if (s == NULL)
        return PCL_ERR_NULL_POINTER;
    if (frames >= s->sample->frames)
        return PCL_ERR_OUT_OF_RANGE;
    s->start_offset = frames;
    return PCL_OK;
}

pcl_status_t pcl_sampler_set_declick(pcl_sampler_t *s, uint32_t declick_ms)
{
    if (s == NULL)
        return PCL_ERR_NULL_POINTER;
    if (declick_ms > PCL_SAMPLER_MAX_DECLICK_MS)
        return PCL_ERR_OUT_OF_RANGE;
    s->declick_len = smp_declick_len(declick_ms, s->sample_rate_hz);
    return PCL_OK;
}

void pcl_sampler_set_reverse(pcl_sampler_t *s, int reverse) { s->reverse = reverse != 0; }

void pcl_sampler_note_on(pcl_sampler_t *s, uint8_t note, uint8_t velocity)
{
    if (velocity == 0u)
        smp_note_off(s);
    else
        smp_note_on(s, note > SMP_MAX_NOTE ? (uint8_t)SMP_MAX_NOTE : note, velocity);
}

void pcl_sampler_note_off(pcl_sampler_t *s) { smp_note_off(s); }

int pcl_sampler_is_playing(const pcl_sampler_t *s)
{
    return s->head[0].active || s->head[1].active;
}

/* ---- Processing ---- */

/* At the start of every control period: the modulation and the per-sample step ramps. */
static void smp_tick(pcl_sampler_t *s, int have_mod, pcl_sample_t mod)
{
    unsigned i;

    s->mod_oct = have_mod ? smp_mod_oct(s->pitch_depth, mod) : 0;
    for (i = 0; i < 2u; i++)
    {
        pcl_sampler_head_t *const h = &s->head[i];
        if (!h->active)
            continue;
        h->step_end = smp_scale_step(h->step_note, s->mod_oct);
        h->step_inc = (int32_t)(((int64_t)h->step_end - h->step) / PCL_CONTROL_DIV);
    }
}

PCL_FORCE_INLINE void smp_render_impl(
    pcl_sampler_t *s, pcl_sample_t *out, size_t n, const int hermite)
{
    size_t i;

    for (i = 0; i < n; i++)
    {
        const pcl_coef_t gs = pcl_smooth_next(&s->gain);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
        float accf = 0.0f;
#else
        int64_t acc = 0;
#endif
        unsigned k;

        for (k = 0; k < 2u; k++)
        {
            pcl_sampler_head_t *const h = &s->head[k];
            pcl_coef_t                g;
            pcl_sample_t              y;

            if (!h->active)
                continue;
            h->step = (uint32_t)((int64_t)h->step + h->step_inc);
            y = smp_read(h, hermite);
            g = smp_coef_mul(smp_coef_mul(gs, h->amp), h->fade);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
            accf += y * g;
#elif PCL_SAMPLE_TYPE == PCL_Q15
            acc += pcl_mul_q15_q30(y, g);
#else
            acc += pcl_mul_q31_q30(y, g);
#endif
            if (h->fade_left != 0u)
            {
                h->fade_left--;
                if (h->fade_left != 0u)
                    h->fade = (pcl_coef_t)(h->fade + h->fade_step);
                else if (h->stopping)
                {
                    h->fade = 0;
                    h->active = 0;
                    continue;
                }
                else
                    h->fade = SMP_COEF_ONE;
            }
            smp_advance(s, h);
            if (h->stopping && h->fade_left == 0u)
                h->active = 0; /* a zero-length declick ended it in smp_advance */
        }
#if PCL_SAMPLE_TYPE == PCL_FLOAT
        out[i] = accf;
#else
        out[i] = pcl_interp_sat(acc);
#endif
    }
}

static void smp_render_linear(pcl_sampler_t *s, pcl_sample_t *out, size_t n)
{
    smp_render_impl(s, out, n, 0);
}

static void smp_render_hermite(pcl_sampler_t *s, pcl_sample_t *out, size_t n)
{
    smp_render_impl(s, out, n, 1);
}

void pcl_sampler_process(pcl_sampler_t *s, pcl_sample_t *out, const pcl_sample_t *pitch_mod,
    const pcl_event_t *events, size_t num_events, size_t frames)
{
    size_t i = 0, e = 0;

    while (i < frames)
    {
        size_t end;

        if (s->pos == 0u)
            smp_tick(s, pitch_mod != NULL, pitch_mod != NULL ? pitch_mod[i] : (pcl_sample_t)0);
        while (e < num_events && events[e].offset <= i)
        {
            if (events[e].type == PCL_EVT_NOTE_ON)
                pcl_sampler_note_on(s, events[e].note, events[e].velocity);
            else if (events[e].type == PCL_EVT_NOTE_OFF)
                smp_note_off(s);
            e++;
        }
        end = i + ((size_t)PCL_CONTROL_DIV - s->pos);
        if (end > frames)
            end = frames;
        if (e < num_events && events[e].offset < end)
            end = events[e].offset;

        if (s->interp == PCL_SAMPLER_HERMITE)
            smp_render_hermite(s, out + i, end - i);
        else
            smp_render_linear(s, out + i, end - i);

        s->pos = (uint8_t)(s->pos + (end - i));
        if (s->pos >= PCL_CONTROL_DIV)
        {
            unsigned k;
            s->pos = 0u;
            for (k = 0; k < 2u; k++)
                s->head[k].step = s->head[k].step_end; /* land exactly on the target */
        }
        i = end;
    }
}
