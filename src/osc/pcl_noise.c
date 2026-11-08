#include "pcl/pcl_noise.h"

#include "core/pcl_rng.h"

/*
 * Every generator works in integers and yields a Q31 value; noise_out() turns that into the
 * build's sample type. See the header for the relation between the three views.
 */

#define NOISE_Q31_TO_Q15_SHIFT 16

#if PCL_SAMPLE_TYPE == PCL_FLOAT
#define NOISE_Q31_TO_FLOAT (1.0f / 2147483648.0f)
#define SAH_Q30_TO_FLOAT (1.0f / 1073741824.0f)
#endif

static pcl_sample_t noise_out(int32_t q31)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return (float)q31 * NOISE_Q31_TO_FLOAT;
#elif PCL_SAMPLE_TYPE == PCL_Q15
    return (int16_t)(q31 >> NOISE_Q31_TO_Q15_SHIFT);
#else
    return q31;
#endif
}

/* ---- Pink: Voss-McCartney ---- */

/*
 * Rows are Q4.27 (a draw >> 4), so the sum of PCL_NOISE_PINK_ROWS = 16 of them is exactly
 * Q31 and can never overflow. Row 0 is replaced every sample; row k (1 .. 15) whenever the
 * sample counter has exactly k trailing zeros, i.e. every 2^k samples. R. F. Voss and
 * J. Clarke, Nature 258, 1975; the row-update scheme is J. McCartney's (see the design note).
 */
#define PINK_ROW_SHIFT 4
#define PINK_OCTAVE_ROWS (PCL_NOISE_PINK_ROWS - 1)
#define PINK_COUNTER_MASK ((1u << PINK_OCTAVE_ROWS) - 1u)

static void pink_reset(pcl_noise_t *n)
{
    int i;

    n->u.pink.counter = 0u;
    n->u.pink.sum = 0;
    for (i = 0; i < PCL_NOISE_PINK_ROWS; i++)
    {
        const int32_t v = pcl_rng_s32(pcl_rng_next(&n->rng)) >> PINK_ROW_SHIFT;
        n->u.pink.rows[i] = v;
        n->u.pink.sum += v; /* partial sums stay within the Q31 range */
    }
}

static void pink_process(pcl_noise_t *n, pcl_sample_t *out, size_t frames)
{
    uint32_t rng = n->rng;
    uint32_t counter = n->u.pink.counter;
    int32_t  sum = n->u.pink.sum;
    int32_t *rows = n->u.pink.rows;
    size_t   i;

    for (i = 0; i < frames; i++)
    {
        int32_t  v = pcl_rng_s32(pcl_rng_next(&rng)) >> PINK_ROW_SHIFT;
        uint32_t c;

        sum += v - rows[0];
        rows[0] = v;

        counter++;
        c = counter & PINK_COUNTER_MASK; /* 0 once in 2^15 samples: nothing to update */
        if (c != 0u)
        {
            unsigned row = 1u;
            while ((c & 1u) == 0u) /* at most 14 iterations, 2 on average */
            {
                c >>= 1;
                row++;
            }
            v = pcl_rng_s32(pcl_rng_next(&rng)) >> PINK_ROW_SHIFT;
            sum += v - rows[row];
            rows[row] = v;
        }
        out[i] = noise_out(sum);
    }
    n->rng = rng;
    n->u.pink.counter = counter;
    n->u.pink.sum = sum;
}

/* ---- Brown: leaky integrator of white noise, then a DC blocker ---- */

/*
 * State is Q3.29 (full scale 2^29): two bits of headroom, so no intermediate overflows an
 * int32_t and no 64-bit arithmetic is needed.
 *
 *     u    = (x16 * gain) >> 2                 x16 = top 16 bits of a draw
 *     acc += u - (acc >> k)                    pole at 1 - 2^-k, clamped to +/- full scale
 *     y    = acc - prev_in + prev_out - (prev_out >> (k + 2))     DC blocker, clamped
 *
 * The leak puts the integrator corner near BROWN_CORNER_HZ at every sample rate; the gain
 * table gives about 0.10 rms (0.09 measured after the blocker) for every k.
 */
#define BROWN_FULL_SCALE (1 << 29)
#define BROWN_INPUT_SHIFT 16
#define BROWN_GAIN_SHIFT 2
#define BROWN_Q29_TO_Q31_SHIFT 2
#define BROWN_BLOCKER_EXTRA_BITS 2
#define BROWN_K_MIN 6
#define BROWN_K_MAX 13

/* k = round(log2(sample_rate / (2 pi * 15 Hz))): the lowest sample rate that uses k, k = 7 .. 13.
 */
static const uint32_t brown_k_min_rate[BROWN_K_MAX - BROWN_K_MIN]
    = { 8530u, 17061u, 34121u, 68243u, 136485u, 272971u, 545941u };

/* 0.10 * sqrt(2) / (rms(x16) * 2^(k/2)) in Q16, rms(x16) = 1/sqrt(3), for k = 6 .. 13. */
static const uint16_t brown_gain[BROWN_K_MAX - BROWN_K_MIN + 1]
    = { 2007u, 1419u, 1003u, 709u, 502u, 355u, 251u, 177u };

static uint8_t brown_pick_k(uint32_t sample_rate_hz)
{
    uint8_t k = BROWN_K_MIN;
    while (k < BROWN_K_MAX && sample_rate_hz >= brown_k_min_rate[k - BROWN_K_MIN])
        k++;
    return k;
}

static void brown_reset(pcl_noise_t *n)
{
    n->u.brown.acc = 0;
    n->u.brown.prev_in = 0;
    n->u.brown.prev_out = 0;
    n->u.brown.gain = (int32_t)brown_gain[n->brown_k - BROWN_K_MIN];
    n->u.brown.clamps = 0u;
}

static void brown_process(pcl_noise_t *n, pcl_sample_t *out, size_t frames)
{
    uint32_t       rng = n->rng;
    int32_t        acc = n->u.brown.acc;
    int32_t        prev_in = n->u.brown.prev_in;
    int32_t        prev_out = n->u.brown.prev_out;
    uint32_t       clamps = n->u.brown.clamps;
    const int32_t  gain = n->u.brown.gain;
    const unsigned k = n->brown_k;
    const unsigned j = k + BROWN_BLOCKER_EXTRA_BITS;
    size_t         i;

    for (i = 0; i < frames; i++)
    {
        const int32_t x = pcl_rng_s32(pcl_rng_next(&rng)) >> BROWN_INPUT_SHIFT; /* int16 range */
        const int32_t u = (x * gain) >> BROWN_GAIN_SHIFT;
        int32_t       y;

        acc += u - (acc >> k);
        if (acc > BROWN_FULL_SCALE - 1)
        {
            acc = BROWN_FULL_SCALE - 1;
            clamps++;
        }
        else if (acc < -BROWN_FULL_SCALE)
        {
            acc = -BROWN_FULL_SCALE;
            clamps++;
        }

        /* (acc - prev_in) is within 2^30 and the second term within 2^29: no overflow. */
        y = (acc - prev_in) + (prev_out - (prev_out >> j));
        prev_in = acc;
        if (y > BROWN_FULL_SCALE - 1)
        {
            y = BROWN_FULL_SCALE - 1;
            clamps++;
        }
        else if (y < -BROWN_FULL_SCALE)
        {
            y = -BROWN_FULL_SCALE;
            clamps++;
        }
        prev_out = y;
        out[i] = noise_out(y * (1 << BROWN_Q29_TO_Q31_SHIFT));
    }
    n->rng = rng;
    n->u.brown.acc = acc;
    n->u.brown.prev_in = prev_in;
    n->u.brown.prev_out = prev_out;
    n->u.brown.clamps = clamps;
}

/* ---- White ---- */

static void white_process(pcl_noise_t *n, pcl_sample_t *out, size_t frames)
{
    uint32_t rng = n->rng;
    size_t   i;

    for (i = 0; i < frames; i++)
        out[i] = noise_out(pcl_rng_s32(pcl_rng_next(&rng)));
    n->rng = rng;
}

/* ---- Noise module ---- */

static void noise_restart(pcl_noise_t *n)
{
    n->rng = pcl_rng_seed(n->seed);
    if (n->colour == PCL_NOISE_PINK)
        pink_reset(n);
    else if (n->colour == PCL_NOISE_BROWN)
        brown_reset(n);
}

static int noise_colour_valid(pcl_noise_colour_t colour)
{
    return (unsigned)colour < (unsigned)PCL_NOISE_COLOUR_COUNT;
}

pcl_status_t pcl_noise_init(pcl_noise_t *n, const pcl_noise_config_t *cfg)
{
    if (n == NULL || cfg == NULL)
        return PCL_ERR_NULL_POINTER;
    if (cfg->sample_rate_hz < PCL_SAMPLE_RATE_MIN_HZ || cfg->sample_rate_hz > PCL_SAMPLE_RATE_MAX_HZ
        || !noise_colour_valid(cfg->colour))
        return PCL_ERR_OUT_OF_RANGE;

    n->sample_rate_hz = cfg->sample_rate_hz;
    n->seed = cfg->seed;
    n->colour = (uint8_t)cfg->colour;
    n->brown_k = brown_pick_k(cfg->sample_rate_hz);
    noise_restart(n);
    return PCL_OK;
}

void pcl_noise_reset(pcl_noise_t *n) { noise_restart(n); }

pcl_status_t pcl_noise_set_colour(pcl_noise_t *n, pcl_noise_colour_t colour)
{
    if (n == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!noise_colour_valid(colour))
        return PCL_ERR_OUT_OF_RANGE;
    n->colour = (uint8_t)colour;
    noise_restart(n);
    return PCL_OK;
}

void pcl_noise_set_seed(pcl_noise_t *n, uint32_t seed)
{
    n->seed = seed;
    noise_restart(n);
}

uint32_t pcl_noise_clamp_count(const pcl_noise_t *n)
{
    return n->colour == PCL_NOISE_BROWN ? n->u.brown.clamps : 0u;
}

void pcl_noise_process(pcl_noise_t *n, pcl_sample_t *out, size_t frames)
{
    switch (n->colour)
    {
    case PCL_NOISE_PINK:
        pink_process(n, out, frames);
        break;
    case PCL_NOISE_BROWN:
        brown_process(n, out, frames);
        break;
    default:
        white_process(n, out, frames);
        break;
    }
}

/* ---- Sample-and-hold ---- */

/* Schmitt trigger thresholds on trig_in: fire when rising through HIGH, re-arm below LOW. */
#if PCL_SAMPLE_TYPE == PCL_FLOAT
#define SAH_GATE_HIGH 0.5f
#define SAH_GATE_LOW 0.25f
#elif PCL_SAMPLE_TYPE == PCL_Q15
#define SAH_GATE_HIGH PCL_CONST_Q15(0.5)
#define SAH_GATE_LOW PCL_CONST_Q15(0.25)
#else
#define SAH_GATE_HIGH PCL_CONST_Q31(0.5)
#define SAH_GATE_LOW PCL_CONST_Q31(0.25)
#endif

#define SAH_PICK_SHIFT 1 /* a 32-bit draw -> Q2.30 in [-1, 1) */
#define SAH_Q30_TO_Q31_SHIFT 1
#define SAH_Q30_TO_Q15_SHIFT 15
#define SAH_MS_PER_SECOND 1000u
#define SAH_PHASE_ONE 4294967296.0 /* 2^32 */
#define SAH_HZ_Q16_FRAC_BITS 16

/* A new random value as a pcl_coef_t (Q2.30 in fixed-point builds). */
static pcl_coef_t sah_pick(uint32_t *rng)
{
    const int32_t v = pcl_rng_s32(pcl_rng_next(rng)) >> SAH_PICK_SHIFT;
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return (float)v * SAH_Q30_TO_FLOAT;
#else
    return v;
#endif
}

static pcl_sample_t sah_out(pcl_coef_t v)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return v;
#elif PCL_SAMPLE_TYPE == PCL_Q15
    return (int16_t)(v >> SAH_Q30_TO_Q15_SHIFT);
#else
    return v * (1 << SAH_Q30_TO_Q31_SHIFT);
#endif
}

static int sah_rate_valid(pcl_hz_t hz, uint32_t sample_rate_hz)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    /* Written positively so that NaN fails both comparisons. */
    return hz >= 0.0f && hz <= 0.5f * (float)sample_rate_hz;
#else
    return (uint64_t)hz * 2u <= ((uint64_t)sample_rate_hz << SAH_HZ_Q16_FRAC_BITS);
#endif
}

/* Clock increment per sample (2^32 = one period), rounded; setter only. */
static uint32_t sah_rate_to_inc(pcl_hz_t hz, uint32_t sample_rate_hz)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return (uint32_t)((double)hz * SAH_PHASE_ONE / (double)sample_rate_hz + 0.5);
#else
    return (uint32_t)((((uint64_t)hz << SAH_HZ_Q16_FRAC_BITS) + (sample_rate_hz / 2u))
        / sample_rate_hz);
#endif
}

/* Slew time in samples (>= 1 when slew_ms > 0 at the minimum sample rate); setter only. */
static uint32_t sah_ramp_samples(uint32_t slew_ms, uint32_t sample_rate_hz)
{
    const uint64_t samples
        = ((uint64_t)slew_ms * sample_rate_hz + SAH_MS_PER_SECOND / 2u) / SAH_MS_PER_SECOND;
    return samples > 0u ? (uint32_t)samples : 1u;
}

static void sah_restart(pcl_sah_t *s)
{
    s->rng = pcl_rng_seed(s->seed);
    s->phase = 0u;
    s->gate = 0u;
    pcl_smooth_init(&s->value, sah_pick(&s->rng), s->value.ramp_samples);
}

pcl_status_t pcl_sah_init(pcl_sah_t *s, const pcl_sah_config_t *cfg)
{
    if (s == NULL || cfg == NULL)
        return PCL_ERR_NULL_POINTER;
    if (cfg->sample_rate_hz < PCL_SAMPLE_RATE_MIN_HZ || cfg->sample_rate_hz > PCL_SAMPLE_RATE_MAX_HZ
        || !sah_rate_valid(cfg->rate_hz, cfg->sample_rate_hz) || cfg->slew_ms > PCL_SAH_SLEW_MAX_MS)
        return PCL_ERR_OUT_OF_RANGE;

    s->sample_rate_hz = cfg->sample_rate_hz;
    s->seed = cfg->seed;
    s->inc = sah_rate_to_inc(cfg->rate_hz, cfg->sample_rate_hz);
    s->slew_ms = cfg->slew_ms;
    s->value.ramp_samples
        = cfg->slew_ms == 0u ? 1u : sah_ramp_samples(cfg->slew_ms, cfg->sample_rate_hz);
    sah_restart(s);
    return PCL_OK;
}

void pcl_sah_reset(pcl_sah_t *s) { sah_restart(s); }

pcl_status_t pcl_sah_set_rate(pcl_sah_t *s, pcl_hz_t rate_hz)
{
    if (s == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!sah_rate_valid(rate_hz, s->sample_rate_hz))
        return PCL_ERR_OUT_OF_RANGE;
    s->inc = sah_rate_to_inc(rate_hz, s->sample_rate_hz);
    return PCL_OK;
}

pcl_status_t pcl_sah_set_slew_ms(pcl_sah_t *s, uint32_t slew_ms)
{
    if (s == NULL)
        return PCL_ERR_NULL_POINTER;
    if (slew_ms > PCL_SAH_SLEW_MAX_MS)
        return PCL_ERR_OUT_OF_RANGE;
    s->slew_ms = slew_ms;
    /* Re-initializing at the target ends any ramp in progress by jumping to where it was heading.
     */
    pcl_smooth_init(&s->value, s->value.target,
        slew_ms == 0u ? 1u : sah_ramp_samples(slew_ms, s->sample_rate_hz));
    return PCL_OK;
}

void pcl_sah_set_seed(pcl_sah_t *s, uint32_t seed)
{
    s->seed = seed;
    sah_restart(s);
}

void pcl_sah_process(pcl_sah_t *s, pcl_sample_t *out, const pcl_sample_t *trig_in,
    const pcl_event_t *events, size_t num_events, size_t frames)
{
    uint32_t       rng = s->rng;
    uint32_t       phase = s->phase;
    uint8_t        gate = s->gate;
    const uint32_t inc = s->inc;
    size_t         next_event = 0;
    size_t         i;

    for (i = 0; i < frames; i++)
    {
        const uint32_t clock = phase + inc;
        int            trigger = clock < phase; /* the clock wrapped (never when inc == 0) */

        phase = clock;

        if (trig_in != NULL)
        {
            const pcl_sample_t g = trig_in[i];
            if (g >= SAH_GATE_HIGH)
            {
                if (gate == 0u)
                {
                    gate = 1u;
                    trigger = 1;
                }
            }
            else if (g < SAH_GATE_LOW)
            {
                gate = 0u;
            }
        }

        /* Sorted events: consume everything at or before this sample (a stale or unsorted one is
         * skipped). */
        while (next_event < num_events && events[next_event].offset <= i)
        {
            if (events[next_event].offset == i && events[next_event].type == PCL_EVT_CLOCK)
                trigger = 1;
            next_event++;
        }

        if (trigger)
            pcl_smooth_set_target(&s->value, sah_pick(&rng));

        /*
         * Read the value before advancing the ramp: with a slew, out[n] is still the old value at
         * the trigger sample; without one, set_target has already jumped. (pcl_smooth_t is a
         * struct this library owns; reading its current value here avoids an API change.)
         */
        out[i] = sah_out(s->value.current);
        (void)pcl_smooth_next(&s->value);
    }
    s->rng = rng;
    s->phase = phase;
    s->gate = gate;
}
