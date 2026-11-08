/*
 * pcl_noise.h - noise sources (white, pink, brown) and random sample-and-hold.
 *
 * Design note: docs/modules/noise.md.
 *
 * Every generator is integer-only and produces the same stream in every build:
 *
 *     Q31   = the native output
 *     Q15   = Q31 >> 16            (arithmetic shift, so it never saturates)
 *     float = (float)Q31 * 2^-31
 *
 * so one seed gives the same noise in float, Q15 and Q31 builds. The PRNG is xorshift32
 * (never rand()); all state lives in the module struct.
 *
 * Rate: white, pink and brown are audio rate. The sample-and-hold has no control-rate
 * quantization: a trigger takes effect on its exact sample. Its optional slew is a
 * linear ramp that runs only while active.
 */
#ifndef PCL_NOISE_H
#define PCL_NOISE_H

#include "pcl/pcl_event.h"
#include "pcl/pcl_smooth.h"
#include "pcl/pcl_types.h"

PCL_BEGIN_DECLS

/* A seed of 0 would lock xorshift32 at 0 forever, so it is replaced by this constant. */
#define PCL_NOISE_DEFAULT_SEED 0x2545F491u

/* Longest slew the sample-and-hold accepts. */
#define PCL_SAH_SLEW_MAX_MS 10000u

/* ---- White / pink / brown ---- */

typedef enum
{
    PCL_NOISE_WHITE = 0, /* uniform, flat spectrum                              */
    PCL_NOISE_PINK = 1,  /* -3 dB/octave (Voss-McCartney, 16 rows)              */
    PCL_NOISE_BROWN = 2, /* -6 dB/octave above ~15 Hz (leaky integrator + DC blocker) */
    PCL_NOISE_COLOUR_COUNT = 3
} pcl_noise_colour_t;

#define PCL_NOISE_PINK_ROWS 16

typedef struct
{
    uint32_t           sample_rate_hz; /* PCL_SAMPLE_RATE_MIN_HZ .. PCL_SAMPLE_RATE_MAX_HZ */
    pcl_noise_colour_t colour;
    uint32_t           seed; /* any value; 0 means PCL_NOISE_DEFAULT_SEED */
} pcl_noise_config_t;

typedef struct
{
    /* private - callers must not touch fields directly */
    uint32_t sample_rate_hz;
    uint32_t seed;
    uint32_t rng;
    uint8_t  colour;
    uint8_t  brown_k; /* integrator leak: pole at 1 - 2^-k */
    union
    {
        struct
        {
            uint32_t counter;
            int32_t  sum; /* sum of rows, exactly the Q31 output */
            int32_t  rows[PCL_NOISE_PINK_ROWS];
        } pink;
        struct
        {
            int32_t  acc;      /* integrator state, Q3.29 */
            int32_t  prev_in;  /* DC blocker: previous input  */
            int32_t  prev_out; /* DC blocker: previous output */
            int32_t  gain;     /* input gain, Q0.16 relative to a 16-bit input */
            uint32_t clamps;   /* times a state clamp was hit (diagnostic)     */
        } brown;
    } u;
} pcl_noise_t;

/*
 * Initializes `n` from `cfg`. Returns PCL_ERR_NULL_POINTER for NULL arguments and
 * PCL_ERR_OUT_OF_RANGE for a sample rate or colour outside its range; `n` is left
 * untouched on error. The sample rate only matters for brown noise (it sets the leak).
 * Control-thread call.
 */
pcl_status_t pcl_noise_init(pcl_noise_t *n, const pcl_noise_config_t *cfg);

/* Restarts the sequence from the seed. Control-thread call. */
void pcl_noise_reset(pcl_noise_t *n);

/*
 * Switches colour and restarts the sequence from the seed (same as init with that colour).
 * PCL_ERR_OUT_OF_RANGE for an unknown colour, with the module left unchanged.
 */
pcl_status_t pcl_noise_set_colour(pcl_noise_t *n, pcl_noise_colour_t colour);

/* Reseeds (0 means PCL_NOISE_DEFAULT_SEED) and restarts the sequence. */
void pcl_noise_set_seed(pcl_noise_t *n, uint32_t seed);

/*
 * Number of times brown noise's internal state clamp has been hit since the last
 * reset (always 0 for white and pink, which cannot saturate). Diagnostic: the clamps
 * are never reached at the nominal level in practice.
 */
uint32_t pcl_noise_clamp_count(const pcl_noise_t *n);

/*
 * Writes `frames` samples of noise to `out` (required). Audio rate; any count >= 1.
 * Does not allocate, block or check arguments.
 *
 * Nominal levels (full scale = 1): white uniform, rms 0.577; pink rms 0.143, hard-bounded;
 * brown rms 0.09. No sample is ever clipped.
 */
void pcl_noise_process(pcl_noise_t *n, pcl_sample_t *out, size_t frames);

/* ---- Random sample-and-hold ---- */

typedef struct
{
    uint32_t sample_rate_hz; /* PCL_SAMPLE_RATE_MIN_HZ .. PCL_SAMPLE_RATE_MAX_HZ                 */
    uint32_t seed;           /* any value; 0 means PCL_NOISE_DEFAULT_SEED                         */
    pcl_hz_t rate_hz;        /* internal clock, 0 .. 0.5 * sample_rate_hz; 0 = off. PCL_HZ(x)     */
    uint32_t slew_ms;        /* 0 .. PCL_SAH_SLEW_MAX_MS; 0 = step                                */
} pcl_sah_config_t;

typedef struct
{
    /* private - callers must not touch fields directly */
    pcl_smooth_t value; /* held / slewing value, uniform in [-1, 1) as pcl_coef_t */
    uint32_t     sample_rate_hz;
    uint32_t     seed;
    uint32_t     rng;
    uint32_t     phase; /* internal clock */
    uint32_t     inc;   /* clock increment per sample, 2^32 = one period */
    uint32_t     slew_ms;
    uint8_t      gate; /* Schmitt trigger state of trig_in: 1 = high */
} pcl_sah_t;

/*
 * Initializes `s`: seeds the PRNG and takes the first random value immediately, so the
 * output is not zero while waiting for the first trigger. Errors as for pcl_noise_init
 * (NaN rate included); `s` is left untouched on error. Control-thread call.
 */
pcl_status_t pcl_sah_init(pcl_sah_t *s, const pcl_sah_config_t *cfg);

/* Restarts from the seed: first value re-picked, clock phase and gate cleared, ramp cancelled. */
void pcl_sah_reset(pcl_sah_t *s);

/* Sets the internal clock rate (0 turns it off). Same range and errors as the config field. */
pcl_status_t pcl_sah_set_rate(pcl_sah_t *s, pcl_hz_t rate_hz);

/*
 * Sets the slew time. A ramp in progress is ended immediately (the output jumps to the
 * value it was heading for). Same range and errors as the config field.
 */
pcl_status_t pcl_sah_set_slew_ms(pcl_sah_t *s, uint32_t slew_ms);

/* Reseeds (0 means PCL_NOISE_DEFAULT_SEED) and restarts, as pcl_sah_reset. */
void pcl_sah_set_seed(pcl_sah_t *s, uint32_t seed);

/*
 * Produces `frames` samples (any count >= 1) in `out` (required). A new random value is
 * picked, uniform in [-1, 1), at each of:
 *   - every wrap of the internal clock (rate_hz; the first is one period after init);
 *   - each PCL_EVT_CLOCK in `events` (offset = the sample index it fires at; other event
 *     types and offsets >= frames are ignored; the array must be sorted by offset);
 *   - each rising edge of `trig_in`: a Schmitt trigger that fires when the signal rises
 *     through 0.5 and re-arms below 0.25. A gate that is already high when processing
 *     starts after init/reset fires on its first sample.
 * Any number of triggers on one sample make one pick. `trig_in`, `events` and
 * `num_events` may be NULL/0. They may alias `out`.
 *
 * Timing: with slew_ms == 0 a trigger at sample n makes out[n] the new value. With slew,
 * out[n] is still the old value and the output reaches the new one at
 * n + round(slew_ms * sample_rate / 1000); a trigger during a ramp restarts it from the
 * current output. Does not allocate or block; a trigger with slew costs one divide.
 */
void pcl_sah_process(pcl_sah_t *s, pcl_sample_t *out, const pcl_sample_t *trig_in,
    const pcl_event_t *events, size_t num_events, size_t frames);

PCL_END_DECLS

#endif /* PCL_NOISE_H */
