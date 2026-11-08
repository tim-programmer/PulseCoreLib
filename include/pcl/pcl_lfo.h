/*
 * pcl_lfo.h - low-frequency oscillator: a normalised control signal for pitch, PWM, gain, ...
 *
 * Design note: docs/modules/lfo.md.
 *
 * Rate: CONTROL RATE. The shape is evaluated once every PCL_CONTROL_DIV samples (counted from
 * the start of the stream, so the output does not depend on how it is split into blocks) and
 * linearly interpolated to audio rate in between. Consequences: a square edge, a saw wrap or a
 * random step that falls inside a control period becomes a PCL_CONTROL_DIV-sample linear ramp
 * (no clicks, but edges are not sample-exact), and the rate is limited to 16 control points per
 * cycle, i.e. sample_rate / (16 * PCL_CONTROL_DIV) (187.5 Hz at 48 kHz, where the interpolated
 * sine has -44.5 dB distortion; far less at normal LFO rates). There is no output latency and
 * no one-period lag on rate modulation.
 *
 * Shapes are aligned with the sine: sine and triangle cross zero rising at phase 0 (triangle
 * peaks at 0.25); saw up crosses zero rising, peaks just before 0.5, jumps to -1 and rises back
 * to 0 at 1; saw down mirrors it; the square is high from 0 to pulse_width; the random shapes take
 * a new uniform value in [-1, 1) at each phase wrap (random values advance on every wrap whatever
 * the current shape). The phase is a uint32_t accumulator (2^32 = one cycle).
 */
#ifndef PCL_LFO_H
#define PCL_LFO_H

#include "pcl/pcl_noise.h"
#include "pcl/pcl_smooth.h"
#include "pcl/pcl_types.h"

PCL_BEGIN_DECLS

typedef enum
{
    PCL_LFO_SINE = 0,
    PCL_LFO_TRIANGLE = 1,
    PCL_LFO_SAW_UP = 2,
    PCL_LFO_SAW_DOWN = 3,
    PCL_LFO_SQUARE = 4,
    PCL_LFO_RANDOM_STEP = 5,   /* sample-and-hold: a new value each cycle      */
    PCL_LFO_RANDOM_SMOOTH = 6, /* linear interpolation between successive values */
    PCL_LFO_SHAPE_COUNT = 7
} pcl_lfo_shape_t;

typedef struct
{
    uint32_t sample_rate_hz;   /* PCL_SAMPLE_RATE_MIN_HZ .. PCL_SAMPLE_RATE_MAX_HZ            */
    pcl_lfo_shape_t shape;     /* initial shape                                               */
    pcl_hz_t        frequency; /* Hz, 0 .. sample_rate / (16 * PCL_CONTROL_DIV); PCL_HZ(x)    */
    pcl_oct_t  rate_depth_oct; /* octaves per unit of rate_mod, 0 .. <8; PCL_OCT(x)           */
    pcl_coef_t pulse_width;    /* fraction of a cycle, 0.01 .. 0.99; PCL_COEF(x); square only */
    int        unipolar;       /* 0: output in [-1, 1]; non-zero: output in [0, 1]            */
    uint32_t   start_phase;    /* phase at init/reset/retrigger; 2^32 = one cycle             */
    uint32_t   seed;           /* random shapes; 0 means PCL_NOISE_DEFAULT_SEED               */
} pcl_lfo_config_t;

typedef struct
{
    /* private - callers must not touch fields directly */
    pcl_smooth_t freq;  /* frequency / maximum frequency, advanced once per control period */
    pcl_smooth_t width; /* pulse width, advanced once per control period                   */
    uint32_t     sample_rate_hz;
    pcl_oct_t    rate_depth;
    uint32_t     phase; /* phase at the next control point                       */
    uint32_t     start_phase;
    uint32_t     seed;
    uint32_t     rng;
    pcl_coef_t   v_next;        /* bipolar shape value at `phase`                    */
    pcl_coef_t   cur;           /* output so far in the current period               */
    pcl_coef_t   step;          /* per-sample increment in the current period        */
    pcl_coef_t   r_prev, r_cur; /* random values, Q2.30 / float in [-1, 1)           */
    uint8_t      pos;           /* samples already output in the current period      */
    uint8_t      shape;
    uint8_t      unipolar;
    uint8_t      jump; /* phase was set; re-evaluate v_next at the next control point */
} pcl_lfo_t;

/*
 * Initializes `lfo` from `cfg`, settled (no initial glide). Returns PCL_ERR_NULL_POINTER for NULL
 * arguments and PCL_ERR_OUT_OF_RANGE for any field outside the ranges above (NaN included); `lfo`
 * is left untouched on error. Control-thread call.
 */
pcl_status_t pcl_lfo_init(pcl_lfo_t *lfo, const pcl_lfo_config_t *cfg);

/*
 * Restarts from the configured start phase and seed, cancelling any glide; the configuration
 * (shape, target frequency, depth, width, polarity) is kept. Control-thread call.
 */
void pcl_lfo_reset(pcl_lfo_t *lfo);

/*
 * Switches shape. Phase and random state carry on; the output ramps from the old shape's
 * value to the new one over the current control period. PCL_ERR_OUT_OF_RANGE for an unknown
 * shape, with the module unchanged.
 */
pcl_status_t pcl_lfo_set_shape(pcl_lfo_t *lfo, pcl_lfo_shape_t shape);

/* Sets the target frequency; it glides there over ~PCL_SMOOTH_MS. Range and errors as in the
 * config. */
pcl_status_t pcl_lfo_set_frequency(pcl_lfo_t *lfo, pcl_hz_t frequency);

/* Sets the octaves per unit of rate_mod (not smoothed). Range and errors as in the config. */
pcl_status_t pcl_lfo_set_rate_depth(pcl_lfo_t *lfo, pcl_oct_t octaves);

/* Sets the square's pulse width (smoothed). Range and errors as in the config. */
pcl_status_t pcl_lfo_set_pulse_width(pcl_lfo_t *lfo, pcl_coef_t width);

/* Selects bipolar ([-1, 1]) or unipolar ([0, 1]) output; takes effect at the next control point. */
void pcl_lfo_set_unipolar(pcl_lfo_t *lfo, int unipolar);

/*
 * Sets the phase, effective at the next control point (a deliberate jump: the output steps over
 * one control period). Control-thread call.
 */
void pcl_lfo_set_phase(pcl_lfo_t *lfo, uint32_t phase);

/* Same as pcl_lfo_set_phase(lfo, start_phase from the config): restart the cycle. */
void pcl_lfo_retrigger(pcl_lfo_t *lfo);

/* Reseeds the random shapes (0 means PCL_NOISE_DEFAULT_SEED) and resets, as pcl_lfo_reset. */
void pcl_lfo_set_seed(pcl_lfo_t *lfo, uint32_t seed);

/*
 * The phase at the next control point (2^32 = one cycle). With the stream at a control-period
 * boundary this is the phase of the next output sample.
 */
uint32_t pcl_lfo_phase(const pcl_lfo_t *lfo);

/*
 * Writes `frames` samples (any count >= 1) to `out` (required). `rate_mod` may be NULL for
 * unmodulated; otherwise it is normalised [-1, 1], read at each control point, and scales the
 * rate by 2^(rate_mod * rate_depth_oct), clamped to the maximum rate. `rate_mod` and `out` may
 * alias. Does not allocate, block or check arguments.
 *
 * Output stays within [-1, 1] (bipolar) or [0, 1] (unipolar) in every build; the fixed-point
 * formats stop just short of +1.
 */
void pcl_lfo_process(
    pcl_lfo_t *lfo, pcl_sample_t *out, const pcl_sample_t *rate_mod, size_t frames);

PCL_END_DECLS

#endif /* PCL_LFO_H */
