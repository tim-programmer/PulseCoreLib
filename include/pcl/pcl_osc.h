/*
 * pcl_osc.h - band-limited oscillator: sine, triangle, saw, square/pulse.
 *
 * Saw and pulse are corrected with 2-point PolyBLEP, the triangle with 2-point PolyBLAMP,
 * the sine comes from a quarter-wave table. The phase is a uint32_t accumulator (2^32 is
 * one cycle). Design note: docs/modules/oscillator.md.
 *
 * Rate: phase accumulation, waveform generation, band-limiting and sync are audio rate.
 * Pitch and PWM modulation are control rate: they are sampled once every PCL_CONTROL_DIV
 * samples (counted from the start of the stream, so results do not depend on how the
 * stream is split into blocks) and interpolated linearly in between.
 *
 * Latency: the output is delayed by exactly one sample. That lets a hard-sync event
 * correct the sample before it as well as the sample after it (a true 2-point correction)
 * without looking ahead into the next block. The first output sample after init/reset
 * is 0.
 *
 * Hard sync: a master's `sync_out` is a buffer of fractions of a sample: 0 = no wrap, and
 * a non-zero value d in (0, 1) = "the master's phase wrapped d samples before this
 * sample". Pass it as the slave's `sync_in`. Wraps that land exactly on a sample instant
 * are reported as 2^-15, the smallest non-zero value, so 0 stays unambiguous.
 */
#ifndef PCL_OSC_H
#define PCL_OSC_H

#include "pcl/pcl_smooth.h"
#include "pcl/pcl_types.h"

PCL_BEGIN_DECLS

typedef enum
{
    PCL_OSC_SINE = 0,
    PCL_OSC_TRIANGLE = 1,
    PCL_OSC_SAW = 2,
    PCL_OSC_PULSE = 3,
    PCL_OSC_WAVEFORM_COUNT = 4
} pcl_osc_waveform_t;

typedef struct
{
    uint32_t           sample_rate_hz;  /* PCL_SAMPLE_RATE_MIN_HZ .. PCL_SAMPLE_RATE_MAX_HZ       */
    pcl_osc_waveform_t waveform;        /* initial waveform                                       */
    pcl_hz_t           frequency;       /* Hz, 0 .. 0.45 * sample_rate_hz; build with PCL_HZ(x)   */
    pcl_oct_t          pitch_depth_oct; /* octaves per unit of pitch_mod, 0 .. <8; PCL_OCT(x)     */
    pcl_coef_t         pulse_width;     /* fraction of a cycle, 0.01 .. 0.99; PCL_COEF(x)         */
    pcl_coef_t         pwm_depth; /* width swing per unit of pwm_mod, 0 .. 0.49; PCL_COEF(x) */
} pcl_osc_config_t;

typedef struct
{
    /* private - callers must not touch fields directly */
    pcl_smooth_t freq;  /* cycles/sample, advanced once per control period */
    pcl_smooth_t width; /* pulse width, advanced once per control period   */
    uint32_t     sample_rate_hz;
    pcl_oct_t    pitch_depth;
    pcl_coef_t   pwm_depth;
    uint32_t     phase;
    uint32_t     inc;        /* running per-sample phase increment                    */
    uint32_t     inc_target; /* increment the current control period is heading to    */
    int32_t      inc_step;
    uint32_t     thr31; /* pulse threshold, Q0.31 fraction of a cycle            */
    uint32_t     thr_target;
    int32_t      thr_step;
    uint32_t     win; /* band-limiting window: max(inc at both period ends)    */
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    float recip; /* 1 / win                                               */
#else
    uint32_t recip; /* normalized reciprocal of win (see pcl_osc.c)          */
#endif
    uint32_t     recip_shift;
    pcl_sample_t k_tri;        /* (4/3) * win/2^32: triangle corner correction scale */
    pcl_sample_t k_sine;       /* (2*pi/3) * win/2^32: sine sync slope scale         */
    pcl_sample_t held;         /* the previous sample, not yet output (1-sample latency) */
    pcl_sample_t sync_pending; /* wrap fraction to report with the next sample       */
    uint8_t      ctrl_count;   /* samples until the next control tick                */
    uint8_t      waveform;
} pcl_osc_t;

/*
 * Initializes `osc`, phase 0, frequency settled (no initial glide). Returns
 * PCL_ERR_NULL_POINTER for NULL arguments and PCL_ERR_OUT_OF_RANGE for any field outside
 * its range or an unknown waveform; `osc` is left untouched on error. Not for the audio thread.
 */
pcl_status_t pcl_osc_init(pcl_osc_t *osc, const pcl_osc_config_t *cfg);

/*
 * Restores phase 0, cancels all ramps (frequency and width jump to their targets) and
 * clears the output delay line. Configuration (waveform, depths, targets) is kept.
 */
void pcl_osc_reset(pcl_osc_t *osc);

/*
 * Sets the target frequency in Hz (0 .. 0.45 * sample rate); the oscillator glides there
 * over ~PCL_SMOOTH_MS. Control-thread call. Errors leave the previous value in place.
 */
pcl_status_t pcl_osc_set_frequency(pcl_osc_t *osc, pcl_hz_t hz);

/* Switches waveform immediately; the phase continues, so the switch may click. */
pcl_status_t pcl_osc_set_waveform(pcl_osc_t *osc, pcl_osc_waveform_t waveform);

/* Octaves of pitch change per unit of pitch_mod, 0 .. <8. Not smoothed. */
pcl_status_t pcl_osc_set_pitch_depth(pcl_osc_t *osc, pcl_oct_t octaves);

/* Base pulse width as a fraction of a cycle, 0.01 .. 0.99; smoothed like the frequency. */
pcl_status_t pcl_osc_set_pulse_width(pcl_osc_t *osc, pcl_coef_t width);

/* Width swing per unit of pwm_mod, 0 .. 0.49. Not smoothed. */
pcl_status_t pcl_osc_set_pwm_depth(pcl_osc_t *osc, pcl_coef_t depth);

/*
 * Jumps the phase (2^32 = one cycle). A deliberate discontinuity: no band-limiting
 * correction is applied. Control-thread call.
 */
void pcl_osc_set_phase(pcl_osc_t *osc, uint32_t phase);

/*
 * Renders `frames` samples into `out` (any count >= 1). All of the following may be NULL:
 *   pitch_mod  normalized [-1, 1]; octaves = pitch_mod * pitch_depth_oct (control rate)
 *   pwm_mod    normalized [-1, 1]; width = pulse_width + pwm_depth * pwm_mod (control rate,
 *              pulse waveform only), clamped so both edges' correction windows stay apart
 *   sync_in    master wrap fractions (see above); resets the phase with sub-sample accuracy
 *   sync_out   receives this oscillator's own phase-return events, same encoding;
 *              NULL = don't compute. Request it consistently across calls.
 * Every buffer may alias any other (each element is read before the same index is written).
 * Does not allocate, block or check arguments.
 */
void pcl_osc_process(pcl_osc_t *osc, pcl_sample_t *out, const pcl_sample_t *pitch_mod,
    const pcl_sample_t *pwm_mod, const pcl_sample_t *sync_in, pcl_sample_t *sync_out,
    size_t frames);

PCL_END_DECLS

#endif /* PCL_OSC_H */
