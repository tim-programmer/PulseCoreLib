/*
 * pcl_biquad.h - second-order filter, the RBJ "Audio EQ Cookbook" set: low-pass, high-pass,
 * band-pass, notch, all-pass, peaking EQ, low shelf and high shelf.
 *
 * Design note: docs/modules/biquad.md.
 *
 *     y[n] = b0 x[n] + b1 x[n-1] + b2 x[n-2] - a1 y[n-1] - a2 y[n-2]        (Direct Form I)
 *
 * The coefficients are the bilinear transform of each type's analogue prototype, evaluated with
 * K = tan(pi * f / fs); they equal the cookbook's. Use this module for filters whose parameters
 * hold still or move smoothly (EQ, tone, crossovers). For fast cutoff sweeps use the state-variable
 * filter (pcl_svf.h): a biquad with interpolated coefficients is not robust under violent
 * modulation.
 *
 * Rate: the filter runs at AUDIO rate; its coefficients are updated at CONTROL rate, once every
 * PCL_CONTROL_DIV samples (counted from the start of the stream, so the output does not depend on
 * how it is split into blocks) and ramped linearly across the following period. Consequently the
 * cutoff_mod input is sampled once per control period, and a setter takes effect at the next
 * control point and is smoothed over ~PCL_SMOOTH_MS (cutoff and Q in octaves, gain in dB).
 *
 * Precision: the history is 32 bit in every sample type (Q4.28 in fixed point). Fixed point carries
 * the rounding residual of each output into the next sample (error feedback), which keeps low
 * cutoffs accurate (111 dB SNR at 20 Hz, 48 kHz). The float build does not and is weakest below
 * about 100 Hz; use a fixed-point build or the SVF there.
 *
 * Levels: peaking and shelving gains are limited to +-18 dB. The output saturates at full scale in
 * Q15/Q31 (the history keeps up to 8x full scale); the float history is clamped to +-8 so that a
 * filter driven unstable cannot reach NaN/Inf. Inputs are not sanitized: feed finite samples.
 */
#ifndef PCL_BIQUAD_H
#define PCL_BIQUAD_H

#include "pcl/pcl_smooth.h"
#include "pcl/pcl_types.h"

PCL_BEGIN_DECLS

/* Lowest cutoff accepted, in Hz; full accuracy from 20 Hz. The highest is 0.45 * sample_rate_hz. */
#define PCL_BIQUAD_CUTOFF_MIN_HZ 5

/* Limits of the quality factor and of the peaking / shelving gain. */
#define PCL_BIQUAD_Q_MIN 0.2
#define PCL_BIQUAD_Q_MAX 100.0
#define PCL_BIQUAD_GAIN_MAX_DB 18.0

typedef enum
{
    PCL_BIQUAD_LP,       /* low-pass, resonance Q                                          */
    PCL_BIQUAD_HP,       /* high-pass, resonance Q                                         */
    PCL_BIQUAD_BP,       /* band-pass, 0 dB gain at the centre (constant peak gain)        */
    PCL_BIQUAD_NOTCH,    /* notch                                                          */
    PCL_BIQUAD_ALLPASS,  /* all-pass                                                       */
    PCL_BIQUAD_PEAK,     /* peaking EQ: gain_db at the centre, bandwidth by Q              */
    PCL_BIQUAD_LOWSHELF, /* gain_db below the corner, steepness by Q (0.707 = no overshoot) */
    PCL_BIQUAD_HIGHSHELF /* gain_db above the corner                                       */
} pcl_biquad_type_t;

typedef struct
{
    uint32_t sample_rate_hz; /* PCL_SAMPLE_RATE_MIN_HZ .. PCL_SAMPLE_RATE_MAX_HZ            */
    pcl_biquad_type_t type;
    pcl_hz_t          cutoff_hz; /* PCL_BIQUAD_CUTOFF_MIN_HZ .. 0.45 * sample_rate_hz; PCL_HZ(x) */
    pcl_quality_t     q;         /* 0.2 .. 100; PCL_QUALITY(x)                                  */
    pcl_db_t          gain_db;   /* -18 .. +18; PCL_DB(x); PEAK and the shelves only            */
    pcl_oct_t mod_depth_oct;     /* octaves of cutoff per unit of cutoff_mod, 0 .. <8; PCL_OCT  */
} pcl_biquad_config_t;

/* Private: one coefficient. Fixed point: b0 b1 b2 are Q5.27, a1 a2 are Q2.30. */
#if PCL_SAMPLE_TYPE == PCL_FLOAT
typedef float pcl_biquad_c_t;
#else
typedef int32_t pcl_biquad_c_t;
#endif

typedef struct
{
    /* private - callers must not touch fields directly */
    pcl_smooth_t cutoff; /* base cutoff as u = octaves above 1 kHz / 8, in -1 .. 1               */
    pcl_smooth_t q;    /* log2(Q) / 8                                                            */
    pcl_smooth_t gain; /* gain_db / 18                                                           */
    pcl_biquad_c_t c[5];     /* coefficients in use: b0 b1 b2 a1 a2 */
    pcl_biquad_c_t c_end[5]; /* targets at the end of the control period                          */
    pcl_biquad_c_t c_step[5];  /* per-sample steps */
    pcl_wide_t x1, x2, y1, y2; /* history: inputs and outputs, Q4.28                           */
    int32_t    err; /* fixed point: rounding residual of the last output, 2^-55 units       */
    pcl_oct_t  mod_depth;
    uint32_t   inc_1k; /* 1 kHz as a phase increment (fixed point only)                         */
    uint32_t   sample_rate_hz;
    uint8_t    type;
    uint8_t    pos;       /* samples already output in the current control period               */
    uint8_t    ramping;   /* the coefficients move during this period                          */
    uint8_t    mod_dirty; /* the coefficients in use came from a modulated cutoff              */
    uint8_t    retarget;  /* a type change is waiting for the next control point               */
} pcl_biquad_t;

/*
 * Initializes `bq` from `cfg`, settled (no initial ramp) with the history cleared. Returns
 * PCL_ERR_NULL_POINTER for NULL arguments and PCL_ERR_OUT_OF_RANGE for any field outside the ranges
 * above (NaN included); `bq` is left untouched on error. Control-thread call.
 */
pcl_status_t pcl_biquad_init(pcl_biquad_t *bq, const pcl_biquad_config_t *cfg);

/* Clears the history and cancels any ramp; the configuration is kept. Control-thread call. */
void pcl_biquad_reset(pcl_biquad_t *bq);

/*
 * Setters: same ranges and errors as the config; on error the module is unchanged. The cutoff, Q
 * and gain glide over ~PCL_SMOOTH_MS; a new type is reached by a linear ramp over one control
 * period with the history kept. Control-thread calls.
 */
pcl_status_t pcl_biquad_set_type(pcl_biquad_t *bq, pcl_biquad_type_t type);
pcl_status_t pcl_biquad_set_cutoff(pcl_biquad_t *bq, pcl_hz_t cutoff_hz);
pcl_status_t pcl_biquad_set_q(pcl_biquad_t *bq, pcl_quality_t q);
pcl_status_t pcl_biquad_set_gain(pcl_biquad_t *bq, pcl_db_t gain_db);
pcl_status_t pcl_biquad_set_mod_depth(pcl_biquad_t *bq, pcl_oct_t octaves);

/*
 * Filters `frames` samples (any count >= 1) of `in` into `out`. `cutoff_mod` may be NULL
 * (unmodulated), else it is normalized [-1, 1] and the cutoff moves by cutoff_mod * mod_depth
 * octaves (control rate: the sample at each control point counts; keep the motion smooth, see
 * above). `in` may alias `out`. The modulated cutoff is clamped to 3.9 Hz .. 0.45 * sample_rate_hz.
 * Does not allocate or block, and does not check its arguments.
 */
void pcl_biquad_process(pcl_biquad_t *bq, const pcl_sample_t *in, pcl_sample_t *out,
    const pcl_sample_t *cutoff_mod, size_t frames);

PCL_END_DECLS

#endif /* PCL_BIQUAD_H */
