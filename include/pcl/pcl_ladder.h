/*
 * pcl_ladder.h - four-pole ladder filter (Moog-style), zero-delay feedback, multimode by pole
 * mixing. Design note: docs/modules/ladder.md.
 *
 * Four identical TPT one-pole low-passes in series with negative feedback k * y4, solved without a
 * unit delay (V. Zavalishin, "The Art of VA Filter Design"), so the cutoff is exact, resonance does
 * not detune it, and at k = 4 (resonance 1) the filter rings at exactly the cutoff. The low-pass
 * thins the bass as resonance rises (DC gain 1 / (1 + k)) unless `bass_comp` is set.
 *
 * Rate: the filter runs at audio rate; its coefficients (cutoff, resonance, drive) update at
 * control rate, once every PCL_CONTROL_DIV samples counted from the start of the stream, and ramp
 * linearly in between. Cutoff modulation is sampled at those points.
 *
 * With `saturate`, the solved feedback sum passes through tanh (Zavalishin's "cheap" nonlinear
 * form): self-oscillation above resonance 1 settles at a fixed level and loud input compresses.
 * `drive` scales the input before the feedback sum; the output is not rescaled.
 */
#ifndef PCL_LADDER_H
#define PCL_LADDER_H

#include "pcl/pcl_smooth.h"
#include "pcl/pcl_types.h"

PCL_BEGIN_DECLS

/* Lowest cutoff accepted, in Hz. The highest is 0.45 * sample_rate_hz. */
#define PCL_LADDER_CUTOFF_MIN_HZ 5

/* Highest resonance: 1 is the edge of self-oscillation (k = 4), above it the filter oscillates. */
#define PCL_LADDER_RESONANCE_MAX 1.1

typedef enum
{
    PCL_LADDER_LP24 = 0, /* y4: the classic 24 dB/octave low-pass                    */
    PCL_LADDER_LP18,     /* y3                                                        */
    PCL_LADDER_LP12,     /* y2                                                        */
    PCL_LADDER_LP6,      /* y1                                                        */
    PCL_LADDER_BP12,     /* 2 (y1 - y2): gain 1 at the cutoff without resonance       */
    PCL_LADDER_BP24,     /* 4 (y2 - 2 y3 + y4): gain 1 at the cutoff without resonance */
    PCL_LADDER_HP12,     /* u - 2 y1 + y2                                             */
    PCL_LADDER_HP24,     /* u - 4 y1 + 6 y2 - 4 y3 + y4                               */
    PCL_LADDER_MODE_COUNT
} pcl_ladder_mode_t;

typedef struct
{
    uint32_t          sample_rate_hz; /* PCL_SAMPLE_RATE_MIN_HZ .. PCL_SAMPLE_RATE_MAX_HZ         */
    pcl_hz_t          cutoff_hz;     /* PCL_LADDER_CUTOFF_MIN_HZ .. 0.45 * sample_rate_hz; PCL_HZ */
    pcl_coef_t        resonance;     /* 0 .. PCL_LADDER_RESONANCE_MAX (k = 4 r); PCL_COEF(x)      */
    pcl_oct_t         mod_depth_oct; /* octaves of cutoff per unit of cutoff_mod, 0 .. <8         */
    pcl_ladder_mode_t mode;
    int               saturate;  /* non-zero: tanh on the feedback sum                       */
    pcl_wide_t        drive;     /* input gain, 1 .. <8 (linear); PCL_WIDE(x)                 */
    int               bass_comp; /* non-zero: input scaled by 1 + k, so the low-pass keeps DC */
} pcl_ladder_config_t;

typedef struct
{
    /* private - callers must not touch fields directly */
    pcl_smooth_t cutoff;             /* base cutoff as u = octaves above 1 kHz / 8              */
    pcl_smooth_t resonance;          /* r; fixed point: Q2.30 r has the bit pattern of Q4.28 k   */
    pcl_smooth_t drive;              /* Q4.28 in fixed point                                     */
    pcl_wide_t   s[4];               /* stage states, Q4.28 (+-8)                                */
    pcl_coef_t   G, G_end, dG;       /* g / (1 + g), Q2.30                               */
    pcl_wide_t   k, k_end, dk;       /* feedback 4 r, Q4.28                              */
    pcl_wide_t   drv, drv_end, ddrv; /* drive in use, Q4.28                              */
    pcl_coef_t   a, a_end;           /* 1 / (1 + k G^4), Newton-tracked while ramping, Q2.30 */
    pcl_wide_t   ak, ak_end;         /* a * k, Q4.28                                     */
    pcl_oct_t    mod_depth;
    uint32_t     inc_1k; /* 1 kHz as a phase increment (fixed point only)               */
    uint32_t     sample_rate_hz;
    uint8_t      pos; /* samples already output in the current control period         */
    uint8_t      mode;
    uint8_t      saturate;
    uint8_t      bass_comp;
    uint8_t      ramping;   /* the coefficients move during this period                     */
    uint8_t      mod_dirty; /* the coefficients in use came from a modulated cutoff         */
} pcl_ladder_t;

/*
 * Initializes `ld` with zero states and the coefficients settled. Returns PCL_ERR_NULL_POINTER for
 * NULL arguments and PCL_ERR_OUT_OF_RANGE for any field outside its range (NaN included); `ld` is
 * left untouched on error. Not for the audio thread.
 */
pcl_status_t pcl_ladder_init(pcl_ladder_t *ld, const pcl_ladder_config_t *cfg);

/* Clears the states and cancels every ramp; the configuration is kept. Control-thread call. */
void pcl_ladder_reset(pcl_ladder_t *ld);

/*
 * Setters: validate (errors leave the filter unchanged) and take effect at the next control point.
 * Cutoff (Hz) glides in octaves, resonance (0 .. 1.1) and drive (1 .. <8) glide linearly, all over
 * ~PCL_SMOOTH_MS. Depth, mode, saturate and bass_comp switch at once (a mode switch may click).
 * Control-thread calls.
 */
pcl_status_t pcl_ladder_set_cutoff(pcl_ladder_t *ld, pcl_hz_t cutoff_hz);
pcl_status_t pcl_ladder_set_resonance(pcl_ladder_t *ld, pcl_coef_t resonance);
pcl_status_t pcl_ladder_set_mod_depth(pcl_ladder_t *ld, pcl_oct_t octaves);
pcl_status_t pcl_ladder_set_mode(pcl_ladder_t *ld, pcl_ladder_mode_t mode);
pcl_status_t pcl_ladder_set_drive(pcl_ladder_t *ld, pcl_wide_t drive);
void         pcl_ladder_set_saturate(pcl_ladder_t *ld, int saturate);
void         pcl_ladder_set_bass_comp(pcl_ladder_t *ld, int bass_comp);

/*
 * Filters `frames` samples (any count >= 1) from `in` to `out`, which may alias. `cutoff_mod` is
 * optional (NULL = unmodulated): normalized [-1, 1], times mod_depth_oct octaves, sampled at
 * control rate. The states are limited to +-8; fixed point saturates the output at full scale,
 * float does not clip it. Does not allocate, block or check arguments.
 */
void pcl_ladder_process(pcl_ladder_t *ld, const pcl_sample_t *in, pcl_sample_t *out,
    const pcl_sample_t *cutoff_mod, size_t frames);

PCL_END_DECLS

#endif /* PCL_LADDER_H */
