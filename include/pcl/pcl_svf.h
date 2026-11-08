/*
 * pcl_svf.h - state-variable filter (VCF): low-pass, band-pass and high-pass from one resonant
 * filter, with an exponential cutoff modulation input.
 *
 * Design note: docs/modules/svf.md.
 *
 *     hp = a1 * (x - c*s1 - s2)       a1 = 1 / (1 + g*(g+k))    c = g + k
 *     bp = g*hp + s1                  g  = tan(pi * fc / fs)    k = 1/Q
 *     lp = g*bp + s2                  s1 += 2*g*hp              s2 += 2*g*bp
 *
 * The trapezoidal (topology-preserving) discretisation of the analogue state-variable filter
 * (Zavalishin): the cutoff is exact, the resonance does not detune it, and the coefficients may be
 * modulated fast without the instabilities of the Chamberlin form.
 *
 * Rate: the filter runs at AUDIO rate; its coefficients are updated at CONTROL rate, once every
 * PCL_CONTROL_DIV samples (counted from the start of the stream, so the output does not depend on
 * how it is split into blocks) and ramped linearly across the following period. Consequently:
 *   - the cutoff_mod input is sampled once per control period; the filter reaches the cutoff it
 * asks for within one period (0.33 ms at 48 kHz), and the input's useful bandwidth is about fs /
 * 32;
 *   - a setter takes effect at the next control point and is smoothed over ~PCL_SMOOTH_MS, in
 *     octaves for the cutoff (so a sweep is musically even) and linearly for the damping.
 *
 * Resonance r in 0 .. 1 maps to the damping k = sqrt(2) * (1 - r)^2 (Q = 1/k): r = 0 is the flat
 * Butterworth response (Q = 0.707), r = 0.5 has Q = 2.8, r = 0.8 Q = 17.7, r = 0.9 Q = 71, and r =
 * 1 is undamped: the filter rings at the cutoff for ever (self-oscillation). At f = fc the low-pass
 * and band-pass gains are exactly Q.
 *
 * Levels: the internal states are 32 bit in every sample type (Q4.28 in fixed point, so 18 dB above
 * full scale) and are limited to +-8, so the filter never runs away. The band-pass peaks at Q
 * times the input; outputs above full scale saturate in Q15/Q31. The float build does not clip
 * its outputs: with finite input they stay finite and follow the (limited) states, which in the
 * worst case tested (a self-oscillating filter under violent modulation) is 8.9. With `saturate`
 * on, the band-pass state also passes through tanh each sample (an analogue-style soft limiter that
 * keeps even a self-oscillating filter near full scale). Inputs are not sanitized: feed finite
 * samples.
 */
#ifndef PCL_SVF_H
#define PCL_SVF_H

#include "pcl/pcl_smooth.h"
#include "pcl/pcl_types.h"

PCL_BEGIN_DECLS

/* Lowest cutoff accepted, in Hz. The highest is 0.45 * sample_rate_hz. */
#define PCL_SVF_CUTOFF_MIN_HZ 5

typedef enum
{
    PCL_SVF_LP,      /* low-pass                                                     */
    PCL_SVF_BP,      /* band-pass, gain Q at the cutoff                              */
    PCL_SVF_BP_NORM, /* band-pass, gain 1 (0 dB) at the cutoff: k * bp               */
    PCL_SVF_HP,      /* high-pass                                                    */
    PCL_SVF_NOTCH,   /* lp + hp                                                      */
    PCL_SVF_PEAK,    /* lp - hp                                                      */
    PCL_SVF_ALLPASS  /* lp + hp - k * bp                                             */
} pcl_svf_mode_t;

typedef struct
{
    uint32_t   sample_rate_hz; /* PCL_SAMPLE_RATE_MIN_HZ .. PCL_SAMPLE_RATE_MAX_HZ              */
    pcl_hz_t   cutoff_hz;      /* PCL_SVF_CUTOFF_MIN_HZ .. 0.45 * sample_rate_hz; PCL_HZ(x)      */
    pcl_coef_t resonance;      /* 0 .. 1; PCL_COEF(x)                                           */
    pcl_oct_t  mod_depth_oct;  /* octaves of cutoff per unit of cutoff_mod, 0 .. <8; PCL_OCT(x) */
    int        saturate;       /* non-zero: tanh soft limiter on the band-pass state            */
} pcl_svf_config_t;

/* Private: d = 1 + g*c, which reaches about 50: Q8.24 in fixed point. */
typedef pcl_db_t pcl_svf_d_t;

typedef struct
{
    /* private - callers must not touch fields directly */
    pcl_smooth_t cutoff;  /* base cutoff as u = octaves above 1 kHz / 8, in -1 .. 1          */
    pcl_smooth_t damping; /* k                                                                */
    pcl_wide_t   s1, s2;  /* integrator states                                                */
    pcl_wide_t   g, c;    /* coefficients in use: g and c = g + k                             */
    pcl_coef_t   a1;      /* 1 / (1 + g*c), kept by Newton's method while g and c ramp        */
    pcl_wide_t   g_end, c_end, dg, dc; /* targets at the end of the period and per-sample steps */
    pcl_coef_t   a1_end;               /* the exact a1 at the end of the period                 */
    pcl_coef_t   ac, ac_end;           /* a1 * c (about 1.5 at most): the s1 term of hp         */
    pcl_svf_d_t  d, d_step, d_step2;   /* 1 + g*c and its forward differences while ramping     */
    pcl_oct_t    mod_depth;
    uint32_t     inc_1k; /* 1 kHz as a phase increment (fixed point only)           */
    uint32_t     sample_rate_hz;
    uint8_t      pos; /* samples already output in the current control period        */
    uint8_t      saturate;
    uint8_t      ramping;   /* the coefficients move during this period                    */
    uint8_t      mod_dirty; /* the coefficients in use came from a modulated cutoff        */
} pcl_svf_t;

/*
 * Initializes `svf` from `cfg`, settled (no initial ramp) with the states at 0. Returns
 * PCL_ERR_NULL_POINTER for NULL arguments and PCL_ERR_OUT_OF_RANGE for any field outside the ranges
 * above (NaN included); `svf` is left untouched on error. Control-thread call.
 */
pcl_status_t pcl_svf_init(pcl_svf_t *svf, const pcl_svf_config_t *cfg);

/* Clears the integrator states and cancels any ramp; the configuration is kept. Control-thread
 * call. */
void pcl_svf_reset(pcl_svf_t *svf);

/*
 * Setters: same ranges and errors as the config; on error the module is unchanged. The cutoff and
 * resonance glide over ~PCL_SMOOTH_MS; the depth and `saturate` apply at the next control point.
 * Control-thread calls.
 */
pcl_status_t pcl_svf_set_cutoff(pcl_svf_t *svf, pcl_hz_t cutoff_hz);
pcl_status_t pcl_svf_set_resonance(pcl_svf_t *svf, pcl_coef_t resonance);
pcl_status_t pcl_svf_set_mod_depth(pcl_svf_t *svf, pcl_oct_t octaves);
void         pcl_svf_set_saturate(pcl_svf_t *svf, int saturate);

/*
 * Filters `frames` samples (any count >= 1) of `in` and writes the low-pass, band-pass and
 * high-pass outputs; any output may be NULL (not computed). `cutoff_mod` may be NULL (unmodulated),
 * else it is normalized [-1, 1] and the cutoff moves by cutoff_mod * mod_depth octaves (control
 * rate: the sample at each control point counts). `in` may alias any output; the outputs must not
 * alias each other. The modulated cutoff is clamped to 3.9 Hz .. 0.45 * sample_rate_hz. Does not
 * allocate or block, and does not check its arguments.
 */
void pcl_svf_process(pcl_svf_t *svf, const pcl_sample_t *in, pcl_sample_t *out_lp,
    pcl_sample_t *out_bp, pcl_sample_t *out_hp, const pcl_sample_t *cutoff_mod, size_t frames);

/*
 * The same filter with a single selectable response written to `out` (see pcl_svf_mode_t). The sums
 * for the derived modes saturate in fixed point. `in` may alias `out`.
 */
void pcl_svf_process_mode(pcl_svf_t *svf, const pcl_sample_t *in, pcl_sample_t *out,
    pcl_svf_mode_t mode, const pcl_sample_t *cutoff_mod, size_t frames);

PCL_END_DECLS

#endif /* PCL_SVF_H */
