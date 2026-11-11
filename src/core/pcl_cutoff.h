/*
 * pcl_cutoff.h - cutoff control and Q4.28 arithmetic shared by the SVF and the ladder (internal).
 *
 * Cutoff control: the base cutoff is smoothed as u = (octaves above 1 kHz) / 8 in pcl_coef_t, a
 * modulation sample times a depth adds octaves, and g = tan(pi fc / fs) comes from pcl_tan_inc
 * (fixed point) or pcl_tan_hz (float). The modulated cutoff is clamped at 0.45 fs.
 *
 * Fixed-point helpers: saturation to Q4.28 (+-8), rounded Q4.28 products, sample <-> Q4.28
 * conversion, and the Newton step a <- a (2 - d a) that keeps a = 1/d while d moves (see
 * docs/modules/svf.md, "A stability bug found by the tests").
 *
 * Moved here from pcl_svf.c unchanged; the SVF's output is bit-identical before and after.
 */
#ifndef PCL_CUTOFF_H
#define PCL_CUTOFF_H

#include "core/pcl_octave.h"
#include "pcl/pcl_math.h"
#include "pcl/pcl_types.h"

#define PCL_CUTOFF_REF_HZ 1000 /* the cutoff smoother holds octaves above this              */
#define PCL_CUTOFF_OCT_RANGE 8 /* ... divided by 8, so +-8 octaves fit pcl_coef_t           */
#define PCL_CUTOFF_MIN_HZ 5    /* lowest cutoff a filter accepts                            */
#define PCL_CUTOFF_MAX_RATIO_Q16 29491u /* 0.45 * 2^16, truncated: highest f / fs accepted   */
#define PCL_CUTOFF_MAX_RATIO_F 0.45f

/* Newton denominators d reach about 50: Q8.24 in fixed point. */
typedef pcl_db_t pcl_cutoff_d_t;

#if PCL_SAMPLE_TYPE == PCL_FLOAT

/* Octaves above 1 kHz, divided by 8. */
PCL_INLINE pcl_coef_t pcl_cutoff_u_from_hz(pcl_hz_t f)
{
    return pcl_hz_to_oct(f, (float)PCL_CUTOFF_REF_HZ) * (1.0f / (float)PCL_CUTOFF_OCT_RANGE);
}

PCL_INLINE int pcl_cutoff_valid(pcl_hz_t f, uint32_t sample_rate_hz)
{
    return f >= (float)PCL_CUTOFF_MIN_HZ && f <= PCL_CUTOFF_MAX_RATIO_F * (float)sample_rate_hz;
}

PCL_INLINE int pcl_cutoff_depth_valid(pcl_oct_t d) { return d >= 0.0f && d < 8.0f; }

PCL_INLINE uint32_t pcl_cutoff_inc_ref(uint32_t sample_rate_hz)
{
    (void)sample_rate_hz;
    return 0u;
}

/* tan(pi fc / fs) for the smoothed value `u` plus `mod_oct` octaves. */
PCL_INLINE pcl_wide_t pcl_cutoff_g(
    pcl_coef_t u, pcl_oct_t mod_oct, uint32_t inc_ref, uint32_t sample_rate_hz)
{
    const float oct = u * (float)PCL_CUTOFF_OCT_RANGE + mod_oct;

    (void)inc_ref;
    return pcl_tan_hz(pcl_hz_scale((float)PCL_CUTOFF_REF_HZ, oct), sample_rate_hz);
}

PCL_INLINE pcl_oct_t pcl_cutoff_mod_octaves(pcl_oct_t depth, pcl_sample_t mod)
{
    return mod * depth;
}

/* One Newton step toward 1/d: a * (2 - d*a). Converges quadratically from a close start. */
PCL_INLINE pcl_coef_t pcl_cutoff_newton(pcl_coef_t a, pcl_cutoff_d_t d)
{
    /* Newton from below never exceeds 1/d <= 1, so only the floor needs guarding. */
    const float r = a * (2.0f - d * a);
    return r < 0.0f ? 0.0f : r;
}

#else /* fixed point */

#define PCL_CUTOFF_ROUND28 ((int64_t)1 << (PCL_Q28_SHIFT - 1))
#define PCL_CUTOFF_ROUND30 ((int64_t)1 << (PCL_Q30_SHIFT - 1))
#define PCL_CUTOFF_MAX_INC (PCL_CUTOFF_MAX_RATIO_Q16 << 16) /* the limit as a phase increment */

/*
 * Saturates to the Q4.28 range, which is the +-8 limit. Written as "convert, then check the
 * conversion was lossless" and made opaque, so the products it feeds are single 32x32 -> 64
 * multiplies.
 */
PCL_INLINE int32_t pcl_wide_sat(int64_t v)
{
    int32_t r = (int32_t)v;

    if (v != (int64_t)r)
        r = v > 0 ? INT32_MAX : INT32_MIN;
    PCL_OPAQUE_I32(r);
    return r;
}

/* Rounded Q4.28 product, as a 64-bit value that has not been saturated. */
PCL_INLINE int64_t pcl_wide_mul(int32_t a, int32_t b)
{
    return ((int64_t)a * b + PCL_CUTOFF_ROUND28) >> PCL_Q28_SHIFT;
}

/* A Q4.28 value (any int64) as a sample, saturating at full scale. */
PCL_INLINE pcl_sample_t pcl_wide64_to_sample(int64_t v)
{
    const int64_t one = (int64_t)1 << PCL_Q28_SHIFT;
    const int32_t w = (int32_t)(v >= one ? one - 1 : (v <= -one ? -one : v));

#if PCL_SAMPLE_TYPE == PCL_Q15
    return pcl_sat_q15(
        (w + (1 << (PCL_Q28_SHIFT - PCL_Q15_SHIFT - 1))) >> (PCL_Q28_SHIFT - PCL_Q15_SHIFT));
#else
    return (pcl_sample_t)((uint32_t)w << (PCL_Q31_SHIFT - PCL_Q28_SHIFT));
#endif
}

/* A sample as Q4.28 (rounded in Q31). */
PCL_INLINE pcl_wide_t pcl_sample_to_wide_rounded(pcl_sample_t x)
{
#if PCL_SAMPLE_TYPE == PCL_Q15
    return (pcl_wide_t)x * (1 << (PCL_Q28_SHIFT - PCL_Q15_SHIFT));
#else
    return (pcl_wide_t)(((int64_t)x + (1 << (PCL_Q31_SHIFT - PCL_Q28_SHIFT - 1)))
        >> (PCL_Q31_SHIFT - PCL_Q28_SHIFT));
#endif
}

/* Octaves above 1 kHz, divided by 8, as Q2.30: Q4.28 >> 1. */
PCL_INLINE pcl_coef_t pcl_cutoff_u_from_hz(pcl_hz_t f)
{
    return pcl_hz_to_oct(f, PCL_HZ(PCL_CUTOFF_REF_HZ)) >> 1;
}

PCL_INLINE int pcl_cutoff_valid(pcl_hz_t f, uint32_t sample_rate_hz)
{
    return f >= PCL_HZ(PCL_CUTOFF_MIN_HZ)
        && (uint64_t)f <= (uint64_t)sample_rate_hz * PCL_CUTOFF_MAX_RATIO_Q16;
}

PCL_INLINE int pcl_cutoff_depth_valid(pcl_oct_t d) { return d >= 0; }

/* 1 kHz as a phase increment. */
PCL_INLINE uint32_t pcl_cutoff_inc_ref(uint32_t sample_rate_hz)
{
    return (uint32_t)(((uint64_t)PCL_CUTOFF_REF_HZ << 32) / sample_rate_hz);
}

PCL_INLINE pcl_wide_t pcl_cutoff_g(
    pcl_coef_t u, pcl_oct_t mod_oct, uint32_t inc_ref, uint32_t sample_rate_hz)
{
    const int64_t  oct = ((int64_t)u * 2) + mod_oct; /* Q4.28 */
    const uint32_t inc
        = pcl_octave_scale_inc(inc_ref, (int32_t)pcl_wide_sat(oct), PCL_CUTOFF_MAX_INC);

    (void)sample_rate_hz;
    return pcl_tan_inc(inc);
}

PCL_INLINE pcl_oct_t pcl_cutoff_mod_octaves(pcl_oct_t depth, pcl_sample_t mod)
{
#if PCL_SAMPLE_TYPE == PCL_Q15
    return pcl_mul_coef_q15(depth, mod);
#else
    return pcl_mul_coef_q31(depth, mod);
#endif
}

/* One Newton step toward 1/d (d Q8.24, a Q2.30): a * (2 - d*a), clamped to 0 .. 1. */
PCL_INLINE pcl_coef_t pcl_cutoff_newton(pcl_coef_t a, pcl_cutoff_d_t d)
{
    const int64_t one = (int64_t)1 << PCL_Q30_SHIFT;
    /* d (Q8.24) * a (Q2.30) = Q10.54; >> 24 gives d*a as Q2.30, about 1. */
    const int64_t t = ((int64_t)d * a + ((int64_t)1 << 23)) >> 24;
    /* 2 - t is about 1; kept in 0 .. 2^31 - 1 so the product is a single 32x32 -> 64 multiply */
    const int64_t u = 2 * one - t;
    const int32_t u32 = (int32_t)(u < 0 ? 0 : (u > INT32_MAX ? INT32_MAX : u));
    const int64_t r = ((int64_t)a * u32 + PCL_CUTOFF_ROUND30) >> PCL_Q30_SHIFT;

    return (pcl_coef_t)(r < 0 ? 0 : (r > one ? one : r));
}

#endif

#endif /* PCL_CUTOFF_H */
