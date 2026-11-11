/*
 * pcl_interp.h - fractional interpolation kernels shared by the delay line and the sampler
 * (internal). Moved out of pcl_delayline.h unchanged; the delay's output is bit-identical.
 *
 * Both kernels interpolate between p1 (t = 0) and p2 (t = 1), with t = f / 2^bits, f < 2^bits,
 * bits <= 16. Hermite also takes p0 (before p1) and p3 (after p2) in the direction of t.
 *
 *   linear:  p1 + t (p2 - p1)
 *   Hermite: 4-point cubic (Catmull-Rom) with doubled coefficients, so fixed point needs no halves:
 *            C1 = p2 - p0,  C2 = 2 p0 - 5 p1 + 4 p2 - p3,  C3 = (p3 - p0) + 3 (p1 - p2)
 *            y  = p1 + t (C1 + t (C2 + t C3)) / 2
 *
 * J. O. Smith, "Physical Audio Signal Processing" (fractional delay / interpolated table lookup).
 * Rounding uses arithmetic right shifts (asserted in pcl_types.h). Hermite can overshoot full scale
 * between samples; fixed point saturates.
 */
#ifndef PCL_INTERP_H
#define PCL_INTERP_H

#include "pcl/pcl_types.h"

/* A 64-bit fixed-point sample value, saturated to the sample range (fixed point only). */
PCL_INLINE pcl_sample_t pcl_interp_sat(int64_t v)
{
#if PCL_SAMPLE_TYPE == PCL_Q15
    return pcl_sat_q15((int32_t)(v > INT32_MAX ? INT32_MAX : (v < INT32_MIN ? INT32_MIN : v)));
#elif PCL_SAMPLE_TYPE == PCL_Q31
    return pcl_sat_q31(v);
#else
    (void)v;
    return 0;
#endif
}

PCL_INLINE pcl_sample_t pcl_interp_linear(
    pcl_sample_t p1, pcl_sample_t p2, uint32_t f, unsigned bits)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return p1 + (p2 - p1) * ((float)f * (1.0f / (float)(1u << bits)));
#elif PCL_SAMPLE_TYPE == PCL_Q15
    /* |p2 - p1| < 2^16: with bits <= 15 the product fits 32 bits (a constant test once inlined). */
    if (bits <= 15u)
        return (pcl_sample_t)(p1
            + ((((int32_t)p2 - p1) * (int32_t)f + (int32_t)(1u << (bits - 1u))) >> bits));
    return (pcl_sample_t)(p1
        + ((((int64_t)p2 - p1) * (int64_t)f + (int64_t)(1u << (bits - 1u))) >> bits));
#else
    return (pcl_sample_t)(p1
        + ((((int64_t)p2 - p1) * (int64_t)f + (int64_t)(1u << (bits - 1u))) >> bits));
#endif
}

PCL_INLINE pcl_sample_t pcl_interp_hermite(
    pcl_sample_t p0, pcl_sample_t p1, pcl_sample_t p2, pcl_sample_t p3, uint32_t f, unsigned bits)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    const float t = (float)f * (1.0f / (float)(1u << bits));
    const float c1 = p2 - p0;
    const float c2 = 2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3;
    const float c3 = (p3 - p0) + 3.0f * (p1 - p2);
    return p1 + 0.5f * t * (c1 + t * (c2 + t * c3));
#else
    /* |C| <= 12 full scales (below 2^35.6 in Q31) times f < 2^16: within 64 bits. */
    const int64_t c1 = (int64_t)p2 - p0;
    const int64_t c2 = 2 * (int64_t)p0 - 5 * (int64_t)p1 + 4 * (int64_t)p2 - p3;
    const int64_t c3 = ((int64_t)p3 - p0) + 3 * ((int64_t)p1 - p2);
    const int64_t half = (int64_t)1 << (bits - 1u);
    int64_t       h = c3;

    h = c2 + ((h * (int64_t)f + half) >> bits);
    h = c1 + ((h * (int64_t)f + half) >> bits);
    h = (h * (int64_t)f + ((int64_t)1 << bits)) >> (bits + 1u);
    return pcl_interp_sat((int64_t)p1 + h);
#endif
}

#endif /* PCL_INTERP_H */
