/*
 * pcl_delayline.h - circular-buffer storage, wrap and fractional reads (internal). Used by the
 * delay; meant for the reverb's combs/all-passes and the sampler too.
 *
 * A line is `length` samples in caller memory, stored either as pcl_sample_t (PCL_DL_NATIVE) or as
 * int16_t Q15 (PCL_DL_Q15) in any build. `w` is the next write index; the newest sample is at w
 * - 1. Delays are uint32_t Q20.12 samples: d = i + f reads between x[n - i] and x[n - i - 1].
 *
 * Interpolation (J. O. Smith, "Physical Audio Signal Processing", fractional delay): linear, and
 * 4-point cubic Hermite (Catmull-Rom) through x[n-i+1], x[n-i], x[n-i-1], x[n-i-2]. The Hermite
 * read needs x[n-i+1] to exist, so i >= 2 when the current sample is not yet written.
 */
#ifndef PCL_DELAYLINE_H
#define PCL_DELAYLINE_H

#include "pcl/pcl_types.h"

#define PCL_DL_NATIVE 0
#define PCL_DL_Q15 1

#define PCL_DL_FRAC_BITS 12
#define PCL_DL_ONE ((uint32_t)1 << PCL_DL_FRAC_BITS)
#define PCL_DL_FRAC_MASK (PCL_DL_ONE - 1u)
#define PCL_DL_MAX_LENGTH ((uint32_t)1 << (32 - PCL_DL_FRAC_BITS)) /* samples */

PCL_INLINE size_t pcl_dl_elem_bytes(unsigned storage)
{
    return storage == PCL_DL_Q15 ? sizeof(int16_t) : sizeof(pcl_sample_t);
}

/* Index `back` samples before `w` (back <= length). */
PCL_INLINE uint32_t pcl_dl_back(uint32_t w, uint32_t back, uint32_t length)
{
    return w >= back ? w - back : w + length - back;
}

#if PCL_SAMPLE_TYPE == PCL_FLOAT
#define PCL_DL_Q15_SCALE 32768.0f
#endif

PCL_INLINE pcl_sample_t pcl_dl_load(const void *line, unsigned storage, uint32_t idx)
{
    if (storage == PCL_DL_Q15)
    {
        const int16_t v = ((const int16_t *)line)[idx];
#if PCL_SAMPLE_TYPE == PCL_FLOAT
        return (float)v * (1.0f / PCL_DL_Q15_SCALE);
#elif PCL_SAMPLE_TYPE == PCL_Q15
        return v;
#else
        return (int32_t)v * 65536;
#endif
    }
    return ((const pcl_sample_t *)line)[idx];
}

PCL_INLINE void pcl_dl_store(void *line, unsigned storage, uint32_t idx, pcl_sample_t v)
{
    if (storage == PCL_DL_Q15)
    {
#if PCL_SAMPLE_TYPE == PCL_FLOAT
        /* Round to nearest and saturate; written so that NaN stores 0. */
        float   s = v * PCL_DL_Q15_SCALE;
        int16_t q;
        if (s >= 32767.0f)
            q = INT16_MAX;
        else if (s <= -32768.0f)
            q = INT16_MIN;
        else if (s >= 0.0f)
            q = (int16_t)(s + 0.5f);
        else if (s < 0.0f)
            q = (int16_t)(s - 0.5f);
        else
            q = 0;
        ((int16_t *)line)[idx] = q;
#elif PCL_SAMPLE_TYPE == PCL_Q15
        ((int16_t *)line)[idx] = v;
#else
        ((int16_t *)line)[idx] = pcl_sat_q15((int32_t)(((int64_t)v + 32768) >> 16));
#endif
        return;
    }
    ((pcl_sample_t *)line)[idx] = v;
}

PCL_INLINE pcl_sample_t pcl_dl_saturate(int64_t v)
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

/* Linear: x[n-i] + f (x[n-i-1] - x[n-i]). Requires 1 <= i and i + 1 <= length. */
PCL_INLINE pcl_sample_t pcl_dl_read_linear(
    const void *line, unsigned storage, uint32_t length, uint32_t w, uint32_t d)
{
    const uint32_t     i = d >> PCL_DL_FRAC_BITS;
    const uint32_t     f = d & PCL_DL_FRAC_MASK;
    const uint32_t     k1 = pcl_dl_back(w, i, length);
    const uint32_t     k2 = k1 == 0u ? length - 1u : k1 - 1u;
    const pcl_sample_t p1 = pcl_dl_load(line, storage, k1);
    const pcl_sample_t p2 = pcl_dl_load(line, storage, k2);

#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return p1 + (p2 - p1) * ((float)f * (1.0f / (float)PCL_DL_ONE));
#elif PCL_SAMPLE_TYPE == PCL_Q15
    /* |p2 - p1| < 2^16 and f < 2^12: 32 bits suffice. The result lies between p1 and p2. */
    return (pcl_sample_t)(p1
        + ((((int32_t)p2 - p1) * (int32_t)f + (int32_t)(PCL_DL_ONE / 2u)) >> PCL_DL_FRAC_BITS));
#else
    return (pcl_sample_t)(p1
        + ((((int64_t)p2 - p1) * (int64_t)f + (int64_t)(PCL_DL_ONE / 2u)) >> PCL_DL_FRAC_BITS));
#endif
}

/*
 * 4-point cubic Hermite (Catmull-Rom) between p1 = x[n-i] and p2 = x[n-i-1], with p0 = x[n-i+1]
 * (newer) and p3 = x[n-i-2]. Requires 2 <= i and i + 2 <= length - 1. Computed with doubled
 * coefficients so fixed point needs no halves:
 *     C1 = p2 - p0,  C2 = 2 p0 - 5 p1 + 4 p2 - p3,  C3 = (p3 - p0) + 3 (p1 - p2)
 *     y  = p1 + t (C1 + t (C2 + t C3)) / 2
 * Can overshoot full scale between samples; fixed point saturates.
 */
PCL_INLINE pcl_sample_t pcl_dl_read_hermite(
    const void *line, unsigned storage, uint32_t length, uint32_t w, uint32_t d)
{
    const uint32_t     i = d >> PCL_DL_FRAC_BITS;
    const uint32_t     f = d & PCL_DL_FRAC_MASK;
    const uint32_t     k1 = pcl_dl_back(w, i, length);
    const uint32_t     k0 = k1 + 1u == length ? 0u : k1 + 1u;
    const uint32_t     k2 = k1 == 0u ? length - 1u : k1 - 1u;
    const uint32_t     k3 = k2 == 0u ? length - 1u : k2 - 1u;
    const pcl_sample_t p0 = pcl_dl_load(line, storage, k0);
    const pcl_sample_t p1 = pcl_dl_load(line, storage, k1);
    const pcl_sample_t p2 = pcl_dl_load(line, storage, k2);
    const pcl_sample_t p3 = pcl_dl_load(line, storage, k3);

#if PCL_SAMPLE_TYPE == PCL_FLOAT
    const float t = (float)f * (1.0f / (float)PCL_DL_ONE);
    const float c1 = p2 - p0;
    const float c2 = 2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3;
    const float c3 = (p3 - p0) + 3.0f * (p1 - p2);
    return p1 + 0.5f * t * (c1 + t * (c2 + t * c3));
#else
    /* |C| <= 12 full scales: below 2^35.6 in Q31, so Horner with a 12-bit t stays in 64 bits. */
    const int64_t c1 = (int64_t)p2 - p0;
    const int64_t c2 = 2 * (int64_t)p0 - 5 * (int64_t)p1 + 4 * (int64_t)p2 - p3;
    const int64_t c3 = ((int64_t)p3 - p0) + 3 * ((int64_t)p1 - p2);
    const int64_t half = (int64_t)(PCL_DL_ONE / 2u);
    int64_t       h = c3;

    h = c2 + ((h * (int64_t)f + half) >> PCL_DL_FRAC_BITS);
    h = c1 + ((h * (int64_t)f + half) >> PCL_DL_FRAC_BITS);
    h = (h * (int64_t)f + (int64_t)PCL_DL_ONE) >> (PCL_DL_FRAC_BITS + 1);
    return pcl_dl_saturate((int64_t)p1 + h);
#endif
}

#endif /* PCL_DELAYLINE_H */
