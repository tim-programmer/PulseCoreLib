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

#include "core/pcl_interp.h"
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

    return pcl_interp_linear(p1, p2, f, PCL_DL_FRAC_BITS);
}

/* Hermite between p1 = x[n-i] and p2 = x[n-i-1]. Requires 2 <= i and i + 2 <= length - 1. */
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

    return pcl_interp_hermite(p0, p1, p2, p3, f, PCL_DL_FRAC_BITS);
}

#endif /* PCL_DELAYLINE_H */
