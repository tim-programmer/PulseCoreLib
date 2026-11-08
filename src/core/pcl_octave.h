/*
 * pcl_octave.h - scale a frequency or phase increment by 2^octaves (internal).
 *
 * Shared by the oscillator (pitch input) and the LFO (rate input). No exp2f: the fractional
 * octave comes from the 256-segment 2^x table with linear interpolation, so the three sample
 * types behave alike and fixed-point builds use no float.
 */
#ifndef PCL_OCTAVE_H
#define PCL_OCTAVE_H

#include "core/pcl_tables.h"
#include "pcl/pcl_types.h"

#if PCL_SAMPLE_TYPE == PCL_FLOAT

/*
 * 2^x for x in [0, 1) from the table with linear interpolation, as 1 + (the interpolated
 * (2^x - 1) * 2^16) / 2^16. `pos` is x * PCL_EXP2_SEGMENTS.
 */
PCL_INLINE float pcl_exp2_frac_f(float pos)
{
    uint32_t idx = (uint32_t)pos;

    if (idx >= PCL_EXP2_SEGMENTS)
        idx = PCL_EXP2_SEGMENTS - 1u;
    pos -= (float)idx;
    return 1.0f
        + ((float)pcl_exp2_tab[idx]
              + (float)((int32_t)pcl_exp2_tab[idx + 1u] - (int32_t)pcl_exp2_tab[idx]) * pos)
        * (1.0f / 65536.0f);
}

/* f * 2^oct, with oct clamped to [-8, 8). */
PCL_INLINE float pcl_octave_scale_f(float f, float oct)
{
    static const float pow2_tab[16]
        = { 0.00390625f, 0.0078125f, 0.015625f, 0.03125f, 0.0625f, 0.125f, 0.25f, 0.5f, 1.0f, 2.0f,
              4.0f, 8.0f, 16.0f, 32.0f, 64.0f, 128.0f }; /* 2^-8 .. 2^7 */
    int k;

    if (oct > 7.999f)
        oct = 7.999f;
    else if (oct < -8.0f)
        oct = -8.0f;
    k = (int)oct;
    if ((float)k > oct)
        k--;
    return f * (pcl_exp2_frac_f((oct - (float)k) * (float)PCL_EXP2_SEGMENTS) * pow2_tab[k + 8]);
}

#else

/*
 * (2^f - 1) * 2^16, rounded, for f in [0, 1) given as Q0.28 (the fraction of a Q4.28 octave);
 * 32-bit arithmetic only.
 */
PCL_INLINE uint32_t pcl_exp2m1_q16(uint32_t f)
{
    const uint32_t idx = f >> 20; /* top 8 bits of the fraction */
    const uint32_t frac = (f >> 4) & 0xFFFFu;
    const uint32_t lo = pcl_exp2_tab[idx];
    const uint32_t hi = pcl_exp2_tab[idx + 1u];

    return lo + (((hi - lo) * frac + 0x8000u) >> 16);
}

/*
 * base * 2^oct for an unsigned 32-bit `base`, oct in Q4.28 (range [-8, 8)), 32-bit arithmetic only
 * (no 32x32 -> 64 multiply, so it is cheap on Cortex-M0+). The result saturates at `inc_max`
 * when shifted up; the caller applies any final clamp.
 */
PCL_INLINE uint32_t pcl_octave_scale_inc(uint32_t base, int32_t oct, uint32_t inc_max)
{
    const int32_t  k = oct >> PCL_Q28_SHIFT; /* -8 .. 7 */
    const uint32_t mant = pcl_exp2m1_q16((uint32_t)oct & ((1u << PCL_Q28_SHIFT) - 1u));
    /* base * 2^f = base + base * mant / 2^16, in 32-bit pieces. */
    uint32_t inc = base + (base >> 16) * mant + (((base & 0xFFFFu) * mant) >> 16);

    if (k > 0)
    {
        if (inc > (inc_max >> k))
            inc = inc_max;
        else
            inc <<= k;
    }
    else if (k < 0)
    {
        inc >>= -k;
    }
    return inc;
}

#endif

#endif /* PCL_OCTAVE_H */
