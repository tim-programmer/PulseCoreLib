/*
 * pcl_wave.h - phase-to-waveform helpers shared by the oscillator and the LFO (internal).
 *
 * Phases are uint32_t with 2^32 == one cycle. The sine comes from the shared quarter-wave
 * table with linear interpolation; the saw and triangle are exact ramps in Q1.31.
 */
#ifndef PCL_WAVE_H
#define PCL_WAVE_H

#include "core/pcl_tables.h"
#include "pcl/pcl_types.h"

#define PCL_WAVE_PHASE_HALF 0x80000000u
#define PCL_WAVE_PHASE_QUARTER 0x40000000u

/* Quarter-wave sine with linear interpolation; phase 2^32 = one cycle. Full scale = 1. */
PCL_INLINE pcl_sample_t pcl_wave_sine(uint32_t phase)
{
    /* Fold the phase into the first quadrant: odd quadrants run backwards. */
    const uint32_t b
        = (phase & PCL_WAVE_PHASE_QUARTER) ? (~phase & 0x3FFFFFFFu) : (phase & 0x3FFFFFFFu);
    const uint32_t     idx = b >> 22; /* 0 .. 255 */
    const uint32_t     frac = (b >> 6) & 0xFFFFu;
    const pcl_sample_t lo = pcl_sine_quarter_tab[idx];
    const pcl_sample_t hi = pcl_sine_quarter_tab[idx + 1u];
    pcl_sample_t       y;

    /* The table rises through the quadrant, so hi - lo >= 0. */
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    y = lo + (hi - lo) * ((float)frac * (1.0f / 65536.0f));
#elif PCL_SAMPLE_TYPE == PCL_Q15
    y = (pcl_sample_t)(lo + ((((int32_t)hi - (int32_t)lo) * (int32_t)frac + 0x8000) >> 16));
#else
    y = lo + (int32_t)((((uint32_t)hi - (uint32_t)lo) >> 8) * frac >> 8);
#endif

    /* y is in [0, max], so negating cannot overflow. */
    return (phase & PCL_WAVE_PHASE_HALF) ? (pcl_sample_t)-y : y;
}

/* Rising ramp -1 .. 1 as Q1.31. The phase's top bit flipped is a two's-complement ramp. */
PCL_INLINE int32_t pcl_wave_saw_w31(uint32_t phase)
{
    return (int32_t)(phase ^ PCL_WAVE_PHASE_HALF);
}

/* Triangle -1 at phase 0, +1 at phase 0.5, as Q1.31: 1 - 2|saw|. */
PCL_INLINE int32_t pcl_wave_tri_w31(uint32_t phase)
{
    const int32_t  s = pcl_wave_saw_w31(phase);
    const uint32_t a = (uint32_t)(s ^ (s >> 31)); /* |s| (ones' complement for s < 0) */
    return (int32_t)(0x7FFFFFFFu - (a << 1));
}

#endif /* PCL_WAVE_H */
