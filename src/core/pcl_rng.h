/*
 * pcl_rng.h - internal PRNG (xorshift32). Not part of the public API.
 *
 * G. Marsaglia, "Xorshift RNGs", J. Statistical Software 8(14), 2003 (shifts 13/17/5).
 * The state must be non-zero; pcl_rng_seed() maps any seed to a valid state.
 */
#ifndef PCL_RNG_H
#define PCL_RNG_H

#include "pcl/pcl_noise.h"
#include "pcl/pcl_types.h"

#define PCL_RNG_SHIFT_A 13
#define PCL_RNG_SHIFT_B 17
#define PCL_RNG_SHIFT_C 5

PCL_INLINE uint32_t pcl_rng_seed(uint32_t seed)
{
    return seed != 0u ? seed : PCL_NOISE_DEFAULT_SEED;
}

/* Advances the state and returns it; uniform over 1 .. 2^32-1. */
PCL_INLINE uint32_t pcl_rng_next(uint32_t *state)
{
    uint32_t x = *state;
    x ^= x << PCL_RNG_SHIFT_A;
    x ^= x >> PCL_RNG_SHIFT_B;
    x ^= x << PCL_RNG_SHIFT_C;
    *state = x;
    return x;
}

/* The draw as a signed 32-bit value (two's complement reinterpretation). */
PCL_INLINE int32_t pcl_rng_s32(uint32_t draw) { return (int32_t)draw; }

#endif /* PCL_RNG_H */
