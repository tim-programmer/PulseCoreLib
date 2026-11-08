/*
 * pcl_math.h - shared conversions and shapes: pitch, level, sine, filter pre-warp, tanh.
 *
 * Design note: docs/modules/math.md.
 *
 * Every function is pure (no state, no allocation, safe from any thread) and none can fail, so none
 * returns a status: an argument outside the documented domain is clamped as stated. The fixed-point
 * builds use no float. All three builds use the same table-based algorithms (no expf, tanhf, tanf
 * or log2f), so they agree to within their own resolution.
 *
 * Rate: the pitch, level and pcl_tan_hz functions are meant for SETTERS (when a parameter changes,
 * at most a few times per block); pcl_sin, pcl_cos, pcl_tanh and the two conversions are cheap
 * enough for the audio loop. Nothing here is a module: there is no init or reset.
 */
#ifndef PCL_MATH_H
#define PCL_MATH_H

#include "pcl/pcl_types.h"

PCL_BEGIN_DECLS

/* ---- Pitch ---- */

/*
 * f * 2^oct. `oct` is clamped to [-8, 8); in fixed-point builds the result saturates at the largest
 * pcl_hz_t (65535.99998 Hz). Accurate to 8e-6 relative (0.013 cents).
 */
pcl_hz_t pcl_hz_scale(pcl_hz_t f, pcl_oct_t oct);

/*
 * log2(f / ref): how far f is above ref, in octaves, clamped to [-8, 8). f == 0 gives the lowest
 * octave and ref == 0 the highest (never a division by zero); both zero give 0. Accurate to 8e-6
 * octaves (0.01 cents) before the clamp.
 */
pcl_oct_t pcl_hz_to_oct(pcl_hz_t f, pcl_hz_t ref);

/* MIDI note 0 .. 127 to Hz, equal temperament with note 69 = 440 Hz. Larger notes count as 127. */
pcl_hz_t pcl_midi_to_hz(uint8_t note);

/* cents / 1200 as an octave amount (100 cents = a semitone). Clamped to +-9599 cents. */
pcl_oct_t pcl_cents_to_oct(int32_t cents);

/* ---- Level ---- */

/*
 * 10^(db / 20) as a gain. Below PCL_DB_MIN (-120 dB) the gain is exactly 0. At 6.0206 dB (a gain of
 * 2.0, which Q2.30 cannot hold) and above it saturates at 1.9999999 in every build, so the three
 * agree. Accurate to 8e-6 relative (0.00007 dB). NaN gives 0.
 */
pcl_coef_t pcl_db_to_gain(pcl_db_t db);

/* 20 * log10(gain). A gain <= 0 (or NaN), or below -120 dB, gives PCL_DB_MIN. Accurate to 5e-5 dB.
 */
pcl_db_t pcl_gain_to_db(pcl_coef_t gain);

/* ---- Shapes ---- */

/*
 * sin / cos of an angle given as a phase: uint32_t, 2^32 = one cycle. Full scale = the sample
 * format's largest value. Linear interpolation of the shared quarter-wave table: 4.7e-6 absolute.
 */
pcl_sample_t pcl_sin(uint32_t phase);
pcl_sample_t pcl_cos(uint32_t phase);

/*
 * tan(pi * f / sample_rate_hz): the pre-warped cutoff coefficient of a bilinear / TPT filter.
 * `f` is clamped to 0.45 * sample_rate_hz (tan is 6.31 there, the format holds up to 8, and the
 * pole is at 0.5); `sample_rate_hz` must be a valid rate (a 0 returns 0). Setter rate: it contains
 * a division (or two). Better than 0.6 cents of cutoff from 5 Hz to 21 kHz at 48 kHz.
 */
pcl_wide_t pcl_tan_hz(pcl_hz_t f, uint32_t sample_rate_hz);

/*
 * The same as pcl_tan_hz with f / fs already formed as a phase increment (2^32 = the sample rate):
 * tan(pi * phase_increment / 2^32), clamped at 0.45 * 2^32. A module that holds its frequency as an
 * increment (the oscillator, LFO and filters do) avoids pcl_tan_hz's division with this. Setter
 * rate.
 */
pcl_wide_t pcl_tan_inc(uint32_t phase_increment);

/*
 * tanh(x) as a sample, for x of any size: beyond +-6 the output holds at +-0.99999 (the table's
 * last entry). Odd-symmetric (tanh(-x) == -tanh(x) exactly), monotone, 2.3e-5 absolute error. Audio
 * rate. A NaN input gives 0.
 */
pcl_sample_t pcl_tanh(pcl_wide_t x);

/* ---- Sample <-> wide ---- */

/*
 * Between the sample format and pcl_wide_t (identity in the float build). In the Q31 build the
 * wide value has 28 fraction bits, so the conversion drops the 3 least significant bits (rounded).
 * pcl_wide_to_sample saturates.
 */
PCL_INLINE pcl_wide_t pcl_sample_to_wide(pcl_sample_t x)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return x;
#elif PCL_SAMPLE_TYPE == PCL_Q15
    return (pcl_wide_t)((int32_t)x * (1 << (PCL_Q28_SHIFT - PCL_Q15_SHIFT)));
#else
    /* Rounded; the one input whose rounding would overflow stays at the largest value. */
    return (x > INT32_MAX - 4) ? (pcl_wide_t)(INT32_MAX >> (PCL_Q31_SHIFT - PCL_Q28_SHIFT))
                               : (pcl_wide_t)((x + 4) >> (PCL_Q31_SHIFT - PCL_Q28_SHIFT));
#endif
}

PCL_INLINE pcl_sample_t pcl_wide_to_sample(pcl_wide_t x)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return x;
#elif PCL_SAMPLE_TYPE == PCL_Q15
    if (x >= (1 << PCL_Q28_SHIFT))
        return INT16_MAX;
    if (x <= -(1 << PCL_Q28_SHIFT))
        return INT16_MIN;
    return pcl_sat_q15(
        (x + (1 << (PCL_Q28_SHIFT - PCL_Q15_SHIFT - 1))) >> (PCL_Q28_SHIFT - PCL_Q15_SHIFT));
#else
    if (x >= (1 << PCL_Q28_SHIFT))
        return INT32_MAX;
    if (x <= -(1 << PCL_Q28_SHIFT))
        return INT32_MIN;
    return (pcl_sample_t)((uint32_t)x << (PCL_Q31_SHIFT - PCL_Q28_SHIFT));
#endif
}

PCL_END_DECLS

#endif /* PCL_MATH_H */
