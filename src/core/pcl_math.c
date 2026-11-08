/*
 * pcl_math.c - see include/pcl/pcl_math.h and docs/modules/math.md.
 *
 * Fixed-point builds: integer arithmetic only. The 64-bit products and divisions are in setter-rate
 * functions (they are integer library calls on a Cortex-M0+, not float).
 */
#include "pcl/pcl_math.h"

#include <string.h>

#include "core/pcl_octave.h"
#include "core/pcl_tables.h"
#include "core/pcl_wave.h"

#define MIDI_A4 69u
#define MIDI_MAX 127u
#define SEMITONES_PER_OCTAVE 12u
#define MAX_CENTS 9599

/*
 * Where the tangent switches from its power series to sin / cos: x = pi * f / fs = 0.5. The series
 * (through x^11) is off by 9e-7 relative there, the same as the table path, and below the limit it
 * is both cheaper and (in Q15, whose sine table has 15 bits) more accurate than sin / cos.
 */
#define TAN_SERIES_LIMIT 0.5

/* f / fs above which pcl_tan_hz clamps; tan(0.45 pi) = 6.31. */
#define TAN_MAX_RATIO_Q16 29491u              /* 0.45 * 2^16, truncated */
#define TAN_MAX_INC (TAN_MAX_RATIO_Q16 << 16) /* the same limit as a phase increment */

/* The tanh table covers x = 0 .. 6 in steps of 1/64; Q4.28 index = x >> 22. */
#define TANH_INDEX_SHIFT 22

#define LOG2_10_OVER_20 0.16609640474436813 /* log2(10) / 20: octaves per dB */
#define DB_PER_OCTAVE 6.020599913279624     /* 20 * log10(2) */

/* Largest pcl_hz_t-free constants shared by the builds. */
#define DB_FLOOR_OCT 20 /* octaves below 1.0 where gain_to_db gives up (-120.4 dB) */

#if PCL_SAMPLE_TYPE == PCL_FLOAT

PCL_STATIC_ASSERT(sizeof(float) == 4, float_is_32_bit);

#define OCT_MAX 7.99999905f /* nextafter(8, 0) */
#define OCT_MIN (-8.0f)

/* log2(x) for a positive, normal, finite float, from the table. */
static float log2_f(float x)
{
    uint32_t bits;
    uint32_t idx;
    float    frac;
    float    lo;
    float    hi;

    memcpy(&bits, &x, sizeof bits);
    idx = (bits & 0x7FFFFFu) >> 15; /* top 8 bits of the mantissa */
    frac = (float)(bits & 0x7FFFu) * (1.0f / 32768.0f);
    lo = (float)pcl_log2_tab[idx];
    hi = (float)pcl_log2_tab[idx + 1u];
    return (float)((int)((bits >> 23) & 0xFFu) - 127) + (lo + (hi - lo) * frac) * (1.0f / 65536.0f);
}

pcl_hz_t pcl_hz_scale(pcl_hz_t f, pcl_oct_t oct)
{
    if (!(oct >= OCT_MIN)) /* also catches NaN */
        oct = OCT_MIN;
    return pcl_octave_scale_f(f, oct);
}

pcl_oct_t pcl_hz_to_oct(pcl_hz_t f, pcl_hz_t ref)
{
    float q;
    float r;

    if (!(ref > 0.0f))
        return (f > 0.0f) ? OCT_MAX : 0.0f;
    if (!(f > 0.0f))
        return OCT_MIN;
    q = f / ref;
    if (q >= 256.0f)
        return OCT_MAX;
    if (q <= 1.0f / 256.0f)
        return OCT_MIN;
    r = log2_f(q);
    return r > OCT_MAX ? OCT_MAX : (r < OCT_MIN ? OCT_MIN : r);
}

pcl_hz_t pcl_midi_to_hz(uint8_t note)
{
    /* (note - 69) as whole octaves plus a remainder of 0..11 semitones, so octaves are exact. */
    const uint32_t u = (note > MIDI_MAX ? MIDI_MAX : note) + (6u * SEMITONES_PER_OCTAVE - MIDI_A4);
    const float    oct = (float)(int)(u / SEMITONES_PER_OCTAVE) - 6.0f
        + (float)(u % SEMITONES_PER_OCTAVE) * (1.0f / (float)SEMITONES_PER_OCTAVE);

    return pcl_octave_scale_f(440.0f, oct);
}

pcl_oct_t pcl_cents_to_oct(int32_t cents)
{
    if (cents > MAX_CENTS)
        cents = MAX_CENTS;
    else if (cents < -MAX_CENTS)
        cents = -MAX_CENTS;
    return (float)cents * (1.0f / 1200.0f);
}

pcl_coef_t pcl_db_to_gain(pcl_db_t db)
{
    static const float pow2_neg[DB_FLOOR_OCT + 1]
        = { 1.0f, 0.5f, 0.25f, 0.125f, 0.0625f, 0.03125f, 0.015625f, 0.0078125f, 0.00390625f,
              0.001953125f, 0.0009765625f, 0.00048828125f, 0.000244140625f, 0.0001220703125f,
              6.103515625e-05f, 3.0517578125e-05f, 1.52587890625e-05f, 7.62939453125e-06f,
              3.814697265625e-06f, 1.9073486328125e-06f, 9.5367431640625e-07f }; /* 2^0 .. 2^-20 */
    float oct;
    int   k;

    if (!(db >= PCL_DB_MIN)) /* also catches NaN */
        return 0.0f;
    oct = db * (float)LOG2_10_OVER_20;
    if (oct >= 1.0f)
        return 1.99999988f; /* the largest float below 2.0 */
    k = (int)oct;
    if ((float)k > oct)
        k--;
    return pcl_exp2_frac_f((oct - (float)k) * (float)PCL_EXP2_SEGMENTS) * pow2_neg[-k];
}

pcl_db_t pcl_gain_to_db(pcl_coef_t gain)
{
    float db;

    if (!(gain >= 1.0e-6f)) /* -120 dB; also catches zero, negative and NaN */
        return PCL_DB_MIN;
    if (gain > 1.0e30f)
        gain = 1.0e30f;
    db = log2_f(gain) * (float)DB_PER_OCTAVE;
    return db < PCL_DB_MIN ? PCL_DB_MIN : db;
}

/* tan(pi r) for r = f / fs in [0, 0.45]. */
static float tan_ratio_f(float r)
{
    /* tan x = x + x^3/3 + 2 x^5/15 + 17 x^7/315 + 62 x^9/2835 + 1382 x^11/155925 */
    const float pi = 3.14159265358979f;
    const float x = pi * r;

    if (x < (float)TAN_SERIES_LIMIT)
    {
        const float x2 = x * x;
        return x
            * (1.0f
                + x2
                    * (1.0f / 3.0f
                        + x2
                            * (2.0f / 15.0f
                                + x2
                                    * (17.0f / 315.0f
                                        + x2 * (62.0f / 2835.0f + x2 * (1382.0f / 155925.0f))))));
    }
    {
        const uint32_t phase = (uint32_t)(r * 2147483648.0f); /* pi r as a phase: r * 2^31 */
        return pcl_sin(phase) / pcl_cos(phase);
    }
}

pcl_wide_t pcl_tan_hz(pcl_hz_t f, uint32_t sample_rate_hz)
{
    float r;

    if (sample_rate_hz == 0u || !(f > 0.0f))
        return 0.0f;
    r = f / (float)sample_rate_hz;
    return tan_ratio_f(r > 0.45f ? 0.45f : r);
}

pcl_wide_t pcl_tan_inc(uint32_t phase_increment)
{
    const float r = (float)phase_increment * (1.0f / 4294967296.0f);

    return tan_ratio_f(r > 0.45f ? 0.45f : r);
}

pcl_sample_t pcl_tanh(pcl_wide_t x)
{
    float    a;
    float    pos;
    uint32_t idx;
    float    lo;
    float    y;

    if (x != x)
        return 0.0f;
    a = x < 0.0f ? -x : x;
    if (!(a < (float)(PCL_TANH_SEGMENTS) / (float)PCL_TANH_STEPS_PER_UNIT))
    {
        y = pcl_tanh_tab[PCL_TANH_SEGMENTS];
    }
    else
    {
        pos = a * (float)PCL_TANH_STEPS_PER_UNIT;
        idx = (uint32_t)pos;
        if (idx >= PCL_TANH_SEGMENTS)
            idx = PCL_TANH_SEGMENTS - 1u;
        lo = pcl_tanh_tab[idx];
        y = lo + (pcl_tanh_tab[idx + 1u] - lo) * (pos - (float)idx);
    }
    return x < 0.0f ? -y : y;
}

#else /* fixed point */

#define OCT_MAX INT32_MAX
#define OCT_MIN INT32_MIN

/* Position of the highest set bit of x != 0. A fixed number of steps, no intrinsic. */
static uint32_t msb32(uint32_t x)
{
    uint32_t n = 0u;

    if (x >> 16)
    {
        n += 16u;
        x >>= 16;
    }
    if (x >> 8)
    {
        n += 8u;
        x >>= 8;
    }
    if (x >> 4)
    {
        n += 4u;
        x >>= 4;
    }
    if (x >> 2)
    {
        n += 2u;
        x >>= 2;
    }
    if (x >> 1)
        n += 1u;
    return n;
}

/* log2 of an integer x != 0 in Q.28, from the table (0 .. 32 octaves, so 64-bit). */
static int64_t log2_q28(uint32_t x)
{
    const uint32_t n = msb32(x);
    const uint32_t m = (x << (31u - n)) & 0x7FFFFFFFu; /* the bits below the leading one, Q0.31 */
    const uint32_t idx = m >> 23;
    const uint32_t frac = (m >> 7) & 0xFFFFu;
    const uint32_t lo = pcl_log2_tab[idx];
    const uint32_t hi = pcl_log2_tab[idx + 1u];
    const uint32_t v = lo + (((hi - lo) * frac + 0x8000u) >> 16); /* log2(1 + m) * 2^16 */

    return ((int64_t)n << PCL_Q28_SHIFT) + ((int64_t)v << (PCL_Q28_SHIFT - 16));
}

/* Rounded (x * k) >> shift for a signed 64-bit product. */
static int64_t mul_shift(int64_t x, int64_t k, unsigned shift)
{
    return (x * k + ((int64_t)1 << (shift - 1u))) >> shift;
}

pcl_hz_t pcl_hz_scale(pcl_hz_t f, pcl_oct_t oct)
{
    const int32_t  k = oct >> PCL_Q28_SHIFT; /* -8 .. 7 */
    const uint32_t mant = pcl_exp2m1_q16((uint32_t)oct & ((1u << PCL_Q28_SHIFT) - 1u));
    uint64_t       r = (uint64_t)f * (65536u + mant); /* f * 2^f, scaled by 2^16: < 2^49 */

    if (k >= 0)
        r <<= k;
    else
        r >>= -k;
    r = (r + 0x8000u) >> 16;
    return r > UINT32_MAX ? UINT32_MAX : (pcl_hz_t)r;
}

pcl_oct_t pcl_hz_to_oct(pcl_hz_t f, pcl_hz_t ref)
{
    int64_t d;

    if (ref == 0u)
        return (f != 0u) ? OCT_MAX : 0;
    if (f == 0u)
        return OCT_MIN;
    d = log2_q28(f) - log2_q28(ref);
    return d > INT32_MAX ? INT32_MAX : (d < INT32_MIN ? INT32_MIN : (pcl_oct_t)d);
}

pcl_hz_t pcl_midi_to_hz(uint8_t note)
{
    /* (note - 69) as whole octaves plus a remainder of 0..11 semitones, so octaves are exact. */
    const uint32_t u = (note > MIDI_MAX ? MIDI_MAX : note) + (6u * SEMITONES_PER_OCTAVE - MIDI_A4);
    const int32_t  whole = (int32_t)(u / SEMITONES_PER_OCTAVE) - 6;
    const int32_t  rem = (int32_t)(u % SEMITONES_PER_OCTAVE);

    return pcl_hz_scale(PCL_HZ(440.0), whole * (1 << PCL_Q28_SHIFT) + rem * 22369621); /* 2^28/12 */
}

pcl_oct_t pcl_cents_to_oct(int32_t cents)
{
    if (cents > MAX_CENTS)
        cents = MAX_CENTS;
    else if (cents < -MAX_CENTS)
        cents = -MAX_CENTS;
    return (pcl_oct_t)mul_shift(cents, 234562480592, 20); /* 2^28 / 1200 * 2^20 */
}

pcl_coef_t pcl_db_to_gain(pcl_db_t db)
{
    int64_t  x;
    uint32_t g;
    uint32_t shift;

    if (db < PCL_DB_MIN)
        return 0;
    /* Octaves in Q4.28: db (Q8.24) * log2(10)/20 * 16. */
    x = mul_shift(db, 2853514505, 30);
    if (x >= ((int64_t)1 << PCL_Q28_SHIFT))
        return INT32_MAX;
    shift = (uint32_t)(-(x >> PCL_Q28_SHIFT));                                       /* 0 .. 21 */
    g = (65536u + pcl_exp2m1_q16((uint32_t)x & ((1u << PCL_Q28_SHIFT) - 1u))) << 14; /* Q2.30 */
    if (shift > 0u)
        g = (g + (1u << (shift - 1u))) >> shift;
    return (pcl_coef_t)g;
}

pcl_db_t pcl_gain_to_db(pcl_coef_t gain)
{
    int64_t oct;
    int64_t db;

    if (gain <= 0)
        return PCL_DB_MIN;
    oct = log2_q28((uint32_t)gain) - ((int64_t)30 << PCL_Q28_SHIFT); /* gain is Q2.30 */
    if (oct < -((int64_t)DB_FLOOR_OCT << PCL_Q28_SHIFT))
        return PCL_DB_MIN;
    db = mul_shift(oct, 1616142483, 32); /* Q4.28 octaves -> Q8.24 dB: * 20 log10(2) / 16 */
    return db < PCL_DB_MIN ? PCL_DB_MIN : (pcl_db_t)db;
}

#if PCL_SAMPLE_TYPE == PCL_Q15
#define SAMPLE_TO_Q31_SHIFT (PCL_Q31_SHIFT - PCL_Q15_SHIFT)
#else
#define SAMPLE_TO_Q31_SHIFT 0
#endif

/* Power-series coefficients of tan, Q4.28. */
static const int32_t tan_series_limit = PCL_CONST_Q28(TAN_SERIES_LIMIT);
static const int32_t tan_c3 = PCL_CONST_Q28(1.0 / 3.0);
static const int32_t tan_c5 = PCL_CONST_Q28(2.0 / 15.0);
static const int32_t tan_c7 = PCL_CONST_Q28(17.0 / 315.0);
static const int32_t tan_c9 = PCL_CONST_Q28(62.0 / 2835.0);
static const int32_t tan_c11 = PCL_CONST_Q28(1382.0 / 155925.0);

pcl_wide_t pcl_tan_hz(pcl_hz_t f, uint32_t sample_rate_hz)
{
    uint64_t fmax;
    uint64_t fr;

    if (sample_rate_hz == 0u)
        return 0;
    fmax = (uint64_t)sample_rate_hz * TAN_MAX_RATIO_Q16;
    fr = f > fmax ? fmax : f;
    return pcl_tan_inc((uint32_t)((fr << 16) / sample_rate_hz)); /* f / fs as r * 2^32 */
}

pcl_wide_t pcl_tan_inc(uint32_t phase_increment)
{
    const uint32_t inc = phase_increment > TAN_MAX_INC ? TAN_MAX_INC : phase_increment;
    const int32_t  x = (int32_t)(((uint64_t)inc * 843314857u) >> 32); /* pi r, Q4.28 */

    if (x < tan_series_limit)
    {
        const int64_t x2 = mul_shift(x, x, PCL_Q28_SHIFT);
        int64_t       t = tan_c11;

        t = tan_c9 + mul_shift(x2, t, PCL_Q28_SHIFT);
        t = tan_c7 + mul_shift(x2, t, PCL_Q28_SHIFT);
        t = tan_c5 + mul_shift(x2, t, PCL_Q28_SHIFT);
        t = tan_c3 + mul_shift(x2, t, PCL_Q28_SHIFT);
        t = ((int64_t)1 << PCL_Q28_SHIFT) + mul_shift(x2, t, PCL_Q28_SHIFT);
        return (pcl_wide_t)mul_shift(x, t, PCL_Q28_SHIFT);
    }
    else
    {
        /* sin(pi r) / cos(pi r): the angle pi r is the phase inc / 2, below a quarter cycle, so
         * both are positive and cos >= 0.156. Widen the Q15 values to Q31 first. */
        const uint32_t phase = inc >> 1;
        const uint32_t s = (uint32_t)pcl_sin(phase) << SAMPLE_TO_Q31_SHIFT;
        const uint32_t c = (uint32_t)pcl_cos(phase) << SAMPLE_TO_Q31_SHIFT;

        return (pcl_wide_t)((((uint64_t)s << PCL_Q28_SHIFT) + (c >> 1)) / c);
    }
}

pcl_sample_t pcl_tanh(pcl_wide_t x)
{
    const uint32_t a = x < 0 ? 0u - (uint32_t)x : (uint32_t)x;
    pcl_sample_t   y;

    if (a >= (PCL_TANH_SEGMENTS << TANH_INDEX_SHIFT))
    {
        y = pcl_tanh_tab[PCL_TANH_SEGMENTS];
    }
    else
    {
        const uint32_t     idx = a >> TANH_INDEX_SHIFT;
        const uint32_t     frac = (a >> (TANH_INDEX_SHIFT - 16)) & 0xFFFFu;
        const pcl_sample_t lo = pcl_tanh_tab[idx];
        const pcl_sample_t hi = pcl_tanh_tab[idx + 1u]; /* the table rises, so hi >= lo */

#if PCL_SAMPLE_TYPE == PCL_Q15
        y = (pcl_sample_t)(lo + ((((int32_t)hi - (int32_t)lo) * (int32_t)frac + 0x8000) >> 16));
#else
        /* (hi - lo) is below 2^25; shifting it down first keeps the product within 32 bits. */
        y = lo + (int32_t)(((((uint32_t)hi - (uint32_t)lo) >> 10) * frac + 0x20u) >> 6);
#endif
    }
    return x < 0 ? (pcl_sample_t)-y : y;
}

#endif

pcl_sample_t pcl_sin(uint32_t phase) { return pcl_wave_sine(phase); }

pcl_sample_t pcl_cos(uint32_t phase) { return pcl_wave_sine(phase + PCL_WAVE_PHASE_QUARTER); }
