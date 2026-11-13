/*
 * pcl_math tests: every function against an independent double-precision model, evaluated at
 * the quantised input the function actually receives. Tolerances come from the measured error of
 * each function (docs/modules/math.md) with some margin; none is looser than it needs to be.
 */
#include <stdlib.h>

#include "pcl_test.h"

static const double pi = 3.14159265358979323846;

/* ---- Format helpers: one source for all three builds ---- */

#if PCL_SAMPLE_TYPE == PCL_FLOAT
#define HZ_D(f) ((double)(f))
#define OCT_D(o) ((double)(o))
#define DB_D(d) ((double)(d))
#define WIDE_D(w) ((double)(w))
#define HZ_OF(x) ((float)(x))
#define OCT_OF(x) ((float)(x))
#define DB_OF(x) ((float)(x))
#define WIDE_OF(x) ((float)(x))
#define COEF_OF(x) ((float)(x))
#define IS_FLOAT 1
#else
#define HZ_D(f) ((double)(f) / 65536.0)
#define OCT_D(o) ((double)(o) / 268435456.0)
#define DB_D(d) ((double)(d) / 16777216.0)
#define WIDE_D(w) ((double)(w) / 268435456.0)
#define HZ_OF(x) ((uint32_t)floor((x) * 65536.0 + 0.5))
#define OCT_OF(x) ((int32_t)floor((x) * 268435456.0 + 0.5))
#define DB_OF(x) ((int32_t)floor((x) * 16777216.0 + 0.5))
#define WIDE_OF(x) ((int32_t)floor((x) * 268435456.0 + 0.5))
#define COEF_OF(x) ((int32_t)floor((x) * 1073741824.0 + 0.5))
#define IS_FLOAT 0
#endif

/* Tolerances (see the file comment). */
#if PCL_SAMPLE_TYPE == PCL_FLOAT
#define HZ_LSB 0.0 /* float has no fixed resolution */
#define HZ_REL 1.0e-5
#define OCT_TOL 2.0e-5
#define TANH_TOL 3.0e-5
#define SIN_TOL 6.0e-6
#define TAN_ABS 1.0e-9
#define TAN_REL 2.0e-6
#define TAN_REL_HIGH 2.0e-6
#define JUNCTION_BACK_TOL 1.0e-30
#define JUNCTION_BACK_MAX 0
#elif PCL_SAMPLE_TYPE == PCL_Q15
#define HZ_LSB (1.5 / 65536.0)
#define HZ_REL 1.6e-5 /* two 16-bit roundings: table entry and interpolated mantissa */
#define OCT_TOL 2.5e-5
#define TANH_TOL 8.0e-5 /* 2.5 Q15 LSB: the table's 2.3e-5 plus rounding */
#define SIN_TOL 8.0e-5
#define TAN_ABS 1.0e-8
#define TAN_REL 3.0e-6
#define TAN_REL_HIGH 3.0e-4 /* from x = 0.5 on, the Q15 sine table limits it */
#define JUNCTION_BACK_TOL 5.0e-5
#define JUNCTION_BACK_MAX 1
#else
#define HZ_LSB (1.5 / 65536.0)
#define HZ_REL 1.6e-5
#define OCT_TOL 2.5e-5
#define TANH_TOL 3.0e-5
#define SIN_TOL 6.0e-6
#define TAN_ABS 1.0e-8
#define TAN_REL 3.0e-6
#define TAN_REL_HIGH 3.0e-6
#define JUNCTION_BACK_TOL 1.0e-30
#define JUNCTION_BACK_MAX 0
#endif

static double coef_d(pcl_coef_t c) { return pcl_test_coef_to_double(c); }

static double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

#define LARGEST_HZ 65535.99998474121

/* ---- Pitch ---- */

static void test_hz_scale_accuracy(void)
{
    double f;
    double o;

    for (f = 10.0; f <= 8000.0; f *= 1.37)
    {
        const pcl_hz_t fq = HZ_OF(f);
        for (o = -7.99; o < 7.99; o += 0.0137)
        {
            const double ref = HZ_D(fq) * pow(2.0, OCT_D(OCT_OF(o)));
            const double got = HZ_D(pcl_hz_scale(fq, OCT_OF(o)));
            if (!IS_FLOAT && ref > LARGEST_HZ)
                PCL_CHECK_NEAR(got, LARGEST_HZ, 1e-9); /* saturates */
            else
                PCL_CHECK_NEAR(got, ref, ref * HZ_REL + HZ_LSB);
        }
    }
}

static void test_hz_scale_landmarks_and_clamps(void)
{
    PCL_CHECK_NEAR(HZ_D(pcl_hz_scale(HZ_OF(440.0), OCT_OF(0.0))), 440.0, 0.0); /* oct 0: exact */
    PCL_CHECK_NEAR(HZ_D(pcl_hz_scale(HZ_OF(440.0), OCT_OF(1.0))), 880.0, 1e-3);
    PCL_CHECK_NEAR(HZ_D(pcl_hz_scale(HZ_OF(440.0), OCT_OF(-1.0))), 220.0, 1e-3);
    PCL_CHECK_NEAR(HZ_D(pcl_hz_scale(HZ_OF(0.0), OCT_OF(3.0))), 0.0, 0.0);
    /* The octave clamps at -8: 1 Hz can go no lower than 1/256. */
    PCL_CHECK_NEAR(HZ_D(pcl_hz_scale(HZ_OF(1.0), OCT_OF(-7.999))), 1.0 / 256.0, 1e-4);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    PCL_CHECK_NEAR(pcl_hz_scale(1.0f, -100.0f), 1.0 / 256.0, 1e-6);
    PCL_CHECK_NEAR(pcl_hz_scale(1.0f, 100.0f), 256.0, 0.5);
    PCL_CHECK_NEAR(pcl_hz_scale(1.0f, NAN), 1.0 / 256.0, 1e-6); /* NaN clamps, no UB */
    PCL_CHECK_NEAR(pcl_hz_scale(1.0f, -INFINITY), 1.0 / 256.0, 1e-6);
    PCL_CHECK_NEAR(pcl_hz_scale(1.0f, INFINITY), 256.0, 0.5);
#else
    PCL_CHECK_EQ_INT(pcl_hz_scale(HZ_OF(60000.0), OCT_OF(3.0)), UINT32_MAX);
    PCL_CHECK_EQ_INT(pcl_hz_scale(UINT32_MAX, OCT_OF(0.0)), UINT32_MAX);
    PCL_CHECK_EQ_INT(pcl_hz_scale(UINT32_MAX, INT32_MAX), UINT32_MAX);
    PCL_CHECK_EQ_INT(pcl_hz_scale(1u, INT32_MIN), 0); /* 1/256 of a LSB rounds to 0 */
    PCL_CHECK_EQ_INT(pcl_hz_scale(UINT32_MAX, INT32_MIN), 16777216); /* 2^32 / 256 */
#endif
}

static void test_hz_scale_monotone(void)
{
    double   o;
    double   prev = -1.0;
    pcl_hz_t f = HZ_OF(100.0);

    for (o = -7.99; o < 3.5; o += 0.0007)
    {
        const double v = HZ_D(pcl_hz_scale(f, OCT_OF(o)));
        PCL_CHECK(v >= prev);
        prev = v;
    }
}

static void test_hz_to_oct_accuracy(void)
{
    double f;
    double r;

    for (f = 1.0; f <= 20000.0; f *= 1.31)
        for (r = 1.0; r <= 20000.0; r *= 1.29)
        {
            const pcl_hz_t fq = HZ_OF(f);
            const pcl_hz_t rq = HZ_OF(r);
            const double   ref = clampd(log2(HZ_D(fq) / HZ_D(rq)), -8.0, 8.0);
            PCL_CHECK_NEAR(OCT_D(pcl_hz_to_oct(fq, rq)), ref, OCT_TOL);
        }
}

static void test_hz_to_oct_edges(void)
{
    PCL_CHECK_NEAR(OCT_D(pcl_hz_to_oct(HZ_OF(440.0), HZ_OF(440.0))), 0.0, 1e-9);
    PCL_CHECK_NEAR(OCT_D(pcl_hz_to_oct(HZ_OF(880.0), HZ_OF(440.0))), 1.0, OCT_TOL);
    PCL_CHECK_NEAR(OCT_D(pcl_hz_to_oct(HZ_OF(110.0), HZ_OF(440.0))), -2.0, OCT_TOL);
    /* Zero: the extremes, never a division by zero. */
    PCL_CHECK_NEAR(OCT_D(pcl_hz_to_oct(HZ_OF(0.0), HZ_OF(440.0))), -8.0, 1e-9);
    PCL_CHECK(OCT_D(pcl_hz_to_oct(HZ_OF(440.0), HZ_OF(0.0))) > 7.99999);
    PCL_CHECK_NEAR(OCT_D(pcl_hz_to_oct(HZ_OF(0.0), HZ_OF(0.0))), 0.0, 0.0);
    /* Out of range ratios clamp. */
    PCL_CHECK_NEAR(OCT_D(pcl_hz_to_oct(HZ_OF(20000.0), HZ_OF(1.0))), 8.0, 1e-5);
    PCL_CHECK_NEAR(OCT_D(pcl_hz_to_oct(HZ_OF(1.0), HZ_OF(20000.0))), -8.0, 1e-5);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    PCL_CHECK_NEAR(pcl_hz_to_oct(NAN, 440.0f), -8.0, 1e-9);
    PCL_CHECK(pcl_hz_to_oct(440.0f, NAN) == 0.0f || pcl_hz_to_oct(440.0f, NAN) > 7.0f);
    PCL_CHECK_NEAR(pcl_hz_to_oct(INFINITY, 440.0f), 8.0, 1e-5);
#else
    PCL_CHECK_EQ_INT(pcl_hz_to_oct(UINT32_MAX, 1u), INT32_MAX); /* 32 octaves clamp */
    PCL_CHECK_EQ_INT(pcl_hz_to_oct(1u, UINT32_MAX), INT32_MIN);
#endif
}

static void test_pitch_round_trip(void)
{
    double f;
    double g;

    for (f = 20.0; f < 15000.0; f *= 1.23)
        for (g = 0.5 * f; g < 1.9 * f && g < 19000.0; g += 0.37 * f)
        {
            const pcl_hz_t fq = HZ_OF(f);
            const pcl_hz_t gq = HZ_OF(g);
            const double   back = HZ_D(pcl_hz_scale(fq, pcl_hz_to_oct(gq, fq)));
            PCL_CHECK_NEAR(back, HZ_D(gq), HZ_D(gq) * 5.0e-5 + HZ_LSB);
        }
}

static void test_midi_to_hz(void)
{
    unsigned n;

    for (n = 0; n < 128; n++)
    {
        const double ref = 440.0 * pow(2.0, ((double)n - 69.0) / 12.0);
        PCL_CHECK_NEAR(HZ_D(pcl_midi_to_hz((uint8_t)n)), ref, ref * 1.0e-5);
    }
    PCL_CHECK_NEAR(HZ_D(pcl_midi_to_hz(69)), 440.0, 0.0); /* exactly A4 */
    PCL_CHECK_NEAR(HZ_D(pcl_midi_to_hz(60)), 261.6255653005986, 261.63e-5);
    PCL_CHECK_NEAR(HZ_D(pcl_midi_to_hz(127)), 12543.853951415975, 0.15);
    /* Octaves are exact in the exponent, so n + 12 is twice n up to the output's own rounding. */
    for (n = 0; n + 12 < 128; n++)
        PCL_CHECK_NEAR(HZ_D(pcl_midi_to_hz((uint8_t)(n + 12))),
            2.0 * HZ_D(pcl_midi_to_hz((uint8_t)n)),
            2.0 * HZ_LSB + 2.0e-6 * HZ_D(pcl_midi_to_hz((uint8_t)n)));
    for (n = 128; n < 256; n++)
        PCL_CHECK_NEAR(HZ_D(pcl_midi_to_hz((uint8_t)n)), HZ_D(pcl_midi_to_hz(127)), 0.0);
    for (n = 1; n < 128; n++)
        PCL_CHECK(pcl_midi_to_hz((uint8_t)n) > pcl_midi_to_hz((uint8_t)(n - 1)));
}

static void test_cents_to_oct(void)
{
    int32_t c;

    for (c = -9599; c <= 9599; c++)
        PCL_CHECK_NEAR(OCT_D(pcl_cents_to_oct(c)), (double)c / 1200.0, IS_FLOAT ? 5e-7 : 5e-9);
    PCL_CHECK_NEAR(OCT_D(pcl_cents_to_oct(0)), 0.0, 0.0);
    PCL_CHECK_NEAR(OCT_D(pcl_cents_to_oct(1200)), 1.0, 0.0);
    PCL_CHECK_NEAR(OCT_D(pcl_cents_to_oct(-1200)), -1.0, 0.0);
    PCL_CHECK_NEAR(OCT_D(pcl_cents_to_oct(100)), 1.0 / 12.0, 1e-8);
    PCL_CHECK(pcl_cents_to_oct(100000) == pcl_cents_to_oct(9599)); /* clamps */
    PCL_CHECK(pcl_cents_to_oct(-100000) == pcl_cents_to_oct(-9599));
    PCL_CHECK(pcl_cents_to_oct(INT32_MAX) == pcl_cents_to_oct(9599));
    PCL_CHECK(pcl_cents_to_oct(INT32_MIN) == pcl_cents_to_oct(-9599));
    PCL_CHECK(OCT_D(pcl_cents_to_oct(9599)) < 8.0);
}

/* ---- Level ---- */

static void test_db_to_gain_accuracy(void)
{
    double d;

    for (d = -120.0; d <= 6.02; d += 0.0137)
    {
        const double ref = pow(10.0, DB_D(DB_OF(d)) / 20.0);
        const double got = coef_d(pcl_db_to_gain(DB_OF(d)));
        if (ref >= 2.0)
            continue; /* the saturation test covers it */
        PCL_CHECK_NEAR(got, ref, ref * 2.0e-5 + (IS_FLOAT ? 1e-12 : 2.0 / 1073741824.0));
    }
}

static void test_db_to_gain_landmarks_and_clamps(void)
{
    PCL_CHECK_NEAR(coef_d(pcl_db_to_gain(DB_OF(0.0))), 1.0, 0.0); /* 0 dB is exactly unity */
    PCL_CHECK_NEAR(coef_d(pcl_db_to_gain(DB_OF(-6.020599913))), 0.5, 1e-5);
    PCL_CHECK_NEAR(coef_d(pcl_db_to_gain(DB_OF(-20.0))), 0.1, 2e-6);
    PCL_CHECK_NEAR(coef_d(pcl_db_to_gain(DB_OF(6.0))), 1.9952623149688795, 4e-5);
    PCL_CHECK_NEAR(coef_d(pcl_db_to_gain(PCL_DB_MIN)), 1.0e-6, 3e-9);
    PCL_CHECK(pcl_db_to_gain(PCL_DB_MIN - DB_OF(0.001)) == COEF_OF(0.0)); /* below the floor: 0 */
    PCL_CHECK(pcl_db_to_gain(DB_OF(-127.0)) == COEF_OF(0.0));
    PCL_CHECK(PCL_DB_MAX == DB_OF(6.0));
    /* 6.0206 dB is a gain of 2.0 and above it saturates, the same in every build. */
    PCL_CHECK_NEAR(coef_d(pcl_db_to_gain(DB_OF(6.0206))), 2.0, 1e-4);
    PCL_CHECK_NEAR(coef_d(pcl_db_to_gain(DB_OF(6.1))), 2.0, 2e-7);
    PCL_CHECK_NEAR(coef_d(pcl_db_to_gain(DB_OF(100.0))), 2.0, 2e-7);
    PCL_CHECK(coef_d(pcl_db_to_gain(DB_OF(100.0))) < 2.0);
    PCL_CHECK(pcl_db_to_gain(DB_OF(100.0)) == pcl_db_to_gain(DB_OF(6.1)));
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    PCL_CHECK(pcl_db_to_gain(NAN) == 0.0f);
    PCL_CHECK(pcl_db_to_gain(-INFINITY) == 0.0f);
    PCL_CHECK(coef_d(pcl_db_to_gain(INFINITY)) < 2.0);
#else
    PCL_CHECK_EQ_INT(pcl_db_to_gain(0), 1 << 30);
    PCL_CHECK_EQ_INT(pcl_db_to_gain(INT32_MAX), INT32_MAX);
    PCL_CHECK_EQ_INT(pcl_db_to_gain(INT32_MIN), 0);
#endif
}

/* Around 6.0206 dB (a gain of exactly 2.0) the result must climb to the format's maximum and stay.
 */
static void test_db_to_gain_saturation_boundary(void)
{
    const pcl_db_t mid = DB_OF(6.020599913);
    pcl_coef_t     prev = 0;
    int            i;

    for (i = -40; i <= 40; i++)
    {
#if PCL_SAMPLE_TYPE == PCL_FLOAT
        const pcl_db_t d = mid + (float)i * 1.0e-6f;
#else
        const pcl_db_t d = mid + i;
#endif
        const pcl_coef_t g = pcl_db_to_gain(d);
        PCL_CHECK(g >= prev);
        PCL_CHECK(coef_d(g) < 2.0);
        PCL_CHECK(coef_d(g) > 1.9999);
        prev = g;
    }
}

static void test_db_to_gain_monotone(void)
{
    double d;
    double prev = -1.0;

    for (d = -125.0; d < 7.0; d += 0.0021)
    {
        const double g = coef_d(pcl_db_to_gain(DB_OF(d)));
        PCL_CHECK(g >= prev);
        prev = g;
    }
}

static void test_gain_to_db_accuracy(void)
{
    double g;

    for (g = 1.2e-6; g < 1.99; g *= 1.0123)
    {
        const pcl_coef_t gq = COEF_OF(g);
        const double     ref = 20.0 * log10(coef_d(gq));
        if (ref < -120.0)
            continue;
        PCL_CHECK_NEAR(DB_D(pcl_gain_to_db(gq)), ref, 1.5e-4);
    }
}

static void test_gain_to_db_edges(void)
{
    PCL_CHECK_NEAR(DB_D(pcl_gain_to_db(COEF_OF(1.0))), 0.0, 1e-4);
    PCL_CHECK_NEAR(DB_D(pcl_gain_to_db(COEF_OF(0.5))), -6.0205999, 1.5e-4);
    PCL_CHECK_NEAR(DB_D(pcl_gain_to_db(COEF_OF(1.9))), 5.5750, 2e-4);
    PCL_CHECK(pcl_gain_to_db(COEF_OF(0.0)) == PCL_DB_MIN);
    PCL_CHECK(pcl_gain_to_db(COEF_OF(-0.5)) == PCL_DB_MIN);
    PCL_CHECK(pcl_gain_to_db(COEF_OF(1.0e-7)) == PCL_DB_MIN); /* below -120 dB */
    PCL_CHECK(pcl_gain_to_db(COEF_OF(1.0e-6)) >= PCL_DB_MIN);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    PCL_CHECK(pcl_gain_to_db(NAN) == PCL_DB_MIN);
    PCL_CHECK(pcl_gain_to_db(-INFINITY) == PCL_DB_MIN);
    PCL_CHECK_NEAR(pcl_gain_to_db(INFINITY), 20.0 * 30.0, 1e-2); /* clamps at 1e30: no overflow */
    PCL_CHECK_NEAR(pcl_gain_to_db(10.0f), 20.0, 1e-3);           /* float is not limited to 2.0 */
#else
    PCL_CHECK_EQ_INT(pcl_gain_to_db(1), PCL_DB_MIN);
    PCL_CHECK_EQ_INT(pcl_gain_to_db(INT32_MIN), PCL_DB_MIN);
    PCL_CHECK_NEAR(DB_D(pcl_gain_to_db(INT32_MAX)), 6.0206, 1e-3);
#endif
}

static void test_db_round_trip(void)
{
    double d;

    for (d = -100.0; d <= 5.9; d += 0.173)
    {
        const double back = DB_D(pcl_gain_to_db(pcl_db_to_gain(DB_OF(d))));
        /* the gain's own quantisation (Q2.30) dominates at low levels */
        const double quant
            = IS_FLOAT ? 0.0 : 8.7 * 0.5 / (coef_d(pcl_db_to_gain(DB_OF(d))) * 1073741824.0);
        PCL_CHECK_NEAR(back, d, 2.0e-4 + quant);
    }
}

/* ---- Shapes ---- */

static void test_sin_cos(void)
{
    uint32_t i;

    /* Multiplying by the golden-ratio constant spreads the phases evenly over all four quadrants.
     */
    for (i = 0; i < 200000u; i++)
    {
        const uint32_t p = i * 2654435761u;
        const double   a = 2.0 * pi * (double)p / 4294967296.0;
        PCL_CHECK_NEAR(pcl_test_to_double(pcl_sin(p)), sin(a), SIN_TOL);
        PCL_CHECK_NEAR(pcl_test_to_double(pcl_cos(p)), cos(a), SIN_TOL);
        PCL_CHECK(pcl_sin(p + 0x80000000u) == (pcl_sample_t)-pcl_sin(p)); /* half cycle: negated */
        PCL_CHECK(pcl_cos(p) == pcl_sin(p + 0x40000000u));
    }
    PCL_CHECK_NEAR(pcl_test_to_double(pcl_sin(0)), 0.0, 0.0);
    PCL_CHECK_NEAR(pcl_test_to_double(pcl_cos(0)), 1.0, SIN_TOL);
    PCL_CHECK_NEAR(pcl_test_to_double(pcl_sin(0x40000000u)), 1.0, SIN_TOL);
    PCL_CHECK_NEAR(pcl_test_to_double(pcl_sin(0xC0000000u)), -1.0, SIN_TOL);
    PCL_CHECK_NEAR(pcl_test_to_double(pcl_cos(0x40000000u)), 0.0, SIN_TOL);
}

static void tan_sweep(uint32_t fs)
{
    double r;

    for (r = 1.0e-4; r <= 0.45; r *= 1.0031)
    {
        const double f = r * (double)fs;
        double       ref;
        double       rel;
        pcl_hz_t     fq;

        if (f > 65000.0 || f < 0.01)
            continue; /* outside pcl_hz_t */
        fq = HZ_OF(f);
        ref = tan(pi * HZ_D(fq) / (double)fs);
        /* x = pi r below 0.5 is the power series, above it the sine table */
        rel = (pi * r < 0.5) ? TAN_REL : TAN_REL_HIGH;
        PCL_CHECK_NEAR(WIDE_D(pcl_tan_hz(fq, fs)), ref, TAN_ABS + ref * rel);
    }
}

static void test_tan_hz_accuracy(void)
{
    tan_sweep(8000u);
    tan_sweep(44100u);
    tan_sweep(48000u);
    tan_sweep(96000u);
    tan_sweep(768000u);
}

static void test_tan_hz_landmarks_and_clamps(void)
{
    const double t45 = tan(0.45 * pi);

    PCL_CHECK_NEAR(WIDE_D(pcl_tan_hz(HZ_OF(12000.0), 48000u)), 1.0, 1e-4); /* fs/4: tan(pi/4) */
    PCL_CHECK_NEAR(WIDE_D(pcl_tan_hz(HZ_OF(0.0), 48000u)), 0.0, 0.0);
    PCL_CHECK_NEAR(WIDE_D(pcl_tan_hz(HZ_OF(1000.0), 0u)), 0.0, 0.0); /* invalid rate: 0 */
    /* Above 0.45 fs the argument clamps, and the value is the clamp's. */
    PCL_CHECK_NEAR(WIDE_D(pcl_tan_hz(HZ_OF(21600.0), 48000u)), t45, t45 * 1e-3);
    PCL_CHECK_NEAR(WIDE_D(pcl_tan_hz(HZ_OF(21599.0), 48000u)), t45, t45 * 1e-3);
    PCL_CHECK(pcl_tan_hz(HZ_OF(22000.0), 48000u) == pcl_tan_hz(HZ_OF(21600.0), 48000u));
    PCL_CHECK(pcl_tan_hz(HZ_OF(22000.0), 48000u) == pcl_tan_hz(HZ_OF(40000.0), 48000u));
    PCL_CHECK_NEAR(WIDE_D(pcl_tan_hz(HZ_OF(22000.0), 48000u)), t45, t45 * 1e-4);
    PCL_CHECK(pcl_tan_hz(HZ_OF(21000.0), 48000u) < pcl_tan_hz(HZ_OF(21500.0), 48000u));
    PCL_CHECK(WIDE_D(pcl_tan_hz(HZ_OF(24000.0), 48000u)) < 8.0); /* fits the format */
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    PCL_CHECK_NEAR(pcl_tan_hz(NAN, 48000u), 0.0, 0.0);
    PCL_CHECK_NEAR(pcl_tan_hz(-5.0f, 48000u), 0.0, 0.0);
    PCL_CHECK(pcl_tan_hz(INFINITY, 48000u) == pcl_tan_hz(24000.0f, 48000u));
#else
    PCL_CHECK(pcl_tan_hz(UINT32_MAX, 48000u) == pcl_tan_hz(HZ_OF(24000.0), 48000u));
    PCL_CHECK(pcl_tan_hz(UINT32_MAX, PCL_SAMPLE_RATE_MIN_HZ) > 0);
#endif
}

/* pcl_tan_inc is pcl_tan_hz with f / fs already formed as a phase increment. */
static void test_tan_inc(void)
{
    static const uint32_t rates[] = { 8000u, 44100u, 48000u, 96000u };
    size_t                k;
    double                r;

    for (k = 0; k < sizeof rates / sizeof rates[0]; k++)
        for (r = 1.0e-4; r <= 0.46; r *= 1.07)
        {
            const double f = r * (double)rates[k];
            pcl_hz_t     fq;
            uint32_t     inc;
            if (f > 65000.0 || f < 0.01)
                continue;
            fq = HZ_OF(f);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
            inc = (uint32_t)(HZ_D(fq) / (double)rates[k] * 4294967296.0);
            PCL_CHECK_NEAR(pcl_tan_inc(inc), pcl_tan_hz(fq, rates[k]),
                fabs((double)pcl_tan_hz(fq, rates[k])) * 2.0e-6 + 1.0e-9);
#else
            inc = (uint32_t)(((uint64_t)fq << 16) / rates[k]);
            PCL_CHECK(
                pcl_tan_inc(inc) == pcl_tan_hz(fq, rates[k])); /* the same code, bit for bit */
#endif
        }
    PCL_CHECK_NEAR(WIDE_D(pcl_tan_inc(0u)), 0.0, 0.0);
    PCL_CHECK_NEAR(WIDE_D(pcl_tan_inc(0x40000000u)), 1.0, 1.0e-4); /* r = 0.25: tan(pi/4) */
    /* above 0.45 of the sample rate it clamps, and never reaches the pole at 0.5 */
    PCL_CHECK(pcl_tan_inc(0x80000000u) == pcl_tan_inc(0xF0000000u));
    PCL_CHECK(pcl_tan_inc(0x7FFFFFFFu) == pcl_tan_inc(0xFFFFFFFFu));
    PCL_CHECK(WIDE_D(pcl_tan_inc(0xFFFFFFFFu)) < 8.0);
    PCL_CHECK_NEAR(WIDE_D(pcl_tan_inc(0x80000000u)), tan(0.45 * pi), tan(0.45 * pi) * 1.0e-3);
}

static void test_tan_hz_monotone(void)
{
    double   r;
    double   prev = -1.0;
    uint32_t fs;

    for (fs = 8000u; fs <= 96000u; fs += 88000u)
    {
        prev = -1.0;
        for (r = 1.0e-4; r < 0.46; r += 7.0e-6)
        {
            const double v = WIDE_D(pcl_tan_hz(HZ_OF(r * (double)fs), fs));
            PCL_CHECK(v >= prev);
            prev = v;
        }
    }
}

/*
 * Cutoffs stepped finely across the power-series / sin-cos switch at x = pi f / fs = 0.5 (in fixed
 * point every Q16.16 step within +-3 Hz). The sine-table branch can sit a hair below the series, so
 * a build with a coarse table may step backwards, once, by no more than that table's error: 3e-5
 * relative for Q15's. Q31 and float must not step backwards at all.
 */
static void test_tan_hz_junction(void)
{
    static const uint32_t rates[] = { 8000u, 44100u, 48000u, 96000u };
#if IS_FLOAT
    const int steps = 12000;
#else
    const int steps = 6 * 65536;
#endif
    size_t k;

    for (k = 0; k < sizeof rates / sizeof rates[0]; k++)
    {
        const double junction = 0.5 / pi * (double)rates[k];
        double       prev = 0.0;
        int          backward = 0;
        int          i;

        for (i = 0; i <= steps; i++)
        {
            const double v = WIDE_D(
                pcl_tan_hz(HZ_OF(junction - 3.0 + 6.0 * (double)i / (double)steps), rates[k]));
            if (i > 0 && v < prev)
            {
                backward++;
                PCL_CHECK((prev - v) / prev < JUNCTION_BACK_TOL);
            }
            prev = v;
        }
        PCL_CHECK(backward <= JUNCTION_BACK_MAX);
    }
}

static void test_tanh_accuracy(void)
{
    double x;

    for (x = -7.0; x <= 7.0; x += 0.00031)
    {
        const pcl_wide_t xq = WIDE_OF(x);
        PCL_CHECK_NEAR(pcl_test_to_double(pcl_tanh(xq)), tanh(WIDE_D(xq)), TANH_TOL);
    }
}

static void test_tanh_shape(void)
{
    int          i;
    double       x;
    double       prev = -2.0;
    pcl_sample_t end = pcl_tanh(WIDE_OF(7.9));

    PCL_CHECK(pcl_tanh(WIDE_OF(0.0)) == 0);
    for (x = -7.5; x <= 7.5; x += 0.00013)
    {
        const pcl_wide_t xq = WIDE_OF(x);
        const double     y = pcl_test_to_double(pcl_tanh(xq));
        PCL_CHECK(y >= prev); /* monotone */
        prev = y;
        PCL_CHECK(pcl_tanh((pcl_wide_t)-xq) == (pcl_sample_t)-pcl_tanh(xq)); /* exactly odd */
    }
    /* At every table joint the output is the table entry (fixed point), so joints are continuous.
     */
    for (i = 0; i <= 383; i++)
    {
        const pcl_wide_t xq = WIDE_OF((double)i / 64.0);
        PCL_CHECK_NEAR(pcl_test_to_double(pcl_tanh(xq)), tanh((double)i / 64.0), TANH_TOL);
    }
    /* Holds at the last entry beyond 6 and never reaches 1.0. */
    PCL_CHECK(pcl_tanh(WIDE_OF(6.0)) == end);
    PCL_CHECK(pcl_tanh(WIDE_OF(7.99)) == end);
    PCL_CHECK(pcl_tanh(WIDE_OF(-7.99)) == (pcl_sample_t)-end);
    PCL_CHECK_NEAR(pcl_test_to_double(end), tanh(6.0), TANH_TOL);
    PCL_CHECK(pcl_test_to_double(end) < 1.0);
    /* Small signals pass through with unity slope. */
    PCL_CHECK_NEAR(pcl_test_to_double(pcl_tanh(WIDE_OF(0.001))), 0.001, TANH_TOL);
    PCL_CHECK_NEAR(pcl_test_to_double(pcl_tanh(WIDE_OF(-0.002))), -0.002, TANH_TOL);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    PCL_CHECK(pcl_tanh(NAN) == 0.0f);
    PCL_CHECK(pcl_tanh(INFINITY) == end);
    PCL_CHECK(pcl_tanh(-INFINITY) == -end);
    PCL_CHECK(pcl_tanh(1.0e30f) == end);
#else
    PCL_CHECK(pcl_tanh(INT32_MAX) == end);
    PCL_CHECK(pcl_tanh(INT32_MIN) == (pcl_sample_t)-end); /* |INT32_MIN| does not overflow */
#endif
}

/* ---- Sample <-> wide ---- */

static void test_sample_wide_conversion(void)
{
    double v;

    for (v = -1.0; v <= 0.9999; v += 0.00137)
    {
        const pcl_sample_t s = pcl_test_from_double(v);
        const pcl_wide_t   w = pcl_sample_to_wide(s);
        PCL_CHECK_NEAR(WIDE_D(w), pcl_test_to_double(s), 4.0e-9 + (IS_FLOAT ? 0.0 : 0.0));
        /* and back: Q15 is exact; Q31 loses the 3 bits Q4.28 does not have */
#if PCL_SAMPLE_TYPE == PCL_Q31
        PCL_CHECK(abs((int)((int64_t)pcl_wide_to_sample(w) - (int64_t)s)) <= 4);
#else
        PCL_CHECK(pcl_wide_to_sample(w) == s);
#endif
    }
    /* Values beyond full scale saturate. */
    PCL_CHECK_NEAR(pcl_test_to_double(pcl_wide_to_sample(WIDE_OF(0.5))), 0.5, PCL_TEST_SAMPLE_TOL);
    PCL_CHECK_NEAR(
        pcl_test_to_double(pcl_wide_to_sample(WIDE_OF(-0.5))), -0.5, PCL_TEST_SAMPLE_TOL);
#if PCL_SAMPLE_TYPE != PCL_FLOAT
    PCL_CHECK(
        pcl_wide_to_sample(WIDE_OF(3.0)) == (PCL_SAMPLE_TYPE == PCL_Q15 ? INT16_MAX : INT32_MAX));
    PCL_CHECK(
        pcl_wide_to_sample(WIDE_OF(-3.0)) == (PCL_SAMPLE_TYPE == PCL_Q15 ? INT16_MIN : INT32_MIN));
    PCL_CHECK(
        pcl_wide_to_sample(INT32_MAX) == (PCL_SAMPLE_TYPE == PCL_Q15 ? INT16_MAX : INT32_MAX));
    PCL_CHECK(
        pcl_wide_to_sample(INT32_MIN) == (PCL_SAMPLE_TYPE == PCL_Q15 ? INT16_MIN : INT32_MIN));
    /* Just below full scale (1.0 - 2^-28) rounds up to 1.0 and must saturate, not wrap. */
    PCL_CHECK(pcl_wide_to_sample(1 << 28) == (PCL_SAMPLE_TYPE == PCL_Q15 ? INT16_MAX : INT32_MAX));
    PCL_CHECK(
        pcl_wide_to_sample(-(1 << 28)) == (PCL_SAMPLE_TYPE == PCL_Q15 ? INT16_MIN : INT32_MIN));
    PCL_CHECK(pcl_wide_to_sample((1 << 28) - 1) > 0);
    PCL_CHECK(pcl_wide_to_sample(-(1 << 28)) < 0);
    PCL_CHECK(pcl_sample_to_wide(PCL_SAMPLE_TYPE == PCL_Q15 ? INT16_MAX : INT32_MAX) > 0);
    PCL_CHECK(pcl_sample_to_wide(PCL_SAMPLE_TYPE == PCL_Q15 ? INT16_MIN : INT32_MIN) == -(1 << 28));
#if PCL_SAMPLE_TYPE == PCL_Q15
    /* Q15 rounds to nearest, ties up: 1000.5 LSB -> 1001, just below -> 1000. */
    PCL_CHECK_EQ_INT(pcl_wide_to_sample(1000 * 8192 + 4095), 1000);
    PCL_CHECK_EQ_INT(pcl_wide_to_sample(1000 * 8192 + 4096), 1001);
    PCL_CHECK_EQ_INT(pcl_wide_to_sample(-1000 * 8192 - 4096), -1000);
    PCL_CHECK_EQ_INT(pcl_wide_to_sample(-1000 * 8192 - 4097), -1001);
#endif
#else
    PCL_CHECK(pcl_wide_to_sample(3.0f) == 3.0f); /* float does not clip */
#endif
}

int main(void)
{
    PCL_RUN_TEST(test_hz_scale_accuracy);
    PCL_RUN_TEST(test_hz_scale_landmarks_and_clamps);
    PCL_RUN_TEST(test_hz_scale_monotone);
    PCL_RUN_TEST(test_hz_to_oct_accuracy);
    PCL_RUN_TEST(test_hz_to_oct_edges);
    PCL_RUN_TEST(test_pitch_round_trip);
    PCL_RUN_TEST(test_midi_to_hz);
    PCL_RUN_TEST(test_cents_to_oct);
    PCL_RUN_TEST(test_db_to_gain_accuracy);
    PCL_RUN_TEST(test_db_to_gain_landmarks_and_clamps);
    PCL_RUN_TEST(test_db_to_gain_saturation_boundary);
    PCL_RUN_TEST(test_db_to_gain_monotone);
    PCL_RUN_TEST(test_gain_to_db_accuracy);
    PCL_RUN_TEST(test_gain_to_db_edges);
    PCL_RUN_TEST(test_db_round_trip);
    PCL_RUN_TEST(test_sin_cos);
    PCL_RUN_TEST(test_tan_hz_accuracy);
    PCL_RUN_TEST(test_tan_hz_landmarks_and_clamps);
    PCL_RUN_TEST(test_tan_inc);
    PCL_RUN_TEST(test_tan_hz_monotone);
    PCL_RUN_TEST(test_tan_hz_junction);
    PCL_RUN_TEST(test_tanh_accuracy);
    PCL_RUN_TEST(test_tanh_shape);
    PCL_RUN_TEST(test_sample_wide_conversion);
    return pcl_test_summary("math");
}
