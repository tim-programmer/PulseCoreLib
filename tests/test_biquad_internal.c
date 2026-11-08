/*
 * White-box tests of the biquad's private helpers: the test includes the source so it can reach the
 * static functions (the exact 128-bit multiply, saturation, conversions, parameter mappings, step).
 */
#include "../src/filter/pcl_biquad.c"

#include <stdlib.h>
#include <string.h>

#include "pcl_test.h"

static const double pi = 3.14159265358979323846;

/* Tolerances of the design check: absolute for a1 a2, relative for b. The cutoff passes through a
   log2/exp2 round trip (0.03 cents); Q15's tangent above x = 0.5 comes from a 15-bit sine table
   (1e-4 relative); float amplifies its ~1e-6 tangent error through a1. */
#if PCL_SAMPLE_TYPE == PCL_FLOAT
#define IS_FLOAT_TOL 3e-5
#define B_REL_TOL 2e-5
#elif PCL_SAMPLE_TYPE == PCL_Q15
#define IS_FLOAT_TOL 6e-5
#define B_REL_TOL 4e-4
#else
#define IS_FLOAT_TOL 4e-7
#define B_REL_TOL 4e-5
#endif

#if PCL_SAMPLE_TYPE != PCL_FLOAT

static uint64_t g_state = 0x9E3779B97F4A7C15u;

static uint64_t next64(void)
{
    g_state ^= g_state << 13;
    g_state ^= g_state >> 7;
    g_state ^= g_state << 17;
    return g_state;
}

#ifdef __SIZEOF_INT128__
__extension__ typedef __int128          i128_t;
__extension__ typedef unsigned __int128 u128_t;

/* The helper is exact: compare with the compiler's 128-bit arithmetic. */
static void test_mulshr_matches_int128(void)
{
    int i;

    for (i = 0; i < 20000; i++)
    {
        /* operands of the sizes the design uses (up to ~2^40 and ~2^62) and some extremes */
        const int      bits_a = 1 + (int)(next64() % 62u);
        const int      bits_b = 1 + (int)(next64() % 62u);
        int64_t        a = (int64_t)(next64() >> (64 - bits_a));
        int64_t        b = (int64_t)(next64() >> (64 - bits_b));
        const unsigned s = 1u + (unsigned)(next64() % 100u);
        i128_t         p;
        i128_t         want;

        if (next64() & 1u)
            a = -a;
        if (next64() & 1u)
            b = -b;
        p = (i128_t)a * (i128_t)b;
        /* round to nearest, ties away from zero (the helper rounds the magnitude) */
        {
            const int neg = p < 0;
            u128_t    m = neg ? (u128_t)(-p) : (u128_t)p;
            u128_t    half = (u128_t)1 << (s - 1u);

            m = (m + half) >> s;
            want = neg ? -(i128_t)m : (i128_t)m;
        }
        if (want > (i128_t)INT64_MAX || want < -(i128_t)INT64_MAX)
            continue; /* the helper requires a result that fits */
        PCL_CHECK(bq_mulshr(a, b, s) == (int64_t)want);
    }
    /* operands as wide as an int64 allows, where the middle partial products overflow 64 bits */
    for (i = 0; i < 20000; i++)
    {
        int64_t        a = (int64_t)(next64() >> 1) | ((int64_t)1 << 62);
        int64_t        b = (int64_t)(next64() >> 1) | ((int64_t)1 << 62);
        const unsigned s = 64u + (unsigned)(next64() % 40u);
        u128_t         m;

        if (next64() & 1u)
            a = -a;
        if (next64() & 1u)
            b = -b;
        {
            const i128_t p = (i128_t)a * (i128_t)b;
            const int    neg = p < 0;

            m = neg ? (u128_t)(-p) : (u128_t)p;
            m = (m + ((u128_t)1 << (s - 1u))) >> s;
            PCL_CHECK(bq_mulshr(a, b, s) == (neg ? -(int64_t)m : (int64_t)m));
        }
    }
    PCL_CHECK_EQ_INT(bq_mulshr(0, 12345, 10), 0);
    PCL_CHECK_EQ_INT(bq_mulshr(12345, 0, 10), 0);
    PCL_CHECK_EQ_INT(bq_mulshr(1 << 20, 1 << 20, 40), 1);
    PCL_CHECK_EQ_INT(bq_mulshr(-(1 << 20), 1 << 20, 40), -1);
}
#endif

static void test_saturation_and_conversions(void)
{
    const int64_t one = (int64_t)1 << PCL_Q28_SHIFT;

    PCL_CHECK_EQ_INT(bq_sat32(0), 0);
    PCL_CHECK_EQ_INT(bq_sat32(-5), -5);
    PCL_CHECK_EQ_INT(bq_sat32(INT32_MAX), INT32_MAX);
    PCL_CHECK_EQ_INT(bq_sat32(INT32_MIN), INT32_MIN);
    PCL_CHECK_EQ_INT(bq_sat32((int64_t)INT32_MAX + 1), INT32_MAX);
    PCL_CHECK_EQ_INT(bq_sat32((int64_t)INT32_MIN - 1), INT32_MIN);
    PCL_CHECK_EQ_INT(bq_sat32((int64_t)1 << 50), INT32_MAX);
    PCL_CHECK_EQ_INT(bq_sat32(-((int64_t)1 << 50)), INT32_MIN);

    /* history (Q4.28) -> sample: full scale at 1.0, saturating beyond */
    PCL_CHECK_EQ_INT(bq_out(0), 0);
    PCL_CHECK_NEAR(pcl_test_to_double(bq_out((int32_t)(one / 2))), 0.5, PCL_TEST_SAMPLE_TOL);
    PCL_CHECK_NEAR(pcl_test_to_double(bq_out((int32_t)(-one / 2))), -0.5, PCL_TEST_SAMPLE_TOL);
    PCL_CHECK(bq_out((int32_t)one) > 0 && bq_out(INT32_MAX) == bq_out((int32_t)one - 1));
    PCL_CHECK(bq_out((int32_t)-one) < 0 && bq_out(INT32_MIN) == bq_out((int32_t)-one));
    /* sample -> Q4.28 -> sample */
    {
        const pcl_sample_t probes[]
            = { 0, 1, -1, 1000, -1000, 12345, -23456, INT16_MAX, INT16_MIN };
        size_t i;

        for (i = 0; i < sizeof probes / sizeof probes[0]; i++)
            PCL_CHECK(abs((int)bq_out(bq_in(probes[i])) - (int)probes[i]) <= 1);
    }
}

static void test_step_does_not_overflow(void)
{
    /* a coefficient swinging from the top of its range to the bottom in one period */
    PCL_CHECK_EQ_INT(bq_step(INT32_MIN, INT32_MAX),
        (int32_t)(((int64_t)INT32_MIN - INT32_MAX) / PCL_CONTROL_DIV));
    PCL_CHECK_EQ_INT(bq_step(INT32_MAX, INT32_MIN),
        (int32_t)(((int64_t)INT32_MAX - INT32_MIN) / PCL_CONTROL_DIV));
    PCL_CHECK_EQ_INT(bq_step(1600, 0), 1600 / PCL_CONTROL_DIV);
    PCL_CHECK_EQ_INT(bq_step(5, 5), 0);
}

#else /* float */

static void test_clamp(void)
{
    PCL_CHECK(bq_clamp(0.5f) == 0.5f);
    PCL_CHECK(bq_clamp(9.0f) == 8.0f);
    PCL_CHECK(bq_clamp(-9.0f) == -8.0f);
    PCL_CHECK(bq_clamp(NAN) == -8.0f); /* a NaN is clamped too */
    PCL_CHECK(bq_clamp(INFINITY) == 8.0f && bq_clamp(-INFINITY) == -8.0f);
    PCL_CHECK_NEAR(bq_step(1.6f, 0.0f), 0.1, 1e-7);
}

#endif

static void test_parameter_mappings(void)
{
    /* cutoff -> octaves above 1 kHz / 8; Q -> log2(Q) / 8; dB -> dB / 18 */
    PCL_CHECK_NEAR(pcl_test_coef_to_double(bq_u_from_hz(PCL_HZ(1000.0))), 0.0, 1e-4);
    PCL_CHECK_NEAR(pcl_test_coef_to_double(bq_u_from_hz(PCL_HZ(2000.0))), 1.0 / 8.0, 1e-4);
    PCL_CHECK_NEAR(pcl_test_coef_to_double(bq_u_from_hz(PCL_HZ(250.0))), -2.0 / 8.0, 1e-4);
    PCL_CHECK_NEAR(pcl_test_coef_to_double(bq_u_from_q(PCL_QUALITY(1.0))), 0.0, 1e-4);
    PCL_CHECK_NEAR(pcl_test_coef_to_double(bq_u_from_q(PCL_QUALITY(8.0))), 3.0 / 8.0, 1e-4);
    PCL_CHECK_NEAR(pcl_test_coef_to_double(bq_u_from_q(PCL_QUALITY(0.5))), -1.0 / 8.0, 1e-4);
    PCL_CHECK_NEAR(pcl_test_coef_to_double(bq_u_from_db(PCL_DB(0.0))), 0.0, 1e-9);
    PCL_CHECK_NEAR(pcl_test_coef_to_double(bq_u_from_db(PCL_DB(18.0))), 1.0, 1e-7);
    PCL_CHECK_NEAR(pcl_test_coef_to_double(bq_u_from_db(PCL_DB(-9.0))), -0.5, 1e-7);
}

static void test_design_of_the_low_pass(void)
{
    static const double fcs[] = { 20.0, 440.0, 4000.0, 15000.0 };
    size_t              i;

    for (i = 0; i < sizeof fcs / sizeof fcs[0]; i++)
    {
        pcl_biquad_config_t c;
        pcl_biquad_t        f;
        pcl_biquad_c_t      co[5];
        double              v[5], k, n, kk;
        unsigned            j;

        memset(&f, 0, sizeof f);
        c.sample_rate_hz = 48000u;
        c.type = PCL_BIQUAD_LP;
        c.cutoff_hz
            = PCL_HZ(0.0) + (pcl_hz_t)(fcs[i] * (PCL_SAMPLE_TYPE == PCL_FLOAT ? 1.0 : 65536.0));
        c.q = PCL_QUALITY(2.0);
        c.gain_db = PCL_DB(0.0);
        c.mod_depth_oct = PCL_OCT(0.0);
        PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &c), PCL_OK);
        bq_design(&f, 0, co);
        for (j = 0; j < 5; j++)
#if PCL_SAMPLE_TYPE == PCL_FLOAT
            v[j] = (double)co[j];
#else
            v[j] = (double)co[j] / (j < 3 ? 134217728.0 : 1073741824.0);
#endif
        kk = tan(pi * fcs[i] / 48000.0);
        k = 0.5;
        n = 1.0 / (1.0 + k * kk + kk * kk);
        PCL_CHECK_NEAR(v[0], kk * kk * n, B_REL_TOL * (kk * kk * n) + 3e-8);
        PCL_CHECK_NEAR(v[1], 2.0 * kk * kk * n, 2 * B_REL_TOL * (kk * kk * n) + 6e-8);
        PCL_CHECK_NEAR(v[2], kk * kk * n, B_REL_TOL * (kk * kk * n) + 3e-8);
        PCL_CHECK_NEAR(v[3], 2.0 * (kk * kk - 1.0) * n, IS_FLOAT_TOL);
        PCL_CHECK_NEAR(v[4], (1.0 - k * kk + kk * kk) * n, IS_FLOAT_TOL);
    }
}

#if PCL_SAMPLE_TYPE != PCL_FLOAT
/*
 * The reciprocal of D0 is refined by a Newton step to ~54 bits because a1 sits near -2 at low
 * cutoffs. Given the K and 1/Q the module itself computed, the coefficients must be the exact ones
 * (in double) to within the format's resolution.
 */
static void test_design_precision_at_low_cutoff(void)
{
    static const double fcs[] = { 7.0, 20.0, 60.0, 300.0 };
    static const double qs[] = { 0.5, 2.0, 40.0 };
    size_t              i, j;

    for (i = 0; i < sizeof fcs / sizeof fcs[0]; i++)
        for (j = 0; j < sizeof qs / sizeof qs[0]; j++)
        {
            pcl_biquad_config_t c;
            pcl_biquad_t        f;
            pcl_biquad_c_t      co[5];
            double              K, k, n, ex[5];
            unsigned            u;

            c.sample_rate_hz = 48000u;
            c.type = PCL_BIQUAD_LP;
            c.cutoff_hz = (pcl_hz_t)(fcs[i] * 65536.0 + 0.5);
            c.q = PCL_QUALITY(0.0) + (pcl_quality_t)(qs[j] * 65536.0 + 0.5);
            c.gain_db = PCL_DB(0.0);
            c.mod_depth_oct = PCL_OCT(0.0);
            memset(&f, 0, sizeof f);
            PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &c), PCL_OK);
            bq_design(&f, 0, co);
            /* the same K and 1/Q the module used */
            {
                const int64_t  oct = (int64_t)f.cutoff.current * 2;
                const uint32_t inc = pcl_octave_scale_inc(f.inc_1k, bq_sat32(oct), BQ_MAX_INC);

                K = (double)pcl_tan_inc(inc) / 268435456.0;
                k = (double)pcl_octave_scale_inc(
                        (uint32_t)1 << 28, bq_sat32(-(int64_t)f.q.current * 2), UINT32_MAX)
                    / 268435456.0;
            }
            n = 1.0 / (1.0 + k * K + K * K);
            ex[0] = K * K * n;
            ex[1] = 2.0 * K * K * n;
            ex[2] = K * K * n;
            ex[3] = 2.0 * (K * K - 1.0) * n;
            ex[4] = (1.0 - k * K + K * K) * n;
            for (u = 0; u < 3; u++)
                PCL_CHECK_NEAR(
                    (double)co[u] / 134217728.0, ex[u], 1.0 / 134217728.0); /* 1 LSB, Q5.27 */
            for (u = 3; u < 5; u++)
                PCL_CHECK_NEAR(
                    (double)co[u] / 1073741824.0, ex[u], 1.0 / 1073741824.0); /* 1 LSB, Q2.30 */
        }
}
#endif

int main(void)
{
#if PCL_SAMPLE_TYPE != PCL_FLOAT
#ifdef __SIZEOF_INT128__
    PCL_RUN_TEST(test_mulshr_matches_int128);
#endif
    PCL_RUN_TEST(test_saturation_and_conversions);
    PCL_RUN_TEST(test_step_does_not_overflow);
    PCL_RUN_TEST(test_design_precision_at_low_cutoff);
#else
    PCL_RUN_TEST(test_clamp);
#endif
    PCL_RUN_TEST(test_parameter_mappings);
    PCL_RUN_TEST(test_design_of_the_low_pass);
    return pcl_test_summary("biquad internals");
}
