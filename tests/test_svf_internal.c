/*
 * White-box tests of the filter's private helpers: the test includes the source so it can reach the
 * static functions (saturation, conversions, the Newton step). Everything the public tests can only
 * see through the filter's behaviour is checked here directly.
 */
#include "../src/filter/pcl_svf.c"

#include <stdlib.h>
#include <string.h>

#include "pcl_test.h"

static const double pi = 3.14159265358979323846;

#if PCL_SAMPLE_TYPE == PCL_FLOAT
#define DAMP_TOL 3e-7
#else
#define DAMP_TOL 1e-8
#endif

#if PCL_SAMPLE_TYPE != PCL_FLOAT

static void test_saturation(void)
{
    PCL_CHECK_EQ_INT(pcl_wide_sat(0), 0);
    PCL_CHECK_EQ_INT(pcl_wide_sat(123456), 123456);
    PCL_CHECK_EQ_INT(pcl_wide_sat(-123456), -123456);
    PCL_CHECK_EQ_INT(pcl_wide_sat(INT32_MAX), INT32_MAX);
    PCL_CHECK_EQ_INT(pcl_wide_sat(INT32_MIN), INT32_MIN);
    PCL_CHECK_EQ_INT(pcl_wide_sat((int64_t)INT32_MAX + 1), INT32_MAX);
    PCL_CHECK_EQ_INT(pcl_wide_sat((int64_t)INT32_MIN - 1), INT32_MIN);
    PCL_CHECK_EQ_INT(pcl_wide_sat((int64_t)1 << 40), INT32_MAX);
    PCL_CHECK_EQ_INT(pcl_wide_sat(-((int64_t)1 << 40)), INT32_MIN);
    /* the case a wrapping add would get wrong: two positives that sum past +8 */
    PCL_CHECK_EQ_INT(pcl_wide_sat((int64_t)(6 << 28) + (5 << 28)), INT32_MAX);
    PCL_CHECK_EQ_INT(pcl_wide_sat((int64_t)(-6 * (1 << 28)) - (5 * (1 << 28))), INT32_MIN);
}

static void test_conversions(void)
{
    const int64_t one = (int64_t)1 << PCL_Q28_SHIFT;

    /* Q4.28 -> sample: full scale at 1.0, saturating beyond */
    PCL_CHECK_EQ_INT(pcl_wide64_to_sample(0), 0);
    PCL_CHECK_NEAR(pcl_test_to_double(pcl_wide64_to_sample(one / 2)), 0.5, PCL_TEST_SAMPLE_TOL);
    PCL_CHECK_NEAR(pcl_test_to_double(pcl_wide64_to_sample(-one / 2)), -0.5, PCL_TEST_SAMPLE_TOL);
    PCL_CHECK(pcl_wide64_to_sample(one) > 0 && pcl_wide64_to_sample(one * 3) == pcl_wide64_to_sample(one - 1));
    PCL_CHECK(pcl_wide64_to_sample(-one * 3) < 0 && pcl_wide64_to_sample(-one * 3) == pcl_wide64_to_sample(-one));
    PCL_CHECK(pcl_wide64_to_sample((int64_t)1 << 50) == pcl_wide64_to_sample(one - 1));
    PCL_CHECK(pcl_wide64_to_sample(-((int64_t)1 << 50)) == pcl_wide64_to_sample(-one));
    PCL_CHECK(pcl_wide64_to_sample(one - 1) == (PCL_SAMPLE_TYPE == PCL_Q15 ? INT16_MAX : (INT32_MAX - 7)));
    /* sample -> Q4.28 -> sample is the identity for Q15 and loses only the bottom 3 bits for Q31 */
    {
        const pcl_sample_t probes[]
            = { 0, 1, -1, 1000, -1000, 12345, -23456, INT16_MAX, INT16_MIN };
        size_t i;
        for (i = 0; i < sizeof probes / sizeof probes[0]; i++)
        {
            const pcl_sample_t back = pcl_wide64_to_sample(pcl_sample_to_wide_rounded(probes[i]));
            PCL_CHECK(abs((int)back - (int)probes[i]) <= 1);
        }
    }
    PCL_CHECK_EQ_INT(pcl_sample_to_wide_rounded((pcl_sample_t)0), 0);
}

static void test_newton_and_products(void)
{
    const int32_t one30 = 1 << 30;
    const int32_t one24 = 1 << 24;
    pcl_coef_t    a;
    int           i;

    /* 1/d: d = 1 -> 1.0; d = 2 -> 0.5; it is a fixed point of the step */
    PCL_CHECK_NEAR(pcl_test_coef_to_double(pcl_cutoff_newton(one30, one24)), 1.0, 2e-9);
    PCL_CHECK_NEAR(pcl_test_coef_to_double(pcl_cutoff_newton(one30 / 2, 2 * one24)), 0.5, 2e-9);
    /* and it converges quadratically from a neighbouring value */
    a = (pcl_coef_t)((int64_t)one30 * 4 / 10); /* 0.4, true 1/d = 0.5 */
    for (i = 0; i < 6; i++)
        a = pcl_cutoff_newton(a, 2 * one24);
    PCL_CHECK_NEAR(pcl_test_coef_to_double(a), 0.5, 4e-9);
    /* from a far-off start it stays in [0, 1] instead of going negative */
    PCL_CHECK(pcl_cutoff_newton(one30, 40 * one24) >= 0);
    PCL_CHECK(pcl_cutoff_newton(one30, 40 * one24) <= one30);
    PCL_CHECK(pcl_cutoff_newton(one30 / 2, 100 * one24) >= 0);
    /* a1 * c */
    PCL_CHECK_NEAR(pcl_test_coef_to_double(svf_ac(one30 / 2, 2 * (1 << 28))), 1.0, 2e-9);
    PCL_CHECK_NEAR(pcl_test_coef_to_double(svf_ac(one30, 3 * (1 << 28) / 2)), 1.5, 2e-9);
    PCL_CHECK_EQ_INT(svf_ac(0, 5 << 28), 0);
    /* multiply */
    PCL_CHECK_EQ_INT(pcl_wide_mul(1 << 28, 3 << 28), 3 << 28);
    PCL_CHECK_EQ_INT(pcl_wide_mul(-(1 << 28), 3 << 28), -(3 << 28));
    PCL_CHECK_EQ_INT(pcl_wide_mul(0, INT32_MAX), 0);
}

#else /* float */

static void test_clamp_and_newton(void)
{
    PCL_CHECK(svf_clamp(0.5f) == 0.5f);
    PCL_CHECK(svf_clamp(9.0f) == 8.0f);
    PCL_CHECK(svf_clamp(-9.0f) == -8.0f);
    PCL_CHECK(svf_clamp(8.0f) == 8.0f || svf_clamp(8.0f) == 8.0f);
    PCL_CHECK(svf_clamp(NAN) == -8.0f); /* a NaN is clamped too */
    PCL_CHECK(svf_clamp(INFINITY) == 8.0f && svf_clamp(-INFINITY) == -8.0f);
    PCL_CHECK_NEAR(pcl_cutoff_newton(1.0f, 1.0f), 1.0, 1e-7);
    PCL_CHECK_NEAR(pcl_cutoff_newton(0.5f, 2.0f), 0.5, 1e-7);
    PCL_CHECK(pcl_cutoff_newton(1.0f, 40.0f) >= 0.0f); /* never negative */
    PCL_CHECK_NEAR(svf_ac(0.5f, 2.0f), 1.0, 1e-7);
}

#endif

static void test_mappings(void)
{
    /* resonance -> damping: sqrt(2) (1 - r)^2 */
    PCL_CHECK_NEAR(
        pcl_test_coef_to_double(svf_damping_from_resonance(PCL_COEF(0.0))), sqrt(2.0), DAMP_TOL);
    PCL_CHECK_NEAR(pcl_test_coef_to_double(svf_damping_from_resonance(PCL_COEF(0.5))),
        sqrt(2.0) * 0.25, DAMP_TOL);
    PCL_CHECK_NEAR(pcl_test_coef_to_double(svf_damping_from_resonance(PCL_COEF(0.9))),
        sqrt(2.0) * 0.01, DAMP_TOL);
    PCL_CHECK_NEAR(pcl_test_coef_to_double(svf_damping_from_resonance(PCL_COEF(1.0))), 0.0, 1e-9);
    /* cutoff -> octaves above 1 kHz, divided by 8 (Q2.30 / float) */
    PCL_CHECK_NEAR(pcl_test_coef_to_double(pcl_cutoff_u_from_hz(PCL_HZ(1000.0))), 0.0, 1e-4);
    PCL_CHECK_NEAR(pcl_test_coef_to_double(pcl_cutoff_u_from_hz(PCL_HZ(2000.0))), 1.0 / 8.0, 1e-4);
    PCL_CHECK_NEAR(pcl_test_coef_to_double(pcl_cutoff_u_from_hz(PCL_HZ(500.0))), -1.0 / 8.0, 1e-4);
    PCL_CHECK_NEAR(pcl_test_coef_to_double(pcl_cutoff_u_from_hz(PCL_HZ(16000.0))), 0.5, 1e-4);
}

static void test_target_coefficients(void)
{
    static const double fcs[] = { 30.0, 440.0, 1000.0, 7000.0, 18000.0 };
    size_t              i;

    for (i = 0; i < sizeof fcs / sizeof fcs[0]; i++)
    {
        pcl_svf_config_t c;
        pcl_svf_t        f;
        pcl_wide_t       g, cc;
        pcl_coef_t       a1;
        double           gd, cd, ad, k;

        c.sample_rate_hz = 48000u;
        c.cutoff_hz
            = PCL_HZ(0.0) + (pcl_hz_t)(fcs[i] * (PCL_SAMPLE_TYPE == PCL_FLOAT ? 1.0 : 65536.0));
        c.resonance = PCL_COEF(0.7);
        c.mod_depth_oct = PCL_OCT(0.0);
        c.saturate = 0;
        memset(&f, 0, sizeof f);
        PCL_CHECK_EQ_INT(pcl_svf_init(&f, &c), PCL_OK);
        svf_target(&f, 0, &g, &cc, &a1);
        k = sqrt(2.0) * 0.3 * 0.3;
#if PCL_SAMPLE_TYPE == PCL_FLOAT
        gd = g;
        cd = cc;
        ad = a1;
#else
        gd = (double)g / 268435456.0;
        cd = (double)cc / 268435456.0;
        ad = (double)a1 / 1073741824.0;
#endif
        PCL_CHECK_NEAR(gd, tan(pi * fcs[i] / 48000.0), tan(pi * fcs[i] / 48000.0) * 1e-4);
        PCL_CHECK_NEAR(cd - gd, k, 1e-7);
        PCL_CHECK_NEAR(ad, 1.0 / (1.0 + gd * cd), 2e-9 + 1e-7 * ad);
        /* a modulation of one octave doubles the frequency, up to the 0.45 fs limit */
        {
            const double expect_ratio = fmin(2.0, 0.45 * 48000.0 / fcs[i]);
            pcl_wide_t   g2, c2;
            pcl_coef_t   a2;
#if PCL_SAMPLE_TYPE == PCL_FLOAT
            svf_target(&f, 1.0f, &g2, &c2, &a2);
            PCL_CHECK_NEAR(atan(g2) / atan(g), expect_ratio, expect_ratio * 2e-3);
#else
            svf_target(&f, PCL_OCT(1.0), &g2, &c2, &a2);
            PCL_CHECK_NEAR(
                atan((double)g2 / 268435456.0) / atan(gd), expect_ratio, expect_ratio * 2e-3);
#endif
        }
    }
}

int main(void)
{
#if PCL_SAMPLE_TYPE != PCL_FLOAT
    PCL_RUN_TEST(test_saturation);
    PCL_RUN_TEST(test_conversions);
    PCL_RUN_TEST(test_newton_and_products);
#else
    PCL_RUN_TEST(test_clamp_and_newton);
#endif
    PCL_RUN_TEST(test_mappings);
    PCL_RUN_TEST(test_target_coefficients);
    return pcl_test_summary("svf internals");
}
