/* Tests for the linear parameter smoother. */
#include "pcl_test.h"

static double to_d(pcl_coef_t c) { return pcl_test_coef_to_double(c); }

static void test_ramp_samples(void)
{
#if PCL_SMOOTH_MS == 5
    PCL_CHECK_EQ_INT(pcl_smooth_ramp_samples(48000u), 240);
    PCL_CHECK_EQ_INT(pcl_smooth_ramp_samples(44100u), 221); /* 220.5 rounds up */
    PCL_CHECK_EQ_INT(pcl_smooth_ramp_samples(PCL_SAMPLE_RATE_MIN_HZ), 40);
    PCL_CHECK_EQ_INT(pcl_smooth_ramp_samples(PCL_SAMPLE_RATE_MAX_HZ), 3840);
#endif
    PCL_CHECK_EQ_INT(pcl_smooth_ramp_samples(1u), 1); /* never zero */
}

static void test_ramp_up_lands_exactly(void)
{
    pcl_smooth_t s;
    pcl_coef_t   v = PCL_COEF(0.0);
    double       prev = 0.0;
    int          n;

    pcl_smooth_init(&s, PCL_COEF(0.0), 10u);
    PCL_CHECK(pcl_smooth_is_settled(&s));

    pcl_smooth_set_target(&s, PCL_COEF(1.0));
    PCL_CHECK(!pcl_smooth_is_settled(&s));

    for (n = 0; n < 10; n++)
    {
        v = pcl_smooth_next(&s);
        PCL_CHECK(to_d(v) > prev); /* strictly rising, no overshoot */
        PCL_CHECK(to_d(v) <= 1.0);
        prev = to_d(v);
    }
    PCL_CHECK(v == PCL_COEF(1.0)); /* exact, not just close */
    PCL_CHECK(pcl_smooth_is_settled(&s));
    PCL_CHECK(pcl_smooth_next(&s) == PCL_COEF(1.0));
}

static void test_ramp_down(void)
{
    pcl_smooth_t s;
    pcl_coef_t   v = PCL_COEF(1.0);
    double       prev = 1.0;
    int          n;

    pcl_smooth_init(&s, PCL_COEF(1.0), 8u);
    pcl_smooth_set_target(&s, PCL_COEF(0.25));
    for (n = 0; n < 8; n++)
    {
        v = pcl_smooth_next(&s);
        PCL_CHECK(to_d(v) < prev);
        PCL_CHECK(to_d(v) >= 0.25);
        prev = to_d(v);
    }
    PCL_CHECK(v == PCL_COEF(0.25));
}

static void test_retarget_mid_ramp_is_continuous(void)
{
    pcl_smooth_t s;
    double       before, after;
    int          n;

    pcl_smooth_init(&s, PCL_COEF(0.0), 10u);
    pcl_smooth_set_target(&s, PCL_COEF(1.0));
    for (n = 0; n < 5; n++)
        (void)pcl_smooth_next(&s);
    before = to_d(pcl_smooth_next(&s)); /* ~0.6 */

    pcl_smooth_set_target(&s, PCL_COEF(0.0));
    PCL_CHECK(!pcl_smooth_is_settled(&s));
    after = to_d(pcl_smooth_next(&s));

    /* One step of the new ramp is at most |0.6 - 0.0| / 10. */
    PCL_CHECK(after < before);
    PCL_CHECK(before - after <= 0.06 + 1e-6);

    for (n = 1; n < 10; n++)
        (void)pcl_smooth_next(&s);
    PCL_CHECK(pcl_smooth_next(&s) == PCL_COEF(0.0));
}

static void test_short_ramp_jumps(void)
{
    pcl_smooth_t s;

    pcl_smooth_init(&s, PCL_COEF(0.0), 1u);
    pcl_smooth_set_target(&s, PCL_COEF(0.5));
    PCL_CHECK(pcl_smooth_is_settled(&s));
    PCL_CHECK(pcl_smooth_next(&s) == PCL_COEF(0.5));

    pcl_smooth_init(&s, PCL_COEF(0.0), 0u); /* 0 is treated as 1 */
    pcl_smooth_set_target(&s, PCL_COEF(0.5));
    PCL_CHECK(pcl_smooth_next(&s) == PCL_COEF(0.5));
}

static void test_same_target_does_not_ramp(void)
{
    pcl_smooth_t s;

    pcl_smooth_init(&s, PCL_COEF(0.5), 10u);
    pcl_smooth_set_target(&s, PCL_COEF(0.5));
    PCL_CHECK(pcl_smooth_is_settled(&s));
}

static void test_reset_snaps_to_target(void)
{
    pcl_smooth_t s;

    pcl_smooth_init(&s, PCL_COEF(0.0), 100u);
    pcl_smooth_set_target(&s, PCL_COEF(0.75));
    (void)pcl_smooth_next(&s);
    pcl_smooth_reset(&s);
    PCL_CHECK(pcl_smooth_is_settled(&s));
    PCL_CHECK(pcl_smooth_next(&s) == PCL_COEF(0.75));
}

static void test_extreme_span_does_not_overflow(void)
{
    /* Fixed point: the full-range difference does not fit in int32_t. */
    pcl_smooth_t s;
    pcl_coef_t   v = 0;
    int          n;

    pcl_smooth_init(&s, PCL_COEF(-2.0), 4u);
    pcl_smooth_set_target(&s, PCL_COEF(1.999));
    for (n = 0; n < 4; n++)
        v = pcl_smooth_next(&s);
    PCL_CHECK(v == PCL_COEF(1.999));
}

int main(void)
{
    PCL_RUN_TEST(test_ramp_samples);
    PCL_RUN_TEST(test_ramp_up_lands_exactly);
    PCL_RUN_TEST(test_ramp_down);
    PCL_RUN_TEST(test_retarget_mid_ramp_is_continuous);
    PCL_RUN_TEST(test_short_ramp_jumps);
    PCL_RUN_TEST(test_same_target_does_not_ramp);
    PCL_RUN_TEST(test_reset_snaps_to_target);
    PCL_RUN_TEST(test_extreme_span_does_not_overflow);
    return pcl_test_summary("smooth");
}
