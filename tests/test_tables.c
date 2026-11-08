/* Guards the checked-in lookup tables (src/core/pcl_tables.c) against libm. */
#include "core/pcl_tables.h"
#include "pcl_test.h"

static const double pi = 3.14159265358979323846;

static void test_sine_quarter_table(void)
{
    size_t i;

    for (i = 0; i <= PCL_QUARTER_SINE_SEGMENTS; i++)
    {
        const double ref = sin(pi / 2.0 * (double)i / (double)PCL_QUARTER_SINE_SEGMENTS);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
        PCL_CHECK_NEAR(pcl_sine_quarter_tab[i], ref, 1e-7);
#elif PCL_SAMPLE_TYPE == PCL_Q15
        PCL_CHECK_NEAR(pcl_sine_quarter_tab[i], ref * 32767.0, 0.5);
#else
        PCL_CHECK_NEAR(pcl_sine_quarter_tab[i], ref * 2147483647.0, 0.5);
#endif
        if (i > 0)
            PCL_CHECK(pcl_sine_quarter_tab[i] >= pcl_sine_quarter_tab[i - 1]); /* rising */
    }
    PCL_CHECK(pcl_sine_quarter_tab[0] == 0);
}

static void test_exp2_table(void)
{
    size_t i;

    for (i = 0; i <= PCL_EXP2_SEGMENTS; i++)
    {
        double ref = (exp2((double)i / (double)PCL_EXP2_SEGMENTS) - 1.0) * 65536.0;
        if (ref > 65535.0)
            ref = 65535.0; /* the last entry saturates */
        PCL_CHECK_NEAR(pcl_exp2_tab[i], ref, 0.5);
        if (i > 0)
            PCL_CHECK(pcl_exp2_tab[i] >= pcl_exp2_tab[i - 1]); /* the interpolator relies on this */
    }
    PCL_CHECK_EQ_INT(pcl_exp2_tab[0], 0);
}

static void test_log2_table(void)
{
    size_t i;

    for (i = 0; i <= PCL_LOG2_SEGMENTS; i++)
    {
        double ref = log2(1.0 + (double)i / (double)PCL_LOG2_SEGMENTS) * 65536.0;
        if (ref > 65535.0)
            ref = 65535.0; /* the last entry saturates */
        PCL_CHECK_NEAR(pcl_log2_tab[i], ref, 0.5);
        if (i > 0)
            PCL_CHECK(pcl_log2_tab[i] >= pcl_log2_tab[i - 1]);
    }
    PCL_CHECK_EQ_INT(pcl_log2_tab[0], 0);
}

static void test_tanh_table(void)
{
    size_t i;

    for (i = 0; i <= PCL_TANH_SEGMENTS; i++)
    {
        const double ref = tanh((double)i / (double)PCL_TANH_STEPS_PER_UNIT);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
        PCL_CHECK_NEAR(pcl_tanh_tab[i], ref, 1e-7);
#elif PCL_SAMPLE_TYPE == PCL_Q15
        PCL_CHECK_NEAR(pcl_tanh_tab[i], ref * 32767.0, 0.5);
#else
        PCL_CHECK_NEAR(pcl_tanh_tab[i], ref * 2147483647.0, 0.5);
#endif
        if (i > 0)
            PCL_CHECK(pcl_tanh_tab[i] >= pcl_tanh_tab[i - 1]); /* the interpolator relies on this */
    }
    PCL_CHECK(pcl_tanh_tab[0] == 0);
    /* 6 units at 64 steps per unit: the segment count and step must agree. */
    PCL_CHECK_EQ_INT(PCL_TANH_SEGMENTS, 6 * PCL_TANH_STEPS_PER_UNIT);
}

int main(void)
{
    PCL_RUN_TEST(test_sine_quarter_table);
    PCL_RUN_TEST(test_exp2_table);
    PCL_RUN_TEST(test_log2_table);
    PCL_RUN_TEST(test_tanh_table);
    return pcl_test_summary("tables");
}
