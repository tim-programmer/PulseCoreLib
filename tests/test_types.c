/* Tests for pcl_types.h: sample types, constant macros, saturation and fixed-point products. */
#include "pcl_test.h"

static void test_type_sizes(void)
{
    PCL_CHECK_EQ_INT(sizeof(pcl_coef_t), 4);
#if PCL_SAMPLE_TYPE == PCL_Q15
    PCL_CHECK_EQ_INT(sizeof(pcl_sample_t), 2);
#else
    PCL_CHECK_EQ_INT(sizeof(pcl_sample_t), 4);
#endif
}

static void test_constant_macros(void)
{
    PCL_CHECK_EQ_INT(PCL_CONST_Q15(0.5), 16384);
    PCL_CHECK_EQ_INT(PCL_CONST_Q15(-0.5), -16384);
    PCL_CHECK_EQ_INT(PCL_CONST_Q15(-1.0), INT16_MIN);
    PCL_CHECK_EQ_INT(PCL_CONST_Q15(1.0), INT16_MAX); /* +1.0 is not representable: saturates */
    PCL_CHECK_EQ_INT(PCL_CONST_Q15(0.0), 0);

    PCL_CHECK_EQ_INT(PCL_CONST_Q30(1.0), 1 << 30);
    PCL_CHECK_EQ_INT(PCL_CONST_Q30(0.5), 1 << 29);
    PCL_CHECK_EQ_INT(PCL_CONST_Q30(2.0), INT32_MAX);
    PCL_CHECK_EQ_INT(PCL_CONST_Q30(-2.0), INT32_MIN);

    PCL_CHECK_EQ_INT(PCL_CONST_Q31(0.5), 1 << 30);
    PCL_CHECK_EQ_INT(PCL_CONST_Q31(-0.5), -(1 << 30));
    PCL_CHECK_EQ_INT(PCL_CONST_Q31(1.0), INT32_MAX);
    PCL_CHECK_EQ_INT(PCL_CONST_Q31(-1.0), INT32_MIN);

    PCL_CHECK_NEAR(pcl_test_coef_to_double(PCL_COEF(0.75)), 0.75, 1e-9);
}

static void test_saturation(void)
{
    PCL_CHECK_EQ_INT(pcl_sat_q15(0), 0);
    PCL_CHECK_EQ_INT(pcl_sat_q15(INT16_MAX), INT16_MAX);
    PCL_CHECK_EQ_INT(pcl_sat_q15(INT16_MAX + 1), INT16_MAX);
    PCL_CHECK_EQ_INT(pcl_sat_q15(INT32_MAX), INT16_MAX);
    PCL_CHECK_EQ_INT(pcl_sat_q15(INT16_MIN), INT16_MIN);
    PCL_CHECK_EQ_INT(pcl_sat_q15(INT16_MIN - 1), INT16_MIN);
    PCL_CHECK_EQ_INT(pcl_sat_q15(INT32_MIN), INT16_MIN);

    PCL_CHECK_EQ_INT(pcl_sat_q31(0), 0);
    PCL_CHECK_EQ_INT(pcl_sat_q31(INT32_MAX), INT32_MAX);
    PCL_CHECK_EQ_INT(pcl_sat_q31((int64_t)INT32_MAX + 1), INT32_MAX);
    PCL_CHECK_EQ_INT(pcl_sat_q31(INT64_MAX), INT32_MAX);
    PCL_CHECK_EQ_INT(pcl_sat_q31(INT32_MIN), INT32_MIN);
    PCL_CHECK_EQ_INT(pcl_sat_q31((int64_t)INT32_MIN - 1), INT32_MIN);
    PCL_CHECK_EQ_INT(pcl_sat_q31(INT64_MIN), INT32_MIN);
}

static void test_mul_q15(void)
{
    PCL_CHECK_EQ_INT(pcl_mul_q15(16384, 16384), 8192);   /* 0.5 * 0.5 = 0.25 */
    PCL_CHECK_EQ_INT(pcl_mul_q15(16384, -16384), -8192); /* sign handling    */
    PCL_CHECK_EQ_INT(pcl_mul_q15(INT16_MAX, 0), 0);
    PCL_CHECK_EQ_INT(pcl_mul_q15(1, 16384), 1);                     /* 0.5 LSB rounds up */
    PCL_CHECK_EQ_INT(pcl_mul_q15(1, 16383), 0);                     /* just under: down  */
    PCL_CHECK_EQ_INT(pcl_mul_q15(INT16_MIN, INT16_MIN), INT16_MAX); /* (-1)(-1) saturates */
    PCL_CHECK_EQ_INT(pcl_mul_q15(INT16_MIN, INT16_MAX), -INT16_MAX);
}

static void test_mul_q31(void)
{
    PCL_CHECK_EQ_INT(pcl_mul_q31(1 << 30, 1 << 30), 1 << 29); /* 0.5 * 0.5 = 0.25 */
    PCL_CHECK_EQ_INT(pcl_mul_q31(1 << 30, -(1 << 30)), -(1 << 29));
    PCL_CHECK_EQ_INT(pcl_mul_q31(INT32_MAX, 0), 0);
    PCL_CHECK_EQ_INT(pcl_mul_q31(1, 1 << 30), 1);                   /* 0.5 LSB rounds up */
    PCL_CHECK_EQ_INT(pcl_mul_q31(INT32_MIN, INT32_MIN), INT32_MAX); /* saturates */
}

/* 64-bit references for the mixed-format products. */
static int32_t ref_mul_q15_q30(int16_t x, int32_t c)
{
    int64_t p = ((int64_t)x * c + ((int64_t)1 << 29)) >> 30;
    return p > INT16_MAX ? INT16_MAX : (p < INT16_MIN ? INT16_MIN : (int32_t)p);
}

static int32_t ref_mul_q31_q30(int32_t x, int32_t c)
{
    int64_t p = (((int64_t)x * c) + ((int64_t)1 << 29)) >> 30;
    return p > INT32_MAX ? INT32_MAX : (p < INT32_MIN ? INT32_MIN : (int32_t)p);
}

static void test_mul_mixed_matches_64bit_reference(void)
{
    static const int16_t xs[] = { INT16_MIN, INT16_MIN + 1, -12345, -1, 0, 1, 12345, INT16_MAX };
    static const int32_t cs[] = { INT32_MIN, INT32_MIN + 1, -(1 << 30), -0x12345678, -65537, -65536,
        -65535, -1, 0, 1, 65535, 65536, 65537, 1 << 29, 1 << 30, 0x12345678, INT32_MAX };
    uint32_t             seed = 0x1234ABCDu;
    size_t               i, j;
    int                  n;

    for (i = 0; i < sizeof xs / sizeof xs[0]; i++)
    {
        for (j = 0; j < sizeof cs / sizeof cs[0]; j++)
        {
            PCL_CHECK_EQ_INT(pcl_mul_q15_q30(xs[i], cs[j]), ref_mul_q15_q30(xs[i], cs[j]));
            PCL_CHECK_EQ_INT(
                pcl_mul_q31_q30(xs[i] * 65536, cs[j]), ref_mul_q31_q30(xs[i] * 65536, cs[j]));
        }
    }

    for (n = 0; n < 200000; n++)
    {
        const int16_t x = (int16_t)(pcl_test_rand(&seed) * 32768.0);
        const int32_t c = (int32_t)(pcl_test_rand(&seed) * 2147483648.0);
        const int32_t x31 = (int32_t)(pcl_test_rand(&seed) * 2147483648.0);
        PCL_CHECK_EQ_INT(pcl_mul_q15_q30(x, c), ref_mul_q15_q30(x, c));
        PCL_CHECK_EQ_INT(pcl_mul_q31_q30(x31, c), ref_mul_q31_q30(x31, c));
    }
}

static void test_mul_by_unity_is_identity(void)
{
    const int32_t one = PCL_CONST_Q30(1.0);
    int32_t       v;

    for (v = INT16_MIN; v <= INT16_MAX; v += 7)
        PCL_CHECK_EQ_INT(pcl_mul_q15_q30((int16_t)v, one), v);
    PCL_CHECK_EQ_INT(pcl_mul_q31_q30(INT32_MIN, one), INT32_MIN);
    PCL_CHECK_EQ_INT(pcl_mul_q31_q30(INT32_MAX, one), INT32_MAX);
    PCL_CHECK_EQ_INT(pcl_mul_q31_q30(123456789, one), 123456789);
}

static void test_hz_and_oct_macros(void)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    PCL_CHECK_NEAR(PCL_HZ(440.0), 440.0, 0.0);
    PCL_CHECK_NEAR(PCL_OCT(1.5), 1.5, 0.0);
#else
    PCL_CHECK_EQ_INT(PCL_HZ(440.0), 440 * 65536);
    PCL_CHECK_EQ_INT(PCL_HZ(0.5), 32768);
    PCL_CHECK_EQ_INT(PCL_OCT(1.0), 1 << 28);
    PCL_CHECK_EQ_INT(PCL_OCT(-8.0), INT32_MIN);
    PCL_CHECK_EQ_INT(PCL_OCT(8.0), INT32_MAX); /* +8 is not representable: saturates */
#endif
}

static void test_db_and_wide_macros(void)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    PCL_CHECK_NEAR(PCL_DB(-6.0), -6.0, 0.0);
    PCL_CHECK_NEAR(PCL_WIDE(2.5), 2.5, 0.0);
#else
    PCL_CHECK_EQ_INT(PCL_DB(1.0), 1 << 24);
    PCL_CHECK_EQ_INT(PCL_DB(-120.0), -120 * (1 << 24));
    PCL_CHECK_EQ_INT(PCL_DB(200.0), INT32_MAX); /* +-128 dB is the range: saturates */
    PCL_CHECK_EQ_INT(PCL_DB(-200.0), INT32_MIN);
    PCL_CHECK_EQ_INT(PCL_WIDE(1.0), 1 << 28);
    PCL_CHECK_EQ_INT(PCL_WIDE(-8.0), INT32_MIN);
    PCL_CHECK_EQ_INT(PCL_WIDE(9.0), INT32_MAX);
    PCL_CHECK_EQ_INT(sizeof(pcl_db_t), 4);
    PCL_CHECK_EQ_INT(sizeof(pcl_wide_t), 4);
#endif
    PCL_CHECK_NEAR(PCL_DB_MIN, -120.0 * (PCL_SAMPLE_TYPE == PCL_FLOAT ? 1.0 : 16777216.0), 0.0);
    PCL_CHECK_NEAR(PCL_DB_MAX, 6.0 * (PCL_SAMPLE_TYPE == PCL_FLOAT ? 1.0 : 16777216.0), 0.0);
}

static void test_quality_macro(void)
{
    PCL_CHECK_NEAR(
        PCL_QUALITY(0.7071), 0.7071 * (PCL_SAMPLE_TYPE == PCL_FLOAT ? 1.0 : 65536.0), 0.5);
#if PCL_SAMPLE_TYPE != PCL_FLOAT
    PCL_CHECK_EQ_INT(PCL_QUALITY(1.0), 65536);
    PCL_CHECK_EQ_INT(PCL_QUALITY(0.2), 13107); /* rounded: 13107.2 */
    PCL_CHECK_EQ_INT(PCL_QUALITY(100.0), 6553600);
    PCL_CHECK_EQ_INT(sizeof(pcl_quality_t), 4);
    PCL_CHECK(PCL_QUALITY(0.5) < PCL_QUALITY(0.7071)); /* unsigned, ordered */
    /* the same representation as pcl_hz_t, so the frequency-ratio functions apply to it */
    PCL_CHECK_EQ_INT(PCL_QUALITY(8.0), PCL_HZ(8.0));
#else
    PCL_CHECK_NEAR(PCL_QUALITY(100.0), 100.0, 0.0);
#endif
}

static void test_mul_coef_matches_64bit_reference(void)
{
    static const int32_t cs[]
        = { 0, 1, 65535, 65536, 65537, 1 << 28, 1 << 30, 0x12345678, 0x7FFF0000, 0x7FFFFFFF };
    static const int16_t ms[] = { INT16_MIN, INT16_MIN + 1, -12345, -1, 0, 1, 12345, INT16_MAX };
    uint32_t             seed = 0xC0FFEEu;
    size_t               i, j;
    int                  n;

    for (i = 0; i < sizeof cs / sizeof cs[0]; i++)
    {
        for (j = 0; j < sizeof ms / sizeof ms[0]; j++)
        {
            const int64_t ref = ((int64_t)cs[i] * ms[j] + (1 << 14)) >> 15;
            PCL_CHECK_EQ_INT(pcl_mul_coef_q15(cs[i], ms[j]), ref);
        }
    }
    for (n = 0; n < 200000; n++)
    {
        const int32_t c = (int32_t)((pcl_test_rand(&seed) * 0.5 + 0.5) * 2147483647.0);
        const int16_t m = (int16_t)(pcl_test_rand(&seed) * 32768.0);
        const int32_t m31 = (int32_t)(pcl_test_rand(&seed) * 2147483648.0);
        const int64_t ref15 = ((int64_t)c * m + (1 << 14)) >> 15;
        const int64_t ref31 = ((int64_t)c * m31 + ((int64_t)1 << 30)) >> 31;
        PCL_CHECK_EQ_INT(pcl_mul_coef_q15(c, m), ref15);
        PCL_CHECK_EQ_INT(pcl_mul_coef_q31(c, m31), ref31); /* |c*m| < 2^62: never saturates */
    }
    PCL_CHECK_EQ_INT(pcl_mul_coef_q31(INT32_MAX, INT32_MIN), -INT32_MAX); /* sign and scale */
}

int main(void)
{
    PCL_RUN_TEST(test_type_sizes);
    PCL_RUN_TEST(test_constant_macros);
    PCL_RUN_TEST(test_saturation);
    PCL_RUN_TEST(test_mul_q15);
    PCL_RUN_TEST(test_mul_q31);
    PCL_RUN_TEST(test_mul_mixed_matches_64bit_reference);
    PCL_RUN_TEST(test_mul_by_unity_is_identity);
    PCL_RUN_TEST(test_hz_and_oct_macros);
    PCL_RUN_TEST(test_db_and_wide_macros);
    PCL_RUN_TEST(test_quality_macro);
    PCL_RUN_TEST(test_mul_coef_matches_64bit_reference);
    return pcl_test_summary("types");
}
