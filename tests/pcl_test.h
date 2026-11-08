/*
 * pcl_test.h - minimal dependency-free test harness.
 *
 * Usage (one test file == one translation unit == one executable):
 *
 *     #include "pcl_test.h"
 *     static void test_something(void) { PCL_CHECK(1 + 1 == 2); }
 *     int main(void)
 *     {
 *         PCL_RUN_TEST(test_something);
 *         return pcl_test_summary("suite name");
 *     }
 *
 * Checks are non-fatal: a failing check prints its location and the test goes
 * on. The process exit code is non-zero if anything failed, which is what ctest
 * looks at. Host-only code - it uses stdio, doubles and libm freely.
 */
#ifndef PCL_TEST_H
#define PCL_TEST_H

#include <float.h>
#include <math.h>
#include <stdio.h>

#include "pcl/pcl.h"

static int pcl_test_checks;
static int pcl_test_failures;

#define PCL_CHECK(cond)                                                                            \
    do                                                                                             \
    {                                                                                              \
        pcl_test_checks++;                                                                         \
        if (!(cond))                                                                               \
        {                                                                                          \
            pcl_test_failures++;                                                                   \
            printf("    FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                             \
        }                                                                                          \
    } while (0)

#define PCL_CHECK_EQ_INT(actual, expected)                                                         \
    do                                                                                             \
    {                                                                                              \
        const long long pcl_a_ = (long long)(actual);                                              \
        const long long pcl_e_ = (long long)(expected);                                            \
        pcl_test_checks++;                                                                         \
        if (pcl_a_ != pcl_e_)                                                                      \
        {                                                                                          \
            pcl_test_failures++;                                                                   \
            printf("    FAIL %s:%d: %s == %s (got %lld, expected %lld)\n", __FILE__, __LINE__,     \
                #actual, #expected, pcl_a_, pcl_e_);                                               \
        }                                                                                          \
    } while (0)

#define PCL_CHECK_NEAR(actual, expected, tol)                                                      \
    do                                                                                             \
    {                                                                                              \
        const double pcl_a_ = (double)(actual);                                                    \
        const double pcl_e_ = (double)(expected);                                                  \
        pcl_test_checks++;                                                                         \
        if (!(fabs(pcl_a_ - pcl_e_) <= (double)(tol)))                                             \
        {                                                                                          \
            pcl_test_failures++;                                                                   \
            printf("    FAIL %s:%d: %s ~= %s (got %.9g, expected %.9g, tol %.3g)\n", __FILE__,     \
                __LINE__, #actual, #expected, pcl_a_, pcl_e_, (double)(tol));                      \
        }                                                                                          \
    } while (0)

#define PCL_RUN_TEST(fn) pcl_test_run(#fn, fn)

PCL_INLINE void pcl_test_run(const char *name, void (*fn)(void))
{
    const int failures_before = pcl_test_failures;
    fn();
    printf("[%s] %s\n", pcl_test_failures == failures_before ? " OK " : "FAIL", name);
}

PCL_INLINE int pcl_test_summary(const char *suite)
{
    printf("%s: %d checks, %d failed\n", suite, pcl_test_checks, pcl_test_failures);
    return pcl_test_failures == 0 ? 0 : 1;
}

/* ---- Sample helpers: let one test source run against all three sample types ---- */

#define PCL_TEST_Q30_ONE 1073741824.0

#if PCL_SAMPLE_TYPE == PCL_FLOAT
#define PCL_TEST_SAMPLE_TOL 1e-6 /* a few ulps near full scale */
#elif PCL_SAMPLE_TYPE == PCL_Q15
#define PCL_TEST_SAMPLE_TOL (2.0 / 32768.0) /* 2 LSB */
#else
#define PCL_TEST_SAMPLE_TOL (2.0 / 2147483648.0) /* 2 LSB */
#endif

/* v in [-1, 1] -> sample, rounded to nearest and saturated. */
PCL_INLINE pcl_sample_t pcl_test_from_double(double v)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return (float)v;
#elif PCL_SAMPLE_TYPE == PCL_Q15
    double scaled = floor(v * 32768.0 + 0.5);
    return scaled >= 32767.0 ? INT16_MAX : (scaled <= -32768.0 ? INT16_MIN : (int16_t)scaled);
#else
    double scaled = floor(v * 2147483648.0 + 0.5);
    return scaled >= 2147483647.0 ? INT32_MAX
                                  : (scaled <= -2147483648.0 ? INT32_MIN : (int32_t)scaled);
#endif
}

PCL_INLINE double pcl_test_to_double(pcl_sample_t s)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return (double)s;
#elif PCL_SAMPLE_TYPE == PCL_Q15
    return (double)s / 32768.0;
#else
    return (double)s / 2147483648.0;
#endif
}

PCL_INLINE double pcl_test_coef_to_double(pcl_coef_t c)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return (double)c;
#else
    return (double)c / PCL_TEST_Q30_ONE;
#endif
}

/*
 * What a reference value becomes once stored in a sample: fixed-point saturates to
 * the representable range, float does not clip.
 */
PCL_INLINE double pcl_test_clip(double v)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return v;
#else
    const double hi = pcl_test_to_double(PCL_SAMPLE_TYPE == PCL_Q15 ? INT16_MAX : INT32_MAX);
    return v > hi ? hi : (v < -1.0 ? -1.0 : v);
#endif
}

/* Deterministic xorshift32 noise in [-1, 1). */
PCL_INLINE double pcl_test_rand(uint32_t *state)
{
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return (double)x / 2147483648.0 - 1.0;
}

#endif /* PCL_TEST_H */
