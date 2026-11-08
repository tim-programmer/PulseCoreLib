/*
 * pcl_types.h - sample types, status codes and fixed-point helpers.
 *
 * The sample type is chosen at build time with PCL_SAMPLE_TYPE:
 *
 *   PCL_FLOAT   pcl_sample_t = float    nominally [-1, 1]
 *   PCL_Q15     pcl_sample_t = int16_t  Q1.15
 *   PCL_Q31     pcl_sample_t = int32_t  Q1.31
 *
 * PCL_ALLOW_FLOAT_INIT: in Q15/Q31 builds, init/config functions may use float
 * only when this is defined. Processing paths never use float or double.
 */
#ifndef PCL_TYPES_H
#define PCL_TYPES_H

#include <stddef.h>
#include <stdint.h>

#include "pcl/pcl_platform.h"

PCL_BEGIN_DECLS

/* ---- Build configuration ---- */

/* Non-zero on purpose: an unknown PCL_SAMPLE_TYPE token evaluates to 0 in #if. */
#define PCL_FLOAT 1
#define PCL_Q15 2
#define PCL_Q31 3

#ifndef PCL_SAMPLE_TYPE
#define PCL_SAMPLE_TYPE PCL_FLOAT
#endif

/* Largest block a caller is guaranteed to be able to pass to a process function. */
#ifndef PCL_MAX_BLOCK
#define PCL_MAX_BLOCK 256
#endif

/* Control-rate modules update once every PCL_CONTROL_DIV samples. */
#ifndef PCL_CONTROL_DIV
#define PCL_CONTROL_DIV 16
#endif

/* Sample rates accepted by init functions. */
#define PCL_SAMPLE_RATE_MIN_HZ 8000u
#define PCL_SAMPLE_RATE_MAX_HZ 768000u

/* The helpers below rely on right shifts of negative values being arithmetic. */
PCL_STATIC_ASSERT(((-1) >> 1) == -1, arithmetic_right_shift);

/* ---- Fixed-point formats ---- */

#define PCL_Q15_SHIFT 15 /* Q1.15: int16_t, range [-1, 1)  */
#define PCL_Q24_SHIFT 24 /* Q8.24: int32_t, range [-128, 128) */
#define PCL_Q28_SHIFT 28 /* Q4.28: int32_t, range [-8, 8)  */
#define PCL_Q30_SHIFT 30 /* Q2.30: int32_t, range [-2, 2)  */
#define PCL_Q31_SHIFT 31 /* Q1.31: int32_t, range [-1, 1)  */

/*
 * Compile-time conversion of a constant to fixed point, rounded to nearest and
 * saturated to the representable range. Use with constant expressions only: the
 * argument is evaluated more than once, and with a runtime value it would pull
 * the float runtime into a no-FPU build.
 */
#define PCL_FIXED_CONST_(x, shift, lo, hi)                                                         \
    ((x) * (double)(1ull << (shift)) >= (double)(hi)                                               \
            ? (double)(hi)                                                                         \
            : ((x) * (double)(1ull << (shift)) <= (double)(lo)                                     \
                      ? (double)(lo)                                                               \
                      : (x) * (double)(1ull << (shift)) + ((x) < 0 ? -0.5 : 0.5)))

#define PCL_CONST_Q15(x) ((int16_t)PCL_FIXED_CONST_(x, PCL_Q15_SHIFT, INT16_MIN, INT16_MAX))
#define PCL_CONST_Q24(x) ((int32_t)PCL_FIXED_CONST_(x, PCL_Q24_SHIFT, INT32_MIN, INT32_MAX))
#define PCL_CONST_Q28(x) ((int32_t)PCL_FIXED_CONST_(x, PCL_Q28_SHIFT, INT32_MIN, INT32_MAX))
#define PCL_CONST_Q30(x) ((int32_t)PCL_FIXED_CONST_(x, PCL_Q30_SHIFT, INT32_MIN, INT32_MAX))
#define PCL_CONST_Q31(x) ((int32_t)PCL_FIXED_CONST_(x, PCL_Q31_SHIFT, INT32_MIN, INT32_MAX))

/* ---- Sample and coefficient types ---- */

#if PCL_SAMPLE_TYPE == PCL_FLOAT

typedef float pcl_sample_t;
typedef float pcl_coef_t;
typedef float pcl_hz_t;
typedef float pcl_oct_t;
typedef float pcl_db_t;
typedef float pcl_wide_t;
typedef float pcl_quality_t;
#define PCL_COEF(x) ((float)(x))
#define PCL_HZ(x) ((float)(x))
#define PCL_OCT(x) ((float)(x))
#define PCL_DB(x) ((float)(x))
#define PCL_WIDE(x) ((float)(x))
#define PCL_QUALITY(x) ((float)(x))

#elif PCL_SAMPLE_TYPE == PCL_Q15

typedef int16_t  pcl_sample_t;
typedef int32_t  pcl_coef_t;
typedef uint32_t pcl_hz_t;
typedef int32_t  pcl_oct_t;
typedef int32_t  pcl_db_t;
typedef int32_t  pcl_wide_t;
typedef uint32_t pcl_quality_t;
#define PCL_COEF(x) PCL_CONST_Q30(x)
#define PCL_HZ(x) ((uint32_t)((x) * 65536.0 + 0.5))
#define PCL_OCT(x) PCL_CONST_Q28(x)
#define PCL_DB(x) PCL_CONST_Q24(x)
#define PCL_WIDE(x) PCL_CONST_Q28(x)
#define PCL_QUALITY(x) ((uint32_t)((x) * 65536.0 + 0.5))

#elif PCL_SAMPLE_TYPE == PCL_Q31

typedef int32_t  pcl_sample_t;
typedef int32_t  pcl_coef_t;
typedef uint32_t pcl_hz_t;
typedef int32_t  pcl_oct_t;
typedef int32_t  pcl_db_t;
typedef int32_t  pcl_wide_t;
typedef uint32_t pcl_quality_t;
#define PCL_COEF(x) PCL_CONST_Q30(x)
#define PCL_HZ(x) ((uint32_t)((x) * 65536.0 + 0.5))
#define PCL_OCT(x) PCL_CONST_Q28(x)
#define PCL_DB(x) PCL_CONST_Q24(x)
#define PCL_WIDE(x) PCL_CONST_Q28(x)
#define PCL_QUALITY(x) ((uint32_t)((x) * 65536.0 + 0.5))

#else
#error "PCL_SAMPLE_TYPE must be PCL_FLOAT, PCL_Q15 or PCL_Q31"
#endif

/*
 * pcl_coef_t holds a scalar coefficient or parameter value such as a gain.
 * It is a float in float builds and Q2.30 in both fixed-point builds (even Q15,
 * so coefficients keep their precision). Write constants with PCL_COEF(x).
 *
 * pcl_hz_t is a frequency in Hz: a float in float builds, unsigned Q16.16 in fixed-point
 * builds (resolution 0.000015 Hz, up to 65535 Hz), so frequency setters need no float
 * there. pcl_oct_t is an octave amount: a float, or signed Q4.28 (range [-8, 8)).
 * Write constants with PCL_HZ(x) / PCL_OCT(x); like the PCL_CONST_* macros, in fixed-point
 * builds they must be constant expressions.
 *
 * pcl_db_t is a level in decibels: a float, or signed Q8.24 (range [-128, 128) dB, resolution
 * 6e-8 dB). pcl_wide_t is a signal value that may exceed full scale (a filter state, a drive-scaled
 * input): a float, or signed Q4.28 (range [-8, 8), full scale = 1.0 = 2^28). Write constants with
 * PCL_DB(x) / PCL_WIDE(x). Both are conversions only; see pcl_math.h.
 *
 * pcl_quality_t is a dimensionless quality factor Q of a filter: a float, or unsigned Q16.16
 * (resolution 1.5e-5, up to 65535). It has the representation of pcl_hz_t on purpose, so the
 * frequency-ratio functions of pcl_math.h apply to it. Write constants with PCL_QUALITY(x).
 */

/* Limits of the decibel conversions in pcl_math.h. */
#define PCL_DB_MIN PCL_DB(-120.0) /* pcl_gain_to_db floor; pcl_db_to_gain is 0 below it       */
#define PCL_DB_MAX PCL_DB(6.0)    /* pcl_coef_t (Q2.30) cannot hold gains of 2.0 (6.0206 dB)   */

/* ---- Status codes ---- */

typedef enum
{
    PCL_OK = 0,
    PCL_ERR_NULL_POINTER = -1,     /* a required pointer was NULL                  */
    PCL_ERR_OUT_OF_RANGE = -2,     /* a parameter is outside its documented range  */
    PCL_ERR_BUFFER_TOO_SMALL = -3, /* caller-provided memory is too small          */
} pcl_status_t;

/* ---- Saturation ---- */

/* Clamps a 32-bit value to the Q15 range. */
PCL_INLINE int16_t pcl_sat_q15(int32_t x)
{
    if (x > INT16_MAX)
        return INT16_MAX;
    if (x < INT16_MIN)
        return INT16_MIN;
    return (int16_t)x;
}

/* Clamps a 64-bit value to the 32-bit range (Q31 and Q2.30 results). */
PCL_INLINE int32_t pcl_sat_q31(int64_t x)
{
    if (x > INT32_MAX)
        return INT32_MAX;
    if (x < INT32_MIN)
        return INT32_MIN;
    return (int32_t)x;
}

/* ---- Multiplication ----
 *
 * All products round to nearest (ties toward +infinity) and saturate. The only
 * product of two same-format operands that can overflow is (-1) * (-1).
 */

/* Q15 * Q15 -> Q15. Accumulates in int32_t. */
PCL_INLINE int16_t pcl_mul_q15(int16_t a, int16_t b)
{
    int32_t acc = (int32_t)a * (int32_t)b + (1 << (PCL_Q15_SHIFT - 1));
    return pcl_sat_q15(acc >> PCL_Q15_SHIFT);
}

/* Q31 * Q31 -> Q31. Accumulates in int64_t. */
PCL_INLINE int32_t pcl_mul_q31(int32_t a, int32_t b)
{
    int64_t acc = (int64_t)a * (int64_t)b + ((int64_t)1 << (PCL_Q31_SHIFT - 1));
    return pcl_sat_q31(acc >> PCL_Q31_SHIFT);
}

/*
 * Q15 sample * Q2.30 coefficient -> Q15, using only 32-bit arithmetic so it stays
 * cheap on Cortex-M0+. The coefficient is split into a signed high half and an
 * unsigned low half; the result is bit-identical to a full 64-bit product.
 */
PCL_INLINE int16_t pcl_mul_q15_q30(int16_t x, int32_t c)
{
    const int32_t c_hi = c >> 16;                          /* signed, |c_hi| <= 2^15 */
    const int32_t c_lo = (int32_t)((uint32_t)c & 0xFFFFu); /* 0 .. 65535             */
    int32_t       acc = (int32_t)x * c_hi;                 /* weight 2^16            */
    acc += ((int32_t)x * c_lo) >> 16;                      /* floor, exact (see above) */
    acc += 1 << (PCL_Q30_SHIFT - 16 - 1);                  /* rounding, pre-shift    */
    return pcl_sat_q15(acc >> (PCL_Q30_SHIFT - 16));
}

/* Q31 sample * Q2.30 coefficient -> Q31. Accumulates in int64_t. */
PCL_INLINE int32_t pcl_mul_q31_q30(int32_t x, int32_t c)
{
    int64_t acc = (int64_t)x * (int64_t)c + ((int64_t)1 << (PCL_Q30_SHIFT - 1));
    return pcl_sat_q31(acc >> PCL_Q30_SHIFT);
}

/*
 * Non-negative coefficient c (any Q format) times a Q15 sample m, giving a value in the
 * same Q format as c. Used to scale a modulation sample by a depth. 32-bit arithmetic
 * only; c must be >= 0 (bit-identical to a 64-bit product for every such c).
 */
PCL_INLINE int32_t pcl_mul_coef_q15(int32_t c, int16_t m)
{
    const int32_t c_hi = c >> 16;                          /* 0 .. 2^15 - 1  */
    const int32_t c_lo = (int32_t)((uint32_t)c & 0xFFFFu); /* 0 .. 65535     */
    int32_t       acc = (int32_t)m * c_hi * 2;             /* exact multiple of 2^15 */
    acc += ((int32_t)m * c_lo + (1 << (PCL_Q15_SHIFT - 1))) >> PCL_Q15_SHIFT;
    return acc;
}

/* Coefficient c (any Q format) times a Q31 sample m, same Q format as c. int64_t accumulator. */
PCL_INLINE int32_t pcl_mul_coef_q31(int32_t c, int32_t m)
{
    int64_t acc = (int64_t)c * (int64_t)m + ((int64_t)1 << (PCL_Q31_SHIFT - 1));
    return pcl_sat_q31(acc >> PCL_Q31_SHIFT);
}

PCL_END_DECLS

#endif /* PCL_TYPES_H */
