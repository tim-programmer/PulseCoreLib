/*
 * pcl_platform.h - compiler and CPU detection plus portability macros.
 *
 * This is the only header in the core allowed to contain compiler-specific
 * syntax. Everything else uses the macros defined here.
 *
 * Note on naming: PCL_CPU_* describe the machine being compiled for. The
 * PCL_ARCH_* macros are reserved for selecting optional optimized code paths
 * under src/arch/ (cmsis, esp, sse, neon); they are set by the build, never here.
 */
#ifndef PCL_PLATFORM_H
#define PCL_PLATFORM_H

/* ---- Compiler (clang defines __GNUC__, so test it first) ---- */

#if defined(_MSC_VER)
#define PCL_COMPILER_MSVC 1
#elif defined(__clang__)
#define PCL_COMPILER_CLANG 1
#elif defined(__GNUC__)
#define PCL_COMPILER_GCC 1
#else
#define PCL_COMPILER_UNKNOWN 1
#endif

/* ---- CPU family ---- */

#if defined(__x86_64__) || defined(_M_X64) || defined(_M_AMD64)
#define PCL_CPU_X86_64 1
#elif defined(__i386__) || defined(_M_IX86)
#define PCL_CPU_X86 1
#elif defined(__aarch64__) || defined(_M_ARM64)
#define PCL_CPU_AARCH64 1
#elif defined(__arm__) || defined(_M_ARM)
#define PCL_CPU_ARM 1
#elif defined(__riscv)
#define PCL_CPU_RISCV 1
#else
#define PCL_CPU_UNKNOWN 1
#endif

/* ---- C / C++ linkage ---- */

#ifdef __cplusplus
#define PCL_BEGIN_DECLS                                                                            \
    extern "C"                                                                                     \
    {
#define PCL_END_DECLS }
#else
#define PCL_BEGIN_DECLS
#define PCL_END_DECLS
#endif

/* ---- inline: for small helpers defined in headers ---- */

#if defined(PCL_COMPILER_MSVC) && !defined(__cplusplus)
#define PCL_INLINE static __inline
#else
#define PCL_INLINE static inline
#endif

/* ---- restrict: promise that a pointer is not aliased by another argument ---- */

#if defined(__cplusplus) || defined(PCL_COMPILER_MSVC)
#define PCL_RESTRICT __restrict
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 199901L
#define PCL_RESTRICT restrict
#elif defined(__GNUC__)
#define PCL_RESTRICT __restrict__
#else
#define PCL_RESTRICT
#endif

/* ---- alignment: place before the declaration, e.g. PCL_ALIGNED(16) float buf[64]; ---- */

#if defined(PCL_COMPILER_MSVC)
#define PCL_ALIGNED(bytes) __declspec(align(bytes))
#elif defined(__GNUC__)
#define PCL_ALIGNED(bytes) __attribute__((aligned(bytes)))
#else
#define PCL_ALIGNED(bytes)
#endif

/* ---- force-inline: for a body that is written once and specialised by a constant argument ---- */

#if defined(PCL_COMPILER_MSVC)
#define PCL_FORCE_INLINE static __forceinline
#elif defined(__GNUC__)
#define PCL_FORCE_INLINE static inline __attribute__((always_inline))
#else
#define PCL_FORCE_INLINE static inline
#endif

/*
 * Makes a 32-bit integer variable opaque to the optimizer: the compiler no longer knows how it was
 * computed, only that it is a plain 32-bit register. Use it on a saturated value that is about to
 * be multiplied: otherwise GCC folds the 64-bit saturation into the multiplication and emits a full
 * 64 x 64 product (three multiplies and spills) where one 32 x 32 -> 64 multiply (smull) is meant.
 * No effect on the value.
 */
#if defined(__GNUC__)
#define PCL_OPAQUE_I32(x) __asm__("" : "+r"(x))
#else
#define PCL_OPAQUE_I32(x) ((void)0)
#endif

/*
 * Compile-time assertion usable at file scope in C99. `name` must be a valid
 * identifier fragment that is unique within the translation unit.
 */
#define PCL_STATIC_ASSERT(cond, name) typedef char pcl_static_assert_##name[(cond) ? 1 : -1]

#endif /* PCL_PLATFORM_H */
