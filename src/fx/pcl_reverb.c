/*
 * pcl_reverb.c - see include/pcl/pcl_reverb.h and docs/modules/reverb.md.
 *
 * SMALL (Schroeder-Moorer): x -> predelay -> K_S x into 4 parallel combs, each
 *     line <- in + D(line out),   D = the damping shelf below, with DC gain g_i,
 * the comb outputs summed as L = c0 + c1 - c2 - c3 and R = c0 - c1 + c2 - c3, each channel through
 * two Schroeder all-passes (gain 0.7). M. R. Schroeder, "Natural sounding artificial
 * reverberation", JAES 1962; J. A. Moorer, "About this reverberation business", CMJ 1979. The comb
 * lengths are Freeverb's (Jezar at Dreampoint, 2000, public domain), converted to milliseconds.
 *
 * QUALITY (feedback delay network, J.-M. Jot and A. Chaigne, AES 1991): x -> predelay -> 4
 * all-pass diffusers (lengths and gains of J. Dattorro, "Effect design part 1", JAES 1997) -> K_Q u
 * injected into the 8 lines with the signs of Hadamard row 7. Each line output y_i goes through the
 * damping shelf D_i, then the 8 values through H8 / sqrt(8) (Sylvester Hadamard, orthogonal, as a
 * fast Walsh-Hadamard transform: 24 additions; the 1/sqrt(8) is folded into the shelf gains), back
 * into the lines. The wet outputs are rows 1 and 2 of the transform (orthogonal +-1 combinations,
 * so decorrelated). Lines 3 and 6 have their length modulated by a 0.6 Hz sine / cosine.
 *
 * Damping shelf (both variants): lp <- p lp + (1 - p) y, D(y) = g_hf y + (g - g_hf) lp. The DC gain
 * is g = 10^(-3 L / (RT60 fs)) and the high-frequency gain g_hf the same with RT60 * hf_ratio
 * (Jot's absorption condition: every line loses the same dB per second); p = exp(-2 pi hf / fs).
 * Gains come from pcl_db_to_gain, so fixed point needs no float.
 *
 * Fixed point: values are Q4.28 (pcl_wide_t), coefficients Q2.30. Every product in a recursive path
 * rounds toward zero (magnitude truncation) and 16-bit line writes round stochastically (unbiased,
 * and 0 stays 0), so with no input every state reaches exactly 0: no limit cycles. Lines saturate
 * at full scale; native Q31 lines hold Q4.28 << 3 exactly.
 */
#include "pcl/pcl_reverb.h"

#include <string.h>

#include "core/pcl_delayline.h"
#include "pcl/pcl_math.h"

PCL_STATIC_ASSERT(
    (int)PCL_DELAY_STORE_NATIVE == PCL_DL_NATIVE && (int)PCL_DELAY_STORE_Q15 == PCL_DL_Q15,
    reverb_storage_values);
PCL_STATIC_ASSERT(PCL_REVERB_COMBS <= PCL_REVERB_TANK_LINES, reverb_comb_count);

#define RV_US_PER_S 1000000u
#define RV_MS_PER_S 1000u
#define RV_SIZE_ONE_Q16 65536u
#define RV_HF_MAX_PERCENT 45u
#define RV_MIN_LINE 2u
#define RV_PRIME_SEARCH 4096u /* candidates tried per line: prime gaps below 2^20 are < 120 */

/* Line lengths at size 1, in microseconds. */
static const uint32_t rv_comb_us[PCL_REVERB_COMBS] = { 25306u, 26939u, 28957u, 30748u };
static const uint32_t rv_small_ap_us[PCL_REVERB_ALLPASSES] = { 5000u, 1700u, 5300u, 1900u };
static const uint32_t rv_fdn_us[PCL_REVERB_TANK_LINES]
    = { 31700u, 37100u, 41900u, 47300u, 53900u, 61100u, 71300u, 83900u };
static const uint32_t rv_diff_us[PCL_REVERB_ALLPASSES] = { 4770u, 3600u, 12730u, 9310u };

/* QUALITY lines with modulated length, and the modulation: +-8 samples at 48 kHz, 0.6 Hz. */
#define RV_MOD_LINE_A 3u
#define RV_MOD_LINE_B 6u
static const uint8_t rv_mod_line[PCL_REVERB_MOD_LINES] = { RV_MOD_LINE_A, RV_MOD_LINE_B };
#define RV_MOD_SAMPLES_48K 8u
#define RV_MOD_REF_HZ 48000u
#define RV_LFO_MILLIHZ 600u

/* -20 log10(e) * 2 pi: the pole exp(-2 pi f / fs) is db_to_gain(-RV_DB_PER_CYCLE * f / fs). */
#define RV_DB_PER_CYCLE 54.575136
#define RV_DB_FULL 60.0 /* RT60 */
#define RV_INV_SQRT8 0.35355339059327373

/*
 * Levels. The tanks run at half the output level (RV_WET_GAIN restores it outside the loops), for
 * headroom: lines saturate at full scale in fixed point, and an all-pass state can reach 1 / (1 -
 * g) times its input. QUALITY's input gain comes before the diffusers for the same reason. The two
 * variants give the same tail energy at equal RT60.
 */
#define RV_SMALL_IN 0.1
#define RV_SMALL_AP_G 0.7
#define RV_QUALITY_IN 0.165
#define RV_WET_GAIN 2
#define RV_IN_NEG 0x96u /* lines injected with -u: Hadamard row 7 (balances L and R)     */

/*
 * 16-bit line writes: stochastic rounding (add a uniform fraction of an LSB, then floor), except
 * inside a dead zone of +-RV_DITHER_DEADZONE LSB, which truncates toward zero.
 *
 * Stochastic rounding is unbiased, so the decay time does not depend on the level: truncation
 * everywhere shortened a 3 s QUALITY tail to 2.3 s at 16 bits, and rounding to nearest left limit
 * cycles of +-4 LSB. On its own it never lets the FDN fall silent: the matrix spreads a 1-LSB value
 * over 8 lines at 0.35 LSB each, and stochastic rounding regrows about 2.8 of them (a supercritical
 * branching process, measured as a +-3 LSB floor). Truncating values below 2 LSB stops that: 3 s
 * then measures 2.98 s and every tail ends at exactly 0.
 *
 * The dither is a 32-bit LCG (Numerical Recipes constants), one per line in the state:
 * deterministic and block-size independent. Its top RV_DITHER_BITS bits are the fraction (the bits
 * Q4.28 has below Q15).
 */
#define RV_DITHER_DEADZONE 2 /* LSB of Q15 */
#define RV_DITHER_BITS 13
#define RV_LCG_MUL 1664525u
#define RV_LCG_ADD 1013904223u
#define RV_RNG_SEED 0x2545F491u
#define RV_RNG_STRIDE 0x9E3779B9u

PCL_FORCE_INLINE uint32_t rv_dither(uint32_t *rng)
{
    *rng = *rng * RV_LCG_MUL + RV_LCG_ADD;
    return *rng >> (32 - RV_DITHER_BITS);
}

#if PCL_SAMPLE_TYPE == PCL_FLOAT

typedef float rv_val_t;

#define RV_DENORMAL_GUARD_F 1.0e-24f /* as the filters: -480 dBFS on every recursive state     */
#define RV_LINE_LIMIT_F 4.0f         /* float lines clamp here (and store NaN as 0)            */

static const float rv_diff_g[PCL_REVERB_ALLPASSES] = { 0.75f, 0.75f, 0.625f, 0.625f };

PCL_FORCE_INLINE rv_val_t rv_mul(rv_val_t v, pcl_coef_t c) { return v * c; }

PCL_FORCE_INLINE rv_val_t rv_mac2(pcl_coef_t a, rv_val_t x, pcl_coef_t b, rv_val_t y)
{
    return a * x + b * y;
}

PCL_FORCE_INLINE rv_val_t rv_clip(rv_val_t v)
{
    if (v > RV_LINE_LIMIT_F)
        return RV_LINE_LIMIT_F;
    if (v < -RV_LINE_LIMIT_F)
        return -RV_LINE_LIMIT_F;
    if (v != v)
        return 0.0f;
    return v;
}

PCL_FORCE_INLINE rv_val_t rv_load(const void *buf, unsigned storage, uint32_t idx)
{
    if (storage == PCL_DL_Q15)
        return (float)((const int16_t *)buf)[idx] * (1.0f / PCL_DL_Q15_SCALE);
    return ((const float *)buf)[idx];
}

/* 16-bit writes round as in the fixed-point builds (see rv_dither). */
PCL_FORCE_INLINE void rv_store(void *buf, unsigned storage, uint32_t idx, rv_val_t v, uint32_t *rng)
{
    if (storage == PCL_DL_Q15)
    {
        const float r = (float)rv_dither(rng) * (1.0f / (float)(1u << RV_DITHER_BITS));
        float       s = v * PCL_DL_Q15_SCALE;
        int16_t     q;
        if (s >= 32767.0f)
            q = INT16_MAX;
        else if (s <= -32768.0f)
            q = INT16_MIN;
        else if (s < (float)RV_DITHER_DEADZONE && s > -(float)RV_DITHER_DEADZONE)
            q = (int16_t)s; /* toward zero */
        else if (s == s)
        {
            s += r;
            q = (int16_t)s; /* toward zero; make it floor */
            if ((float)q > s)
                q--;
        }
        else
            q = 0;
        ((int16_t *)buf)[idx] = q;
        return;
    }
    (void)rng;
    ((float *)buf)[idx] = rv_clip(v + RV_DENORMAL_GUARD_F);
}

PCL_FORCE_INLINE rv_val_t rv_lerp(rv_val_t p1, rv_val_t p2, uint32_t f)
{
    return p1 + (p2 - p1) * ((float)f * (1.0f / (float)PCL_DL_ONE));
}

PCL_FORCE_INLINE rv_val_t rv_in(pcl_sample_t x) { return x; }

PCL_FORCE_INLINE pcl_sample_t rv_mix_out(pcl_sample_t x, float wet, pcl_coef_t mix)
{
    return x + mix * (wet - x);
}

static int rv_coef_in(pcl_coef_t c, float lo, float hi) { return c >= lo && c <= hi; }

static int rv_ms_valid(pcl_ms_t ms, float lo, float hi) { return ms >= lo && ms <= hi; }

static int rv_hf_valid(pcl_hz_t f, uint32_t fs)
{
    return f >= (float)PCL_REVERB_HF_MIN_HZ && f <= (float)RV_HF_MAX_PERCENT / 100.0f * (float)fs;
}

static uint32_t rv_size_q16(pcl_coef_t size)
{
    return (uint32_t)(size * (float)RV_SIZE_ONE_Q16 + 0.5f);
}

static uint32_t rv_ms_to_samples(pcl_ms_t ms, uint32_t fs)
{
    return (uint32_t)(ms * (float)fs / (float)RV_MS_PER_S + 0.5f);
}

/* Level change over `len` samples for a decay of 60 dB in `rt_ms` (rt_ms * f, f = 1 or hf_ratio).
 */
static pcl_db_t rv_decay_db(uint32_t len, pcl_ms_t rt_ms, pcl_coef_t f, uint32_t fs)
{
    return -(float)RV_DB_FULL * (float)RV_MS_PER_S * (float)len / (rt_ms * f * (float)fs);
}

static pcl_coef_t rv_pole(pcl_hz_t hf, uint32_t fs)
{
    return pcl_db_to_gain(-(float)RV_DB_PER_CYCLE * hf / (float)fs);
}

static pcl_coef_t rv_inv_sqrt8(pcl_coef_t g)
{
    /* Rounded below 1/sqrt(8), so the matrix never gains. */
    return g * 0.35355335f;
}

#define RV_COEF_ONE 1.0f
#define RV_HALF(c) (0.5f * (c))
#define RV_WIDE(v) (v) /* a sum that rv_store / rv_clip saturate */

#else /* fixed point */

typedef int32_t rv_val_t; /* Q4.28 */

#define RV_ONE28 ((int32_t)1 << PCL_Q28_SHIFT)
#define RV_Q15_TO_Q28 (PCL_Q28_SHIFT - PCL_Q15_SHIFT)
#define RV_Q31_TO_Q28 (PCL_Q31_SHIFT - PCL_Q28_SHIFT)
#define RV_HZ_FRAC_BITS 16

static const int32_t rv_diff_g[PCL_REVERB_ALLPASSES]
    = { PCL_CONST_Q30(0.75), PCL_CONST_Q30(0.75), PCL_CONST_Q30(0.625), PCL_CONST_Q30(0.625) };

/*
 * v >> s rounded toward zero (magnitude truncation), without a branch on the sign (audio signs are
 * unpredictable): the arithmetic shift floors, so a negative v first gets 2^s - 1 added.
 */
PCL_FORCE_INLINE int64_t rv_trunc(int64_t v, unsigned s)
{
    return (v + ((v >> 63) & (((int64_t)1 << s) - 1))) >> s;
}

PCL_FORCE_INLINE rv_val_t rv_clip(int64_t v)
{
    return (rv_val_t)(v >= RV_ONE28 ? RV_ONE28 - 1 : (v < -RV_ONE28 ? -RV_ONE28 : v));
}

/* Q4.28 * Q2.30, toward zero. Unsaturated: callers keep |result| < 8. */
PCL_FORCE_INLINE rv_val_t rv_mul(rv_val_t v, pcl_coef_t c)
{
    return (rv_val_t)rv_trunc((int64_t)v * c, PCL_Q30_SHIFT);
}

PCL_FORCE_INLINE rv_val_t rv_mac2(pcl_coef_t a, rv_val_t x, pcl_coef_t b, rv_val_t y)
{
    return (rv_val_t)rv_trunc((int64_t)a * x + (int64_t)b * y, PCL_Q30_SHIFT);
}

PCL_FORCE_INLINE rv_val_t rv_load(const void *buf, unsigned storage, uint32_t idx)
{
#if PCL_SAMPLE_TYPE == PCL_Q15
    (void)storage;
    return (rv_val_t)((const int16_t *)buf)[idx] * (1 << RV_Q15_TO_Q28);
#else
    if (storage == PCL_DL_Q15)
        return (rv_val_t)((const int16_t *)buf)[idx] * (1 << RV_Q15_TO_Q28);
    return ((const int32_t *)buf)[idx] >> RV_Q31_TO_Q28; /* exact: stored as Q4.28 << 3 */
#endif
}

/* Q4.28 -> Q15: stochastic rounding, truncation inside the dead zone (see rv_dither). */
PCL_FORCE_INLINE int16_t rv_to_q15(rv_val_t c, uint32_t *rng)
{
    const rv_val_t r = (rv_val_t)rv_dither(rng);
    const rv_val_t z = (rv_val_t)RV_DITHER_DEADZONE << RV_Q15_TO_Q28;
    const rv_val_t toward_zero = (c >> 31) & ((1 << RV_Q15_TO_Q28) - 1);
    /* |c| < z, as one unsigned comparison; then floor(c + bias). */
    const rv_val_t bias = (uint32_t)(c + z - 1) < (uint32_t)(2 * z - 1) ? toward_zero : r;

    return pcl_sat_q15((c + bias) >> RV_Q15_TO_Q28);
}

PCL_FORCE_INLINE void rv_store(void *buf, unsigned storage, uint32_t idx, int64_t v, uint32_t *rng)
{
    const rv_val_t c = rv_clip(v);
#if PCL_SAMPLE_TYPE == PCL_Q15
    (void)storage;
    ((int16_t *)buf)[idx] = rv_to_q15(c, rng);
#else
    if (storage == PCL_DL_Q15)
        ((int16_t *)buf)[idx] = rv_to_q15(c, rng);
    else
    {
        (void)rng;
        ((int32_t *)buf)[idx] = (int32_t)((uint32_t)c << RV_Q31_TO_Q28);
    }
#endif
}

/* Between two line values; floor rounding keeps the result within [min, max] of p1, p2. */
PCL_FORCE_INLINE rv_val_t rv_lerp(rv_val_t p1, rv_val_t p2, uint32_t f)
{
    return (rv_val_t)(p1 + ((((int64_t)p2 - p1) * (int64_t)f) >> PCL_DL_FRAC_BITS));
}

PCL_FORCE_INLINE rv_val_t rv_in(pcl_sample_t x)
{
#if PCL_SAMPLE_TYPE == PCL_Q15
    return (rv_val_t)x * (1 << RV_Q15_TO_Q28);
#else
    return (rv_val_t)(x >> RV_Q31_TO_Q28); /* floor; outside the loops, so no decay concern */
#endif
}

/* A Q4.28 value (|v| < 2^34) as a sample, rounded and saturated. */
PCL_FORCE_INLINE pcl_sample_t rv_wet_sample(int64_t v)
{
#if PCL_SAMPLE_TYPE == PCL_Q15
    return pcl_sat_q15((int32_t)pcl_sat_q31((v + (1 << (RV_Q15_TO_Q28 - 1))) >> RV_Q15_TO_Q28));
#else
    return pcl_sat_q31(v * (1 << RV_Q31_TO_Q28));
#endif
}

PCL_FORCE_INLINE pcl_sample_t rv_mix_out(pcl_sample_t x, int64_t wet, pcl_coef_t mix)
{
    const pcl_sample_t ws = rv_wet_sample(wet);
    const int64_t      d
        = ((int64_t)mix * ((int64_t)ws - x) + ((int64_t)1 << (PCL_Q30_SHIFT - 1))) >> PCL_Q30_SHIFT;
#if PCL_SAMPLE_TYPE == PCL_Q15
    return pcl_sat_q15((int32_t)(x + d));
#else
    return pcl_sat_q31((int64_t)x + d);
#endif
}

static int rv_coef_in(pcl_coef_t c, int32_t lo, int32_t hi) { return c >= lo && c <= hi; }

static int rv_ms_valid(pcl_ms_t ms, uint32_t lo, uint32_t hi) { return ms >= lo && ms <= hi; }

static int rv_hf_valid(pcl_hz_t f, uint32_t fs)
{
    return f >= PCL_HZ(PCL_REVERB_HF_MIN_HZ)
        && (uint64_t)f * 100u <= ((uint64_t)fs * RV_HF_MAX_PERCENT) << RV_HZ_FRAC_BITS;
}

static uint32_t rv_size_q16(pcl_coef_t size)
{
    return (uint32_t)((size + (1 << (PCL_Q30_SHIFT - 17))) >> (PCL_Q30_SHIFT - 16));
}

/* Q16.16 ms -> samples, rounded. */
static uint32_t rv_ms_to_samples(pcl_ms_t ms, uint32_t fs)
{
    const uint64_t den = (uint64_t)RV_MS_PER_S << RV_HZ_FRAC_BITS;
    return (uint32_t)(((uint64_t)ms * fs + den / 2u) / den);
}

/* -60 dB * len / (rt_ms * f * fs / 1000), Q8.24, floored at -127 dB (a gain of 0). */
static pcl_db_t rv_decay_db(uint32_t len, pcl_ms_t rt_ms, pcl_coef_t f, uint32_t fs)
{
    const uint64_t rt_q16 = ((uint64_t)rt_ms * (uint32_t)f) >> PCL_Q30_SHIFT;
    const uint64_t den = (rt_q16 * fs) >> RV_HZ_FRAC_BITS; /* ms * Hz, >= 655 * 8000 */
    const uint64_t num = ((uint64_t)(RV_DB_FULL * RV_MS_PER_S) << PCL_Q24_SHIFT) * len;
    uint64_t       q = (num + den / 2u) / den;

    if (q > ((uint64_t)127u << PCL_Q24_SHIFT))
        q = (uint64_t)127u << PCL_Q24_SHIFT;
    return -(pcl_db_t)q;
}

static pcl_coef_t rv_pole(pcl_hz_t hf, uint32_t fs)
{
    const uint64_t k = (uint64_t)(RV_DB_PER_CYCLE * (double)(1u << PCL_Q24_SHIFT) + 0.5);
    const uint64_t inc = ((uint64_t)hf << (32 - RV_HZ_FRAC_BITS)) / fs; /* f / fs, Q0.32 */
    return pcl_db_to_gain(-(pcl_db_t)((k * inc + ((uint64_t)1 << 31)) >> 32));
}

static pcl_coef_t rv_inv_sqrt8(pcl_coef_t g)
{
    /* Floored, so the matrix never gains. */
    const int64_t k = (int64_t)(RV_INV_SQRT8 * (double)(1u << PCL_Q30_SHIFT));
    return (pcl_coef_t)(((int64_t)g * k) >> PCL_Q30_SHIFT);
}

#define RV_COEF_ONE ((pcl_coef_t)1 << PCL_Q30_SHIFT)
#define RV_HALF(c) ((c) >> 1)
#define RV_WIDE(v) ((int64_t)(v)) /* a sum that rv_store / rv_clip saturate */

#endif

/* ---- Layout: line lengths and the buffer ---- */

typedef struct
{
    uint32_t pre;                          /* predelay line length                      */
    uint32_t delay[PCL_REVERB_TANK_LINES]; /* nominal tank delays                       */
    uint32_t tank[PCL_REVERB_TANK_LINES];  /* tank line lengths (modulated lines longer) */
    uint32_t ap[PCL_REVERB_ALLPASSES];     /* all-pass lengths                          */
    uint32_t mod_max;                      /* excursion at depth 1, Q20.12              */
    unsigned n_tank;
} rv_layout_t;

static int rv_is_prime(uint32_t n)
{
    uint32_t d;

    if (n < 2u)
        return 0;
    if (n % 2u == 0u)
        return n == 2u;
    for (d = 3u; d * d <= n; d += 2u)
        if (n % d == 0u)
            return 0;
    return 1;
}

/* The smallest prime >= n not already in used[0 .. count). */
static uint32_t rv_next_prime(uint32_t n, const uint32_t *used, unsigned count)
{
    uint32_t k;

    if (n < RV_MIN_LINE)
        n = RV_MIN_LINE;
    for (k = 0; k < RV_PRIME_SEARCH; k++, n++)
    {
        unsigned j;
        int      taken = 0;
        for (j = 0; j < count; j++)
            taken |= used[j] == n;
        if (!taken && rv_is_prime(n))
            return n;
    }
    return n; /* unreachable for lengths below 2^20 */
}

static uint32_t rv_scaled_samples(uint32_t us, uint32_t size_q16, uint32_t fs)
{
    const uint64_t v = (uint64_t)us * fs * size_q16; /* < 2^53 */
    return (uint32_t)((v + ((uint64_t)RV_US_PER_S << 15)) / ((uint64_t)RV_US_PER_S << 16));
}

static int rv_is_mod_line(unsigned i)
{
    unsigned k;
    for (k = 0; k < PCL_REVERB_MOD_LINES; k++)
        if (rv_mod_line[k] == i)
            return 1;
    return 0;
}

static void rv_make_layout(const pcl_reverb_config_t *cfg, rv_layout_t *lay)
{
    const uint32_t  fs = cfg->sample_rate_hz;
    const uint32_t  sq = rv_size_q16(cfg->size);
    const int       quality = cfg->variant == PCL_REVERB_QUALITY;
    const uint32_t *tank_us = quality ? rv_fdn_us : rv_comb_us;
    const uint32_t *ap_us = quality ? rv_diff_us : rv_small_ap_us;
    uint32_t        used[PCL_REVERB_TANK_LINES + PCL_REVERB_ALLPASSES];
    unsigned        count = 0, i;
    const uint32_t  mod_ceil
        = (RV_MOD_SAMPLES_48K * fs + RV_MOD_REF_HZ - 1u) / RV_MOD_REF_HZ; /* whole samples */

    memset(lay, 0, sizeof *lay);
    lay->n_tank = quality ? PCL_REVERB_TANK_LINES : PCL_REVERB_COMBS;
    lay->pre
        = (uint32_t)(((uint64_t)cfg->max_predelay_ms * fs + RV_MS_PER_S - 1u) / RV_MS_PER_S) + 1u;
    lay->mod_max = (uint32_t)(((uint64_t)RV_MOD_SAMPLES_48K * fs * PCL_DL_ONE) / RV_MOD_REF_HZ);
    for (i = 0; i < lay->n_tank; i++)
    {
        lay->delay[i] = rv_next_prime(rv_scaled_samples(tank_us[i], sq, fs), used, count);
        used[count++] = lay->delay[i];
        lay->tank[i] = lay->delay[i] + (quality && rv_is_mod_line(i) ? mod_ceil + 2u : 0u);
    }
    for (i = 0; i < PCL_REVERB_ALLPASSES; i++)
    {
        lay->ap[i] = rv_next_prime(rv_scaled_samples(ap_us[i], sq, fs), used, count);
        used[count++] = lay->ap[i];
    }
}

static uint64_t rv_layout_samples(const rv_layout_t *lay)
{
    uint64_t total = lay->pre;
    unsigned i;
    for (i = 0; i < lay->n_tank; i++)
        total += lay->tank[i];
    for (i = 0; i < PCL_REVERB_ALLPASSES; i++)
        total += lay->ap[i];
    return total;
}

/* ---- Validation ---- */

#if PCL_SAMPLE_TYPE == PCL_FLOAT
#define RV_COEF_LIMIT(x) ((float)(x))
#define RV_MS_LIMIT(x) ((float)(x))
#else
#define RV_COEF_LIMIT(x) PCL_COEF(x)
#define RV_MS_LIMIT(x) PCL_MS(x)
#endif

static int rv_predelay_valid(pcl_ms_t ms, uint32_t max_ms)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return ms >= 0.0f && ms <= (float)max_ms;
#else
    return (uint64_t)ms <= ((uint64_t)max_ms << RV_HZ_FRAC_BITS);
#endif
}

static int rv_rt60_valid(pcl_ms_t ms)
{
    return rv_ms_valid(
        ms, RV_MS_LIMIT(PCL_REVERB_RT60_MIN_MS), RV_MS_LIMIT(PCL_REVERB_RT60_MAX_MS));
}

static int rv_hf_ratio_valid(pcl_coef_t r)
{
    return rv_coef_in(r, RV_COEF_LIMIT(PCL_REVERB_HF_RATIO_MIN), RV_COEF_LIMIT(1.0));
}

static int rv_unit_valid(pcl_coef_t c)
{
    return rv_coef_in(c, RV_COEF_LIMIT(0.0), RV_COEF_LIMIT(1.0));
}

static int rv_cfg_valid(const pcl_reverb_config_t *cfg)
{
    return cfg->sample_rate_hz >= PCL_SAMPLE_RATE_MIN_HZ
        && cfg->sample_rate_hz <= PCL_SAMPLE_RATE_MAX_HZ
        && (unsigned)cfg->variant < (unsigned)PCL_REVERB_VARIANT_COUNT
        && rv_coef_in(cfg->size, RV_COEF_LIMIT(PCL_REVERB_SIZE_MIN), RV_COEF_LIMIT(1.0))
        && cfg->max_predelay_ms <= PCL_REVERB_MAX_PREDELAY_MS
        && rv_predelay_valid(cfg->predelay_ms, cfg->max_predelay_ms) && rv_rt60_valid(cfg->rt60_ms)
        && rv_hf_ratio_valid(cfg->hf_ratio) && rv_hf_valid(cfg->hf_hz, cfg->sample_rate_hz)
        && rv_unit_valid(cfg->width) && rv_unit_valid(cfg->mix) && rv_unit_valid(cfg->mod_depth)
        && (unsigned)cfg->storage < (unsigned)PCL_DELAY_STORE_COUNT;
}

size_t pcl_reverb_required_bytes(const pcl_reverb_config_t *cfg)
{
    rv_layout_t lay;
    size_t      elem;

    if (cfg == NULL || !rv_cfg_valid(cfg))
        return 0;
    rv_make_layout(cfg, &lay);
    elem = pcl_dl_elem_bytes((unsigned)cfg->storage);
    return (size_t)rv_layout_samples(&lay) * elem + (elem - 1u);
}

/* ---- Coefficients ---- */

static pcl_coef_t rv_quality_in_gain(void) { return RV_COEF_LIMIT(RV_QUALITY_IN); }
static pcl_coef_t rv_small_in_gain(void) { return RV_COEF_LIMIT(RV_SMALL_IN); }

static pcl_coef_t rv_input_target(const pcl_reverb_t *rv)
{
    if (rv->freeze)
        return 0;
    return rv->variant == PCL_REVERB_QUALITY ? rv_quality_in_gain() : rv_small_in_gain();
}

static void rv_update_gains(pcl_reverb_t *rv)
{
    const unsigned n = rv->variant == PCL_REVERB_QUALITY ? PCL_REVERB_TANK_LINES : PCL_REVERB_COMBS;
    unsigned       i;

    for (i = 0; i < n; i++)
    {
        pcl_coef_t g, gh;
        if (rv->freeze)
        {
            g = RV_COEF_ONE;
            gh = RV_COEF_ONE;
        }
        else
        {
            g = pcl_db_to_gain(
                rv_decay_db(rv->delay[i], rv->rt60_ms, RV_COEF_ONE, rv->sample_rate_hz));
            gh = pcl_db_to_gain(
                rv_decay_db(rv->delay[i], rv->rt60_ms, rv->hf_ratio, rv->sample_rate_hz));
            if (gh > g)
                gh = g;
        }
        if (rv->variant == PCL_REVERB_QUALITY)
        {
            g = rv_inv_sqrt8(g);
            gh = rv_inv_sqrt8(gh);
        }
        rv->g_hf[i] = gh;
        rv->g_lf[i] = g - gh;
    }
}

static void rv_set_mod_amp(pcl_reverb_t *rv, pcl_coef_t depth)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    rv->mod_amp = (uint32_t)((float)rv->mod_max * depth + 0.5f);
#else
    rv->mod_amp = (uint32_t)(((uint64_t)rv->mod_max * (uint32_t)depth) >> PCL_Q30_SHIFT);
#endif
}

pcl_status_t pcl_reverb_init(
    pcl_reverb_t *rv, const pcl_reverb_config_t *cfg, void *buffer, size_t bytes)
{
    pcl_reverb_t   tmp;
    rv_layout_t    lay;
    size_t         elem;
    unsigned char *p;
    unsigned       i;
    uint32_t       ramp;

    if (rv == NULL || cfg == NULL || buffer == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!rv_cfg_valid(cfg))
        return PCL_ERR_OUT_OF_RANGE;
    if (bytes < pcl_reverb_required_bytes(cfg))
        return PCL_ERR_BUFFER_TOO_SMALL;

    memset(&tmp, 0, sizeof tmp);
    rv_make_layout(cfg, &lay);
    elem = pcl_dl_elem_bytes((unsigned)cfg->storage);
    p = (unsigned char *)buffer + (elem - (size_t)((uintptr_t)buffer % elem)) % elem;
    tmp.pre.buf = p;
    tmp.pre.length = lay.pre;
    p += (size_t)lay.pre * elem;
    for (i = 0; i < lay.n_tank; i++)
    {
        tmp.tank[i].buf = p;
        tmp.tank[i].length = lay.tank[i];
        tmp.delay[i] = lay.delay[i];
        p += (size_t)lay.tank[i] * elem;
    }
    for (i = 0; i < PCL_REVERB_ALLPASSES; i++)
    {
        tmp.ap[i].buf = p;
        tmp.ap[i].length = lay.ap[i];
        p += (size_t)lay.ap[i] * elem;
    }
    tmp.variant = (uint8_t)cfg->variant;
    tmp.storage = (uint8_t)cfg->storage;
    tmp.sample_rate_hz = cfg->sample_rate_hz;
    tmp.max_predelay_ms = cfg->max_predelay_ms;
    tmp.pre_d = rv_ms_to_samples(cfg->predelay_ms, cfg->sample_rate_hz);
    if (tmp.pre_d > lay.pre - 1u)
        tmp.pre_d = lay.pre - 1u;
    tmp.rt60_ms = cfg->rt60_ms;
    tmp.hf_ratio = cfg->hf_ratio;
    tmp.lp_pole = rv_pole(cfg->hf_hz, cfg->sample_rate_hz);
    tmp.freeze = cfg->freeze != 0;
    tmp.mod_max = lay.mod_max;
    rv_set_mod_amp(&tmp, cfg->mod_depth);
    tmp.lfo_inc = (uint32_t)(((uint64_t)RV_LFO_MILLIHZ * PCL_CONTROL_DIV << 32)
        / ((uint64_t)RV_MS_PER_S * cfg->sample_rate_hz));
    rv_update_gains(&tmp);
    ramp = pcl_smooth_ramp_samples(cfg->sample_rate_hz);
    pcl_smooth_init(&tmp.mix, cfg->mix, ramp);
    pcl_smooth_init(&tmp.width, cfg->width, ramp);
    pcl_smooth_init(&tmp.input, rv_input_target(&tmp), ramp);
    pcl_reverb_reset(&tmp);
    *rv = tmp;
    return PCL_OK;
}

static void rv_clear_line(pcl_reverb_line_t *l, unsigned storage, uint32_t index)
{
    if (l->buf != NULL)
        memset(l->buf, 0, (size_t)l->length * pcl_dl_elem_bytes(storage));
    l->w = 0u;
    l->rng = RV_RNG_SEED + index * RV_RNG_STRIDE; /* a different phase of the LCG per line */
}

void pcl_reverb_reset(pcl_reverb_t *rv)
{
    unsigned i;

    rv_clear_line(&rv->pre, rv->storage, 0u);
    for (i = 0; i < PCL_REVERB_TANK_LINES; i++)
    {
        rv_clear_line(&rv->tank[i], rv->storage, 1u + i);
        rv->lp[i] = 0;
    }
    for (i = 0; i < PCL_REVERB_ALLPASSES; i++)
        rv_clear_line(&rv->ap[i], rv->storage, 1u + PCL_REVERB_TANK_LINES + i);
    for (i = 0; i < PCL_REVERB_MOD_LINES; i++)
    {
        rv->mod_d[i] = rv->delay[rv_mod_line[i]] << PCL_DL_FRAC_BITS;
        rv->mod_d_end[i] = rv->mod_d[i];
        rv->mod_d_step[i] = 0;
    }
    rv->lfo_phase = 0u;
    rv->pos = 0u;
    pcl_smooth_reset(&rv->mix);
    pcl_smooth_reset(&rv->width);
    pcl_smooth_reset(&rv->input);
}

/* ---- Setters ---- */

pcl_status_t pcl_reverb_set_predelay(pcl_reverb_t *rv, pcl_ms_t predelay_ms)
{
    if (rv == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!rv_predelay_valid(predelay_ms, rv->max_predelay_ms))
        return PCL_ERR_OUT_OF_RANGE;
    rv->pre_d = rv_ms_to_samples(predelay_ms, rv->sample_rate_hz);
    if (rv->pre_d > rv->pre.length - 1u)
        rv->pre_d = rv->pre.length - 1u;
    return PCL_OK;
}

pcl_status_t pcl_reverb_set_rt60(pcl_reverb_t *rv, pcl_ms_t rt60_ms)
{
    if (rv == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!rv_rt60_valid(rt60_ms))
        return PCL_ERR_OUT_OF_RANGE;
    rv->rt60_ms = rt60_ms;
    rv_update_gains(rv);
    return PCL_OK;
}

pcl_status_t pcl_reverb_set_damping(pcl_reverb_t *rv, pcl_coef_t hf_ratio, pcl_hz_t hf_hz)
{
    if (rv == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!rv_hf_ratio_valid(hf_ratio) || !rv_hf_valid(hf_hz, rv->sample_rate_hz))
        return PCL_ERR_OUT_OF_RANGE;
    rv->hf_ratio = hf_ratio;
    rv->lp_pole = rv_pole(hf_hz, rv->sample_rate_hz);
    rv_update_gains(rv);
    return PCL_OK;
}

pcl_status_t pcl_reverb_set_width(pcl_reverb_t *rv, pcl_coef_t width)
{
    if (rv == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!rv_unit_valid(width))
        return PCL_ERR_OUT_OF_RANGE;
    pcl_smooth_set_target(&rv->width, width);
    return PCL_OK;
}

pcl_status_t pcl_reverb_set_mix(pcl_reverb_t *rv, pcl_coef_t mix)
{
    if (rv == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!rv_unit_valid(mix))
        return PCL_ERR_OUT_OF_RANGE;
    pcl_smooth_set_target(&rv->mix, mix);
    return PCL_OK;
}

pcl_status_t pcl_reverb_set_mod_depth(pcl_reverb_t *rv, pcl_coef_t mod_depth)
{
    if (rv == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!rv_unit_valid(mod_depth))
        return PCL_ERR_OUT_OF_RANGE;
    rv_set_mod_amp(rv, mod_depth);
    return PCL_OK;
}

void pcl_reverb_set_freeze(pcl_reverb_t *rv, int freeze)
{
    rv->freeze = freeze != 0;
    rv_update_gains(rv);
    pcl_smooth_set_target(&rv->input, rv_input_target(rv));
}

/* ---- Processing ----
 *
 * Block-wise: each line runs over up to RV_BLOCK samples in its own loop, with its state in locals.
 * That is exact because every tank line is longer than a block, so a block's reads only see
 * samples written before the block (the all-passes, which may be shorter, run sample by sample in
 * their own loops). Each line has its own dither generator, so the loop order does not matter.
 */
#define RV_BLOCK 16u

/* The shortest tank line: SMALL's first comb at size 0.25 and 8 kHz, 50 samples (QUALITY: 63). */
PCL_STATIC_ASSERT(
    (25306u * (PCL_SAMPLE_RATE_MIN_HZ / 4u)) / RV_US_PER_S >= RV_BLOCK + RV_MOD_SAMPLES_48K + 2u,
    reverb_block_shorter_than_lines);

#define RV_NEXT(w, len) ((w) + 1u == (len) ? 0u : (w) + 1u)

/* Predelay and input gain: u[t] = gain * x[t - pre_d]. */
PCL_FORCE_INLINE void rv_pre_block(
    pcl_reverb_t *rv, const pcl_sample_t *in, rv_val_t *u, size_t n, const unsigned storage)
{
    void *const    buf = rv->pre.buf;
    const uint32_t len = rv->pre.length;
    const uint32_t d = rv->pre_d;
    uint32_t       w = rv->pre.w;
    uint32_t       rng = rv->pre.rng;
    pcl_smooth_t   gain = rv->input;
    size_t         t;

    for (t = 0; t < n; t++)
    {
        rv_store(buf, storage, w, rv_in(in[t]), &rng);
        u[t] = rv_mul(rv_load(buf, storage, pcl_dl_back(w, d, len)), pcl_smooth_next(&gain));
        w = RV_NEXT(w, len);
    }
    rv->pre.w = w;
    rv->pre.rng = rng;
    rv->input = gain;
}

/* Schroeder all-pass in place: w = x + g w[n-M], x <- w[n-M] - g w. */
PCL_FORCE_INLINE void rv_allpass_block(
    pcl_reverb_line_t *l, rv_val_t *x, size_t n, pcl_coef_t g, const unsigned storage)
{
    void *const    buf = l->buf;
    const uint32_t len = l->length;
    uint32_t       w = l->w;
    uint32_t       rng = l->rng;
    size_t         t;

    for (t = 0; t < n; t++)
    {
        const rv_val_t wo = rv_load(buf, storage, w);
        const rv_val_t wn = rv_clip(RV_WIDE(x[t]) + rv_mul(wo, g));
        rv_store(buf, storage, w, wn, &rng);
        x[t] = wo - rv_mul(wn, g);
        w = RV_NEXT(w, len);
    }
    l->w = w;
    l->rng = rng;
}

/* Damping shelf: lp <- p lp + q y (q = 1 - p), returns g_hf y + g_lf lp. */
PCL_FORCE_INLINE rv_val_t rv_damp(
    rv_val_t y, rv_val_t *lp, pcl_coef_t p, pcl_coef_t q, pcl_coef_t gh, pcl_coef_t gl)
{
    rv_val_t s = rv_mac2(p, *lp, q, y);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    s += RV_DENORMAL_GUARD_F;
#endif
    *lp = s;
    return rv_mac2(gh, y, gl, s);
}

/*
 * SMALL: comb c over a block, its output added to wl and wr with the signs of L = c0 + c1 - c2 - c3
 * and R = c0 - c1 + c2 - c3 (orthogonal, so the channels are decorrelated).
 */
PCL_FORCE_INLINE void rv_comb_block(pcl_reverb_t *rv, unsigned c, const rv_val_t *u, rv_val_t *wl,
    rv_val_t *wr, size_t n, const unsigned storage)
{
    const rv_val_t     sl = c < 2u ? 1 : -1;
    const rv_val_t     sr = (c & 1u) ? -1 : 1;
    pcl_reverb_line_t *l = &rv->tank[c];
    void *const        buf = l->buf;
    const uint32_t     len = l->length;
    const pcl_coef_t   p = rv->lp_pole, q = RV_COEF_ONE - p;
    const pcl_coef_t   gh = rv->g_hf[c], gl = rv->g_lf[c];
    rv_val_t           lp = rv->lp[c];
    uint32_t           w = l->w;
    uint32_t           rng = l->rng;
    size_t             t;

    for (t = 0; t < n; t++)
    {
        const rv_val_t y = rv_load(buf, storage, w);
        rv_store(buf, storage, w, RV_WIDE(u[t]) + rv_damp(y, &lp, p, q, gh, gl), &rng);
        wl[t] += sl * y;
        wr[t] += sr * y;
        w = RV_NEXT(w, len);
    }
    rv->lp[c] = lp;
    l->w = w;
    l->rng = rng;
}

/* QUALITY: line j's output over a block, through its damping shelf, into column j of h. */
PCL_FORCE_INLINE void rv_tank_read_block(pcl_reverb_t *rv, unsigned j,
    rv_val_t (*h)[PCL_REVERB_TANK_LINES], size_t n, const unsigned storage)
{
    const pcl_reverb_line_t *l = &rv->tank[j];
    const void *const        buf = l->buf;
    const uint32_t           len = l->length;
    const pcl_coef_t         p = rv->lp_pole, q = RV_COEF_ONE - p;
    const pcl_coef_t         gh = rv->g_hf[j], gl = rv->g_lf[j];
    rv_val_t                 lp = rv->lp[j];
    uint32_t                 w = l->w;
    size_t                   t;

    if (j == RV_MOD_LINE_A || j == RV_MOD_LINE_B)
    {
        const unsigned k = j == RV_MOD_LINE_A ? 0u : 1u;
        const int32_t  step = rv->mod_d_step[k];
        uint32_t       d = rv->mod_d[k];
        for (t = 0; t < n; t++)
        {
            uint32_t k1, k2;
            d = (uint32_t)((int64_t)d + step);
            k1 = pcl_dl_back(w, d >> PCL_DL_FRAC_BITS, len);
            k2 = k1 == 0u ? len - 1u : k1 - 1u;
            h[t][j] = rv_damp(
                rv_lerp(rv_load(buf, storage, k1), rv_load(buf, storage, k2), d & PCL_DL_FRAC_MASK),
                &lp, p, q, gh, gl);
            w = RV_NEXT(w, len);
        }
        rv->mod_d[k] = d;
    }
    else
        for (t = 0; t < n; t++)
        {
            h[t][j] = rv_damp(rv_load(buf, storage, w), &lp, p, q, gh, gl);
            w = RV_NEXT(w, len);
        }
    rv->lp[j] = lp;
}

/* QUALITY: line j <- column j of h plus or minus the input. */
PCL_FORCE_INLINE void rv_tank_write_block(pcl_reverb_t *rv, unsigned j,
    rv_val_t (*h)[PCL_REVERB_TANK_LINES], const rv_val_t *u, size_t n, const unsigned storage)
{
    pcl_reverb_line_t *l = &rv->tank[j];
    void *const        buf = l->buf;
    const uint32_t     len = l->length;
    const rv_val_t     sign = ((RV_IN_NEG >> j) & 1u) ? -1 : 1;
    uint32_t           w = l->w;
    uint32_t           rng = l->rng;
    size_t             t;

    for (t = 0; t < n; t++)
    {
        rv_store(buf, storage, w, RV_WIDE(h[t][j]) + sign * u[t], &rng);
        w = RV_NEXT(w, len);
    }
    l->w = w;
    l->rng = rng;
}

/* Fast Walsh-Hadamard transform of 8 values (unnormalized; the gains hold the 1/sqrt(8)). */
PCL_FORCE_INLINE void rv_fwht8(rv_val_t *h)
{
    unsigned s, j;
    for (s = 1u; s < PCL_REVERB_TANK_LINES; s <<= 1)
        for (j = 0; j < PCL_REVERB_TANK_LINES; j++)
            if ((j & s) == 0u)
            {
                const rv_val_t a = h[j], b = h[j + s];
                h[j] = a + b;
                h[j + s] = a - b;
            }
}

/* Width, mix and the outputs over a block. */
PCL_NOINLINE static void rv_output_block(pcl_reverb_t *rv, const pcl_sample_t *in,
    const rv_val_t *wl, const rv_val_t *wr, size_t wstride, pcl_sample_t *out_l,
    pcl_sample_t *out_r, size_t n)
{
    pcl_smooth_t mix_s = rv->mix;
    pcl_smooth_t width_s = rv->width;
    size_t       t;

    for (t = 0; t < n; t++)
    {
        const pcl_sample_t x = in[t];
        const pcl_coef_t   mix = pcl_smooth_next(&mix_s);
        const pcl_coef_t   w = pcl_smooth_next(&width_s);
        const rv_val_t     l0 = wl[t * wstride], r0 = wr[t * wstride];

        if (out_r == NULL)
            out_l[t] = rv_mix_out(x, RV_WIDE(l0) + r0, mix); /* RV_WET_GAIN * (l0 + r0) / 2 */
        else
        {
            /* (1 + w) / 2 and (1 - w) / 2, halved first: 1 + w does not fit Q2.30. */
            const pcl_coef_t a = RV_HALF(RV_COEF_ONE) + RV_HALF(w);
            const pcl_coef_t b = RV_HALF(RV_COEF_ONE) - RV_HALF(w);
            const rv_val_t   l = rv_mac2(a, l0, b, r0);
            const rv_val_t   r = rv_mac2(b, l0, a, r0);
            out_l[t] = rv_mix_out(x, RV_WIDE(l) * RV_WET_GAIN, mix);
            out_r[t] = rv_mix_out(x, RV_WIDE(r) * RV_WET_GAIN, mix);
        }
    }
    rv->mix = mix_s;
    rv->width = width_s;
}

/*
 * The line loops, compiled once per line format with the storage argument constant, and called
 * through a table: one copy of each loop rather than one per call site. In the Q15 build both
 * formats are int16_t, so one table serves both.
 */
typedef struct
{
    void (*pre)(pcl_reverb_t *rv, const pcl_sample_t *in, rv_val_t *u, size_t n);
    void (*allpass)(pcl_reverb_line_t *l, rv_val_t *x, size_t n, pcl_coef_t g);
    void (*comb)(
        pcl_reverb_t *rv, unsigned c, const rv_val_t *u, rv_val_t *wl, rv_val_t *wr, size_t n);
    void (*tank_read)(pcl_reverb_t *rv, unsigned j, rv_val_t (*h)[PCL_REVERB_TANK_LINES], size_t n);
    void (*tank_write)(pcl_reverb_t *rv, unsigned j, rv_val_t (*h)[PCL_REVERB_TANK_LINES],
        const rv_val_t *u, size_t n);
} rv_ops_t;

#define RV_DEFINE_OPS(name, storage)                                                               \
    PCL_NOINLINE static void rv_pre_##name(                                                        \
        pcl_reverb_t *rv, const pcl_sample_t *in, rv_val_t *u, size_t n)                           \
    {                                                                                              \
        rv_pre_block(rv, in, u, n, storage);                                                       \
    }                                                                                              \
    PCL_NOINLINE static void rv_allpass_##name(                                                    \
        pcl_reverb_line_t *l, rv_val_t *x, size_t n, pcl_coef_t g)                                 \
    {                                                                                              \
        rv_allpass_block(l, x, n, g, storage);                                                     \
    }                                                                                              \
    PCL_NOINLINE static void rv_comb_##name(                                                       \
        pcl_reverb_t *rv, unsigned c, const rv_val_t *u, rv_val_t *wl, rv_val_t *wr, size_t n)     \
    {                                                                                              \
        rv_comb_block(rv, c, u, wl, wr, n, storage);                                               \
    }                                                                                              \
    PCL_NOINLINE static void rv_tank_read_##name(                                                  \
        pcl_reverb_t *rv, unsigned j, rv_val_t(*h)[PCL_REVERB_TANK_LINES], size_t n)               \
    {                                                                                              \
        rv_tank_read_block(rv, j, h, n, storage);                                                  \
    }                                                                                              \
    PCL_NOINLINE static void rv_tank_write_##name(pcl_reverb_t *rv, unsigned j,                    \
        rv_val_t(*h)[PCL_REVERB_TANK_LINES], const rv_val_t *u, size_t n)                          \
    {                                                                                              \
        rv_tank_write_block(rv, j, h, u, n, storage);                                              \
    }                                                                                              \
    static const rv_ops_t rv_ops_##name = { rv_pre_##name, rv_allpass_##name, rv_comb_##name,      \
        rv_tank_read_##name, rv_tank_write_##name }

RV_DEFINE_OPS(q15, PCL_DL_Q15);
#if PCL_SAMPLE_TYPE != PCL_Q15
RV_DEFINE_OPS(native, PCL_DL_NATIVE);
#endif

static void rv_small_chunk(const rv_ops_t *ops, pcl_reverb_t *rv, const pcl_sample_t *in,
    pcl_sample_t *out_l, pcl_sample_t *out_r, size_t n)
{
    const pcl_coef_t apg = RV_COEF_LIMIT(RV_SMALL_AP_G);
    rv_val_t         u[RV_BLOCK], wl[RV_BLOCK], wr[RV_BLOCK];
    unsigned         c;
    size_t           t;

    ops->pre(rv, in, u, n);
    for (t = 0; t < n; t++)
        wl[t] = wr[t] = 0;
    for (c = 0; c < PCL_REVERB_COMBS; c++)
        ops->comb(rv, c, u, wl, wr, n);
    for (t = 0; t < n; t++)
    {
        wl[t] = RV_HALF(wl[t]);
        wr[t] = RV_HALF(wr[t]);
    }
    ops->allpass(&rv->ap[0], wl, n, apg);
    ops->allpass(&rv->ap[1], wl, n, apg);
    ops->allpass(&rv->ap[2], wr, n, apg);
    ops->allpass(&rv->ap[3], wr, n, apg);
    rv_output_block(rv, in, wl, wr, 1u, out_l, out_r, n);
}

static void rv_quality_chunk(const rv_ops_t *ops, pcl_reverb_t *rv, const pcl_sample_t *in,
    pcl_sample_t *out_l, pcl_sample_t *out_r, size_t n)
{
    rv_val_t u[RV_BLOCK];
    rv_val_t h[RV_BLOCK][PCL_REVERB_TANK_LINES];
    unsigned j;
    size_t   t;

    ops->pre(rv, in, u, n);
    for (j = 0; j < PCL_REVERB_ALLPASSES; j++)
        ops->allpass(&rv->ap[j], u, n, rv_diff_g[j]);
    for (j = 0; j < PCL_REVERB_TANK_LINES; j++)
        ops->tank_read(rv, j, h, n);
    for (t = 0; t < n; t++)
        rv_fwht8(h[t]);
    for (j = 0; j < PCL_REVERB_TANK_LINES; j++)
        ops->tank_write(rv, j, h, u, n);
    rv_output_block(rv, in, &h[0][1], &h[0][2], PCL_REVERB_TANK_LINES, out_l, out_r, n);
}

static void rv_chunk(
    pcl_reverb_t *rv, const pcl_sample_t *in, pcl_sample_t *out_l, pcl_sample_t *out_r, size_t n)
{
#if PCL_SAMPLE_TYPE == PCL_Q15
    const rv_ops_t *ops = &rv_ops_q15;
#else
    const rv_ops_t *ops = rv->storage == PCL_DL_Q15 ? &rv_ops_q15 : &rv_ops_native;
#endif
    if (rv->variant == PCL_REVERB_QUALITY)
        rv_quality_chunk(ops, rv, in, out_l, out_r, n);
    else
        rv_small_chunk(ops, rv, in, out_l, out_r, n);
}

/* At the start of every control period: advance the LFO and aim the modulated delays. */
static void rv_tick(pcl_reverb_t *rv)
{
    unsigned k;

    /* Frozen: the read positions glide back to whole samples, where interpolation is lossless. */
    const uint32_t amp = rv->freeze ? 0u : rv->mod_amp;

    rv->lfo_phase += rv->lfo_inc;
    for (k = 0; k < PCL_REVERB_MOD_LINES; k++)
    {
        const pcl_sample_t s = k == 0u ? pcl_sin(rv->lfo_phase) : pcl_cos(rv->lfo_phase);
        const int64_t      base = (int64_t)rv->delay[rv_mod_line[k]] << PCL_DL_FRAC_BITS;
#if PCL_SAMPLE_TYPE == PCL_FLOAT
        const int64_t off = (int64_t)((float)amp * s);
#elif PCL_SAMPLE_TYPE == PCL_Q15
        const int64_t off = ((int64_t)amp * s) >> PCL_Q15_SHIFT;
#else
        const int64_t off = ((int64_t)amp * s) >> PCL_Q31_SHIFT;
#endif
        rv->mod_d_end[k] = (uint32_t)(base + off);
        rv->mod_d_step[k] = (int32_t)(((int64_t)rv->mod_d_end[k] - rv->mod_d[k]) / PCL_CONTROL_DIV);
    }
}

void pcl_reverb_process(pcl_reverb_t *rv, const pcl_sample_t *in, pcl_sample_t *out_l,
    pcl_sample_t *out_r, size_t frames)
{
    const int quality = rv->variant == PCL_REVERB_QUALITY;
    size_t    n = 0;

    while (n < frames)
    {
        size_t chunk = (size_t)PCL_CONTROL_DIV - rv->pos;

        if (chunk > RV_BLOCK)
            chunk = RV_BLOCK;
        if (chunk > frames - n)
            chunk = frames - n;
        if (quality && rv->pos == 0u)
            rv_tick(rv);
        rv_chunk(rv, in + n, out_l + n, out_r != NULL ? out_r + n : NULL, chunk);
        rv->pos = (uint8_t)(rv->pos + chunk);
        if (rv->pos >= PCL_CONTROL_DIV)
        {
            unsigned k;
            rv->pos = 0u;
            for (k = 0; k < PCL_REVERB_MOD_LINES; k++)
                rv->mod_d[k] = rv->mod_d_end[k]; /* land exactly */
        }
        n += chunk;
    }
}
