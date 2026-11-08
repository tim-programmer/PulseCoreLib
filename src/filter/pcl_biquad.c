/*
 * pcl_biquad.c - see include/pcl/pcl_biquad.h and docs/modules/biquad.md.
 *
 * Fixed point: the history is Q4.28, the feedforward coefficients Q5.27, the feedback ones Q2.30.
 * All five products go into one 64-bit accumulator (2^-55 units) and the rounding residual of each
 * output is added to the next sample's accumulator (error feedback). Float: plain Direct Form I
 * with the output history clamped to +-8 and an anti-denormal guard.
 */
#include "pcl/pcl_biquad.h"

#include "core/pcl_octave.h"
#include "pcl/pcl_math.h"

#define BQ_REF_HZ 1000 /* the cutoff smoother holds octaves above this, divided by 8        */
#define BQ_OCT_RANGE 8
#define BQ_Q_REF 1.0 /* the Q smoother holds log2(Q) / 8                                    */
#define BQ_GAIN_RANGE_DB 18 /* the gain smoother holds dB / 18                                */

/* f / fs above which a cutoff is refused (and the filter clamps): 0.45 * 2^16, truncated. */
#define BQ_MAX_RATIO_Q16 29491u
#define BQ_MAX_RATIO_F 0.45f

static int bq_type_valid(pcl_biquad_type_t t)
{
    return (unsigned)t <= (unsigned)PCL_BIQUAD_HIGHSHELF;
}

static int bq_uses_gain(unsigned type) { return type >= (unsigned)PCL_BIQUAD_PEAK; }

#if PCL_SAMPLE_TYPE == PCL_FLOAT

#define BQ_LIMIT_F 8.0f
#define BQ_DENORMAL_GUARD_F 1.0e-24f

static float bq_clamp(float v)
{
    if (!(v > -BQ_LIMIT_F)) /* also catches NaN */
        return -BQ_LIMIT_F;
    if (!(v < BQ_LIMIT_F))
        return BQ_LIMIT_F;
    return v;
}

static pcl_coef_t bq_u_from_hz(pcl_hz_t f)
{
    return pcl_hz_to_oct(f, (float)BQ_REF_HZ) * (1.0f / (float)BQ_OCT_RANGE);
}

static pcl_coef_t bq_u_from_q(pcl_quality_t q)
{
    return pcl_hz_to_oct(q, 1.0f) * (1.0f / (float)BQ_OCT_RANGE);
}

static pcl_coef_t bq_u_from_db(pcl_db_t db) { return db * (1.0f / (float)BQ_GAIN_RANGE_DB); }

static int bq_cutoff_valid(const pcl_biquad_t *bq, pcl_hz_t f)
{
    return f >= (float)PCL_BIQUAD_CUTOFF_MIN_HZ && f <= BQ_MAX_RATIO_F * (float)bq->sample_rate_hz;
}

static int bq_q_valid(pcl_quality_t q)
{
    return q >= (float)PCL_BIQUAD_Q_MIN && q <= (float)PCL_BIQUAD_Q_MAX;
}

static int bq_gain_valid(pcl_db_t g)
{
    return g >= -(float)PCL_BIQUAD_GAIN_MAX_DB && g <= (float)PCL_BIQUAD_GAIN_MAX_DB;
}

static int bq_depth_valid(pcl_oct_t d) { return d >= 0.0f && d < 8.0f; }

static pcl_oct_t bq_mod_octaves(const pcl_biquad_t *bq, pcl_sample_t mod)
{
    return mod * bq->mod_depth;
}

/* The per-sample coefficient step that reaches `end` from `cur` in one control period. */
static pcl_biquad_c_t bq_step(pcl_biquad_c_t end, pcl_biquad_c_t cur)
{
    return (end - cur) / (float)PCL_CONTROL_DIV;
}

/* Coefficients at the current smoothed parameters plus `mod_oct` octaves of cutoff. */
static void bq_design(const pcl_biquad_t *bq, pcl_oct_t mod_oct, pcl_biquad_c_t out[5])
{
    const float oct = bq->cutoff.current * (float)BQ_OCT_RANGE + mod_oct;
    const float kk = pcl_tan_hz(pcl_hz_scale((float)BQ_REF_HZ, oct), bq->sample_rate_hz);
    const float k = pcl_hz_scale(1.0f, -bq->q.current * (float)BQ_OCT_RANGE); /* 1 / Q */
    float       a = 1.0f, r = 1.0f, ainv = 1.0f;
    float       n2, n1, n0, d2, d1, d0;
    float       nn0, nn1, nn2, dd0, dd1, dd2, rec;

    if (bq_uses_gain(bq->type))
    {
        const float db = bq->gain.current * (float)BQ_GAIN_RANGE_DB;

        r = pcl_db_to_gain(db * 0.25f); /* 10^(dB/80) = sqrt(A), under 2 for every legal gain */
        ainv = pcl_db_to_gain(-db * 0.25f);
        a = r * r;
        ainv = ainv * ainv;
    }
    switch ((pcl_biquad_type_t)bq->type)
    {
    case PCL_BIQUAD_LP:
        n2 = 0.0f, n1 = 0.0f, n0 = 1.0f, d2 = 1.0f, d1 = k, d0 = 1.0f;
        break;
    case PCL_BIQUAD_HP:
        n2 = 1.0f, n1 = 0.0f, n0 = 0.0f, d2 = 1.0f, d1 = k, d0 = 1.0f;
        break;
    case PCL_BIQUAD_BP:
        n2 = 0.0f, n1 = k, n0 = 0.0f, d2 = 1.0f, d1 = k, d0 = 1.0f;
        break;
    case PCL_BIQUAD_NOTCH:
        n2 = 1.0f, n1 = 0.0f, n0 = 1.0f, d2 = 1.0f, d1 = k, d0 = 1.0f;
        break;
    case PCL_BIQUAD_ALLPASS:
        n2 = 1.0f, n1 = -k, n0 = 1.0f, d2 = 1.0f, d1 = k, d0 = 1.0f;
        break;
    case PCL_BIQUAD_PEAK:
        n2 = 1.0f, n1 = a * k, n0 = 1.0f, d2 = 1.0f, d1 = k * ainv, d0 = 1.0f;
        break;
    case PCL_BIQUAD_LOWSHELF:
        n2 = a, n1 = r * k * a, n0 = a * a, d2 = a, d1 = r * k, d0 = 1.0f;
        break;
    default: /* PCL_BIQUAD_HIGHSHELF */
        n2 = a * a, n1 = r * k * a, n0 = a, d2 = 1.0f, d1 = r * k, d0 = a;
        break;
    }
    nn0 = n2 + n1 * kk + n0 * kk * kk;
    nn1 = 2.0f * (n0 * kk * kk - n2);
    nn2 = n2 - n1 * kk + n0 * kk * kk;
    dd0 = d2 + d1 * kk + d0 * kk * kk;
    dd1 = 2.0f * (d0 * kk * kk - d2);
    dd2 = d2 - d1 * kk + d0 * kk * kk;
    rec = 1.0f / dd0;
    out[0] = nn0 * rec;
    out[1] = nn1 * rec;
    out[2] = nn2 * rec;
    out[3] = dd1 * rec;
    out[4] = dd2 * rec;
}

#else /* fixed point */

#define BQ_ROUND_SHIFT 27 /* the accumulator is 2^-55 units; the history is 2^-28 */

/* Saturates to the Q4.28 range (+-8); written so that the products it feeds are single smulls. */
static int32_t bq_sat32(int64_t v)
{
    int32_t r = (int32_t)v;

    if (v != (int64_t)r)
        r = v > 0 ? INT32_MAX : INT32_MIN;
    PCL_OPAQUE_I32(r);
    return r;
}

/* A Q4.28 value (any int64) as a sample, saturating at full scale. */
static pcl_sample_t bq_out(int32_t y)
{
    const int32_t one = (int32_t)1 << PCL_Q28_SHIFT;
    const int32_t w = y >= one ? one - 1 : (y <= -one ? -one : y);

#if PCL_SAMPLE_TYPE == PCL_Q15
    return pcl_sat_q15(
        (w + (1 << (PCL_Q28_SHIFT - PCL_Q15_SHIFT - 1))) >> (PCL_Q28_SHIFT - PCL_Q15_SHIFT));
#else
    return (pcl_sample_t)((uint32_t)w << (PCL_Q31_SHIFT - PCL_Q28_SHIFT));
#endif
}

/* A sample as Q4.28. */
static pcl_wide_t bq_in(pcl_sample_t x)
{
#if PCL_SAMPLE_TYPE == PCL_Q15
    return (pcl_wide_t)x * (1 << (PCL_Q28_SHIFT - PCL_Q15_SHIFT));
#else
    return (pcl_wide_t)(((int64_t)x + (1 << (PCL_Q31_SHIFT - PCL_Q28_SHIFT - 1)))
        >> (PCL_Q31_SHIFT - PCL_Q28_SHIFT));
#endif
}

static pcl_coef_t bq_u_from_hz(pcl_hz_t f)
{
    return pcl_hz_to_oct(f, PCL_HZ(BQ_REF_HZ)) >> 1; /* Q4.28 / 8 as Q2.30 */
}

static pcl_coef_t bq_u_from_q(pcl_quality_t q)
{
    return pcl_hz_to_oct(q, PCL_QUALITY(BQ_Q_REF)) >> 1;
}

/* dB (Q8.24) / 18 as Q2.30: * 2^6 / 18. */
static pcl_coef_t bq_u_from_db(pcl_db_t db)
{
    return (pcl_coef_t)(((int64_t)db * 64) / BQ_GAIN_RANGE_DB);
}

static int bq_cutoff_valid(const pcl_biquad_t *bq, pcl_hz_t f)
{
    return f >= PCL_HZ(PCL_BIQUAD_CUTOFF_MIN_HZ)
        && (uint64_t)f <= (uint64_t)bq->sample_rate_hz * BQ_MAX_RATIO_Q16;
}

static int bq_q_valid(pcl_quality_t q)
{
    return q >= PCL_QUALITY(PCL_BIQUAD_Q_MIN) && q <= PCL_QUALITY(PCL_BIQUAD_Q_MAX);
}

static int bq_gain_valid(pcl_db_t g)
{
    return g >= -PCL_DB(PCL_BIQUAD_GAIN_MAX_DB) && g <= PCL_DB(PCL_BIQUAD_GAIN_MAX_DB);
}

static int bq_depth_valid(pcl_oct_t d) { return d >= 0; }

#define BQ_MAX_INC (BQ_MAX_RATIO_Q16 << 16) /* the cutoff limit as a phase increment */

/* The per-sample coefficient step; in 64 bits, as a coefficient can swing across most of its Q5.27
 * range. */
static pcl_biquad_c_t bq_step(pcl_biquad_c_t end, pcl_biquad_c_t cur)
{
    return (pcl_biquad_c_t)(((int64_t)end - (int64_t)cur) / (int64_t)PCL_CONTROL_DIV);
}

static pcl_oct_t bq_mod_octaves(const pcl_biquad_t *bq, pcl_sample_t mod)
{
#if PCL_SAMPLE_TYPE == PCL_Q15
    return pcl_mul_coef_q15(bq->mod_depth, mod);
#else
    return pcl_mul_coef_q31(bq->mod_depth, mod);
#endif
}

/* (a * b) >> s, rounded, from the exact 128-bit product; the result must fit 64 bits. 0 < s < 128.
 */
static int64_t bq_mulshr(int64_t a, int64_t b, unsigned s)
{
    const int      neg = (a < 0) != (b < 0);
    const uint64_t ua = a < 0 ? 0u - (uint64_t)a : (uint64_t)a;
    const uint64_t ub = b < 0 ? 0u - (uint64_t)b : (uint64_t)b;
    const uint64_t a0 = ua & 0xFFFFFFFFu;
    const uint64_t a1 = ua >> 32;
    const uint64_t b0 = ub & 0xFFFFFFFFu;
    const uint64_t b1 = ub >> 32;
    const uint64_t p00 = a0 * b0;
    const uint64_t p01 = a0 * b1;
    const uint64_t p10 = a1 * b0;
    const uint64_t p11 = a1 * b1;
    /* the magnitudes are below 2^63, so a1 and b1 are below 2^31 and the middle sum cannot carry */
    const uint64_t mid = p01 + p10;
    uint64_t       lo = p00 + (mid << 32);
    uint64_t       hi = p11 + (mid >> 32) + (uint64_t)(lo < p00);
    uint64_t       add_lo = 0;
    uint64_t       add_hi = 0;
    uint64_t       r;

    /* + 2^(s-1) for rounding to nearest */
    if (s - 1u < 64u)
        add_lo = (uint64_t)1 << (s - 1u);
    else
        add_hi = (uint64_t)1 << (s - 1u - 64u);
    lo += add_lo;
    hi += add_hi + (uint64_t)(lo < add_lo);
    if (s < 64u)
        r = (lo >> s) | (hi << (64u - s));
    else
        r = hi >> (s - 64u);
    return neg ? -(int64_t)r : (int64_t)r;
}

/* Q4.28 product of two int64-held Q4.28 values. */
static int64_t bq_mul28(int64_t a, int64_t b) { return (a * b + ((int64_t)1 << 27)) >> 28; }

/*
 * The coefficients: bilinear transform of the type's analogue prototype (n2 s^2 + n1 s + n0) /
 * (d2 s^2 + d1 s + d0), s = (1/K)(1 - z^-1)/(1 + z^-1), K = tan(pi f / fs). The polynomial values
 * are held in Q.48 and multiplied exactly in 128 bits, and 1 / D0 is refined by a Newton step to
 * about 54 bits: with a1 near -2 at low cutoffs, D0 must be right to ~1e-10 or the cutoff moves.
 */
static void bq_design(const pcl_biquad_t *bq, pcl_oct_t mod_oct, pcl_biquad_c_t out[5])
{
    const int64_t  one = (int64_t)1 << PCL_Q28_SHIFT;
    const int64_t  oct = (int64_t)bq->cutoff.current * 2 + mod_oct; /* Q4.28 */
    const uint32_t inc = pcl_octave_scale_inc(bq->inc_1k, bq_sat32(oct), BQ_MAX_INC);
    const int64_t  kk = pcl_tan_inc(inc); /* K, Q4.28 */
    const int64_t  k = pcl_octave_scale_inc(
        (uint32_t)one, bq_sat32(-(int64_t)bq->q.current * 2), UINT32_MAX); /* 1 / Q, Q4.28 */
    int64_t  a = one;
    int64_t  r = one;
    int64_t  ainv = one;
    int64_t  n2, n1, n0, d2, d1, d0;
    int64_t  k2, nn[3], dd[3];
    int64_t  d28, r0, t, rec;
    unsigned i;

    if (bq_uses_gain(bq->type))
    {
        const pcl_db_t db = (pcl_db_t)(((int64_t)bq->gain.current * 9 + 16) >> 5); /* Q8.24 */
        const int64_t  r28 = (int64_t)pcl_db_to_gain(db >> 2) >> 2; /* sqrt(A), Q4.28 */
        const int64_t  ri28 = (int64_t)pcl_db_to_gain(-(db >> 2)) >> 2;

        r = r28;
        a = bq_mul28(r28, r28);
        ainv = bq_mul28(ri28, ri28);
    }
    switch ((pcl_biquad_type_t)bq->type)
    {
    case PCL_BIQUAD_LP:
        n2 = 0, n1 = 0, n0 = one, d2 = one, d1 = k, d0 = one;
        break;
    case PCL_BIQUAD_HP:
        n2 = one, n1 = 0, n0 = 0, d2 = one, d1 = k, d0 = one;
        break;
    case PCL_BIQUAD_BP:
        n2 = 0, n1 = k, n0 = 0, d2 = one, d1 = k, d0 = one;
        break;
    case PCL_BIQUAD_NOTCH:
        n2 = one, n1 = 0, n0 = one, d2 = one, d1 = k, d0 = one;
        break;
    case PCL_BIQUAD_ALLPASS:
        n2 = one, n1 = -k, n0 = one, d2 = one, d1 = k, d0 = one;
        break;
    case PCL_BIQUAD_PEAK:
        n2 = one, n1 = bq_mul28(a, k), n0 = one, d2 = one, d1 = bq_mul28(k, ainv), d0 = one;
        break;
    case PCL_BIQUAD_LOWSHELF:
        n2 = a, n1 = bq_mul28(bq_mul28(r, k), a), n0 = bq_mul28(a, a), d2 = a, d1 = bq_mul28(r, k),
        d0 = one;
        break;
    default: /* PCL_BIQUAD_HIGHSHELF */
        n2 = bq_mul28(a, a), n1 = bq_mul28(bq_mul28(r, k), a), n0 = a, d2 = one,
        d1 = bq_mul28(r, k), d0 = a;
        break;
    }
    /* to Q.48 */
    n2 *= (int64_t)1 << 20, n1 *= (int64_t)1 << 20, n0 *= (int64_t)1 << 20;
    d2 *= (int64_t)1 << 20, d1 *= (int64_t)1 << 20, d0 *= (int64_t)1 << 20;
    k2 = bq_mulshr(kk, kk, 8); /* K^2, Q.48 */
    {
        const int64_t n1k = bq_mulshr(n1, kk, 28);
        const int64_t n0k2 = bq_mulshr(n0, k2, 48);
        const int64_t d1k = bq_mulshr(d1, kk, 28);
        const int64_t d0k2 = bq_mulshr(d0, k2, 48);

        nn[0] = n2 + n1k + n0k2;
        nn[1] = 2 * (n0k2 - n2);
        nn[2] = n2 - n1k + n0k2;
        dd[0] = d2 + d1k + d0k2;
        dd[1] = 2 * (d0k2 - d2);
        dd[2] = d2 - d1k + d0k2;
    }
    /* 1 / D0 to ~54 bits: a 28-bit division, then one Newton step R <- R (2 - D0 R) in Q.60. */
    d28 = dd[0] >> 20;
    if (d28 < 1)
        d28 = 1;
    r0 = (int64_t)(((uint64_t)1 << 58) / (uint64_t)d28); /* Q.30 */
    t = ((int64_t)2 << 60) - bq_mulshr(dd[0], r0, 18);
    rec = bq_mulshr(r0, t, 30); /* Q.60 */
    for (i = 0; i < 3; i++)
        out[i] = bq_sat32(bq_mulshr(nn[i], rec, 81)); /* Q5.27 */
    out[3] = bq_sat32(bq_mulshr(dd[1], rec, 78));     /* Q2.30 */
    out[4] = bq_sat32(bq_mulshr(dd[2], rec, 78));
}

#endif

/* ---- Setup ---- */

/* Jumps the coefficients to the current smoothed values with no modulation. */
static void bq_settle(pcl_biquad_t *bq)
{
    unsigned i;

    bq_design(bq, 0, bq->c);
    for (i = 0; i < 5; i++)
    {
        bq->c_end[i] = bq->c[i];
        bq->c_step[i] = 0;
    }
    bq->ramping = 0;
    bq->pos = 0;
    bq->mod_dirty = 0;
    bq->retarget = 0;
}

static void bq_clear_history(pcl_biquad_t *bq)
{
    bq->x1 = 0;
    bq->x2 = 0;
    bq->y1 = 0;
    bq->y2 = 0;
    bq->err = 0;
}

pcl_status_t pcl_biquad_init(pcl_biquad_t *bq, const pcl_biquad_config_t *cfg)
{
    pcl_biquad_t tmp;
    uint32_t     ramp;

    if (bq == NULL || cfg == NULL)
        return PCL_ERR_NULL_POINTER;
    if (cfg->sample_rate_hz < PCL_SAMPLE_RATE_MIN_HZ || cfg->sample_rate_hz > PCL_SAMPLE_RATE_MAX_HZ
        || !bq_type_valid(cfg->type) || !bq_q_valid(cfg->q) || !bq_gain_valid(cfg->gain_db)
        || !bq_depth_valid(cfg->mod_depth_oct))
        return PCL_ERR_OUT_OF_RANGE;
    tmp.sample_rate_hz = cfg->sample_rate_hz;
    if (!bq_cutoff_valid(&tmp, cfg->cutoff_hz))
        return PCL_ERR_OUT_OF_RANGE;

    ramp = pcl_smooth_ramp_samples(cfg->sample_rate_hz);
    pcl_smooth_init(&tmp.cutoff, bq_u_from_hz(cfg->cutoff_hz), ramp);
    pcl_smooth_init(&tmp.q, bq_u_from_q(cfg->q), ramp);
    pcl_smooth_init(&tmp.gain, bq_u_from_db(cfg->gain_db), ramp);
    tmp.type = (uint8_t)cfg->type;
    tmp.mod_depth = cfg->mod_depth_oct;
#if PCL_SAMPLE_TYPE != PCL_FLOAT
    tmp.inc_1k = (uint32_t)(((uint64_t)BQ_REF_HZ << 32) / cfg->sample_rate_hz);
#else
    tmp.inc_1k = 0u;
#endif
    bq_clear_history(&tmp);
    bq_settle(&tmp);
    *bq = tmp;
    return PCL_OK;
}

void pcl_biquad_reset(pcl_biquad_t *bq)
{
    pcl_smooth_reset(&bq->cutoff);
    pcl_smooth_reset(&bq->q);
    pcl_smooth_reset(&bq->gain);
    bq_clear_history(bq);
    bq_settle(bq);
}

pcl_status_t pcl_biquad_set_type(pcl_biquad_t *bq, pcl_biquad_type_t type)
{
    if (bq == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!bq_type_valid(type))
        return PCL_ERR_OUT_OF_RANGE;
    bq->type = (uint8_t)type;
    bq->retarget = 1;
    return PCL_OK;
}

pcl_status_t pcl_biquad_set_cutoff(pcl_biquad_t *bq, pcl_hz_t cutoff_hz)
{
    if (bq == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!bq_cutoff_valid(bq, cutoff_hz))
        return PCL_ERR_OUT_OF_RANGE;
    pcl_smooth_set_target(&bq->cutoff, bq_u_from_hz(cutoff_hz));
    return PCL_OK;
}

pcl_status_t pcl_biquad_set_q(pcl_biquad_t *bq, pcl_quality_t q)
{
    if (bq == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!bq_q_valid(q))
        return PCL_ERR_OUT_OF_RANGE;
    pcl_smooth_set_target(&bq->q, bq_u_from_q(q));
    return PCL_OK;
}

pcl_status_t pcl_biquad_set_gain(pcl_biquad_t *bq, pcl_db_t gain_db)
{
    if (bq == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!bq_gain_valid(gain_db))
        return PCL_ERR_OUT_OF_RANGE;
    pcl_smooth_set_target(&bq->gain, bq_u_from_db(gain_db));
    return PCL_OK;
}

pcl_status_t pcl_biquad_set_mod_depth(pcl_biquad_t *bq, pcl_oct_t octaves)
{
    if (bq == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!bq_depth_valid(octaves))
        return PCL_ERR_OUT_OF_RANGE;
    bq->mod_depth = octaves;
    return PCL_OK;
}

/* ---- Control tick ---- */

/*
 * Runs at the start of every control period: advances the smoothers by one period, computes the
 * coefficients for the end of the period and the per-sample steps that get there. With nothing
 * moving (no modulation, settled smoothers, no pending type change, coefficients not left over from
 * a modulated period) the coefficients stay as they are.
 */
static void bq_tick(pcl_biquad_t *bq, int have_mod, pcl_sample_t mod)
{
    const int moving = !pcl_smooth_is_settled(&bq->cutoff) || !pcl_smooth_is_settled(&bq->q)
        || !pcl_smooth_is_settled(&bq->gain);
    unsigned i;

    if (!have_mod && !moving && !bq->mod_dirty && !bq->retarget)
    {
        for (i = 0; i < 5; i++)
            bq->c_step[i] = 0;
        bq->ramping = 0;
        return;
    }
    if (moving)
    {
        for (i = 0; i < PCL_CONTROL_DIV; i++)
        {
            (void)pcl_smooth_next(&bq->cutoff);
            (void)pcl_smooth_next(&bq->q);
            (void)pcl_smooth_next(&bq->gain);
        }
    }
    bq_design(bq, have_mod ? bq_mod_octaves(bq, mod) : 0, bq->c_end);
    bq->ramping = 0;
    for (i = 0; i < 5; i++)
    {
        bq->c_step[i] = bq_step(bq->c_end[i], bq->c[i]);
        if (bq->c_step[i] != 0)
            bq->ramping = 1;
    }
    bq->mod_dirty = have_mod != 0;
    bq->retarget = 0;
}

/* ---- The filter ---- */

/*
 * Filters `n` samples within one control period. The state lives in locals for the loop: the output
 * pointer could alias the struct's fields as far as the compiler knows (in Q31 both are int32_t).
 */
PCL_FORCE_INLINE void bq_chunk_impl(
    pcl_biquad_t *bq, const pcl_sample_t *in, pcl_sample_t *out, size_t n, const int ramping)
{
    pcl_biquad_c_t b0 = bq->c[0];
    pcl_biquad_c_t b1 = bq->c[1];
    pcl_biquad_c_t b2 = bq->c[2];
    pcl_biquad_c_t a1 = bq->c[3];
    pcl_biquad_c_t a2 = bq->c[4];
    pcl_biquad_c_t s0 = bq->c_step[0];
    pcl_biquad_c_t s1 = bq->c_step[1];
    pcl_biquad_c_t s2 = bq->c_step[2];
    pcl_biquad_c_t s3 = bq->c_step[3];
    pcl_biquad_c_t s4 = bq->c_step[4];
    pcl_wide_t     x1 = bq->x1;
    pcl_wide_t     x2 = bq->x2;
    pcl_wide_t     y1 = bq->y1;
    pcl_wide_t     y2 = bq->y2;
#if PCL_SAMPLE_TYPE != PCL_FLOAT
    int32_t err = bq->err;
#endif
    size_t i;

    for (i = 0; i < n; i++)
    {
        if (ramping)
        {
            b0 += s0;
            b1 += s1;
            b2 += s2;
            a1 += s3;
            a2 += s4;
        }
#if PCL_SAMPLE_TYPE == PCL_FLOAT
        {
            const float x = in[i];
            const float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;

            out[i] = y;
            x2 = x1;
            x1 = x;
            y2 = y1;
            y1 = bq_clamp(y) + BQ_DENORMAL_GUARD_F;
        }
#else
        {
            const pcl_wide_t xi = bq_in(in[i]);
            /* 2^-55 units: b (2^-27) * x (2^-28); a (2^-30) * y (2^-28) is 2^-58, so >> 3 */
            const int64_t acc = (int64_t)b0 * xi + (int64_t)b1 * x1 + (int64_t)b2 * x2 + err
                - (((int64_t)a1 * y1 + (int64_t)a2 * y2 + 4) >> 3);
            const int64_t rounded = (acc + ((int64_t)1 << (BQ_ROUND_SHIFT - 1))) >> BQ_ROUND_SHIFT;
            const pcl_wide_t y = bq_sat32(rounded);

            /* the residual, carried into the next sample; none when the output saturated */
            err = (rounded == (int64_t)y)
                ? (int32_t)(acc - (int64_t)y * ((int64_t)1 << BQ_ROUND_SHIFT))
                : 0;
            out[i] = bq_out(y);
            x2 = x1;
            x1 = xi;
            y2 = y1;
            y1 = y;
        }
#endif
    }

    bq->c[0] = b0;
    bq->c[1] = b1;
    bq->c[2] = b2;
    bq->c[3] = a1;
    bq->c[4] = a2;
    bq->x1 = x1;
    bq->x2 = x2;
    bq->y1 = y1;
    bq->y2 = y2;
#if PCL_SAMPLE_TYPE != PCL_FLOAT
    bq->err = err;
#endif
}

/* The same body twice, so that the common case (coefficients not moving) carries no ramp state. */
static void bq_chunk_static(pcl_biquad_t *bq, const pcl_sample_t *in, pcl_sample_t *out, size_t n)
{
    bq_chunk_impl(bq, in, out, n, 0);
}

static void bq_chunk_ramp(pcl_biquad_t *bq, const pcl_sample_t *in, pcl_sample_t *out, size_t n)
{
    bq_chunk_impl(bq, in, out, n, 1);
}

void pcl_biquad_process(pcl_biquad_t *bq, const pcl_sample_t *in, pcl_sample_t *out,
    const pcl_sample_t *cutoff_mod, size_t frames)
{
    size_t n = 0;

    while (n < frames)
    {
        size_t chunk = (size_t)PCL_CONTROL_DIV - bq->pos;

        if (chunk > frames - n)
            chunk = frames - n;
        if (bq->pos == 0u)
            bq_tick(bq, cutoff_mod != NULL, cutoff_mod != NULL ? cutoff_mod[n] : (pcl_sample_t)0);

        if (bq->ramping)
            bq_chunk_ramp(bq, in + n, out + n, chunk);
        else
            bq_chunk_static(bq, in + n, out + n, chunk);

        bq->pos = (uint8_t)(bq->pos + chunk);
        if (bq->pos >= PCL_CONTROL_DIV)
        {
            /* The period is complete: land exactly on its coefficients, whatever the rounding. */
            unsigned i;

            bq->pos = 0u;
            for (i = 0; i < 5; i++)
                bq->c[i] = bq->c_end[i];
        }
        n += chunk;
    }
}
