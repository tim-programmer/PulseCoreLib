/*
 * pcl_svf.c - see include/pcl/pcl_svf.h and docs/modules/svf.md.
 *
 * Fixed point: states, g and c are pcl_wide_t (Q4.28), a1 and the damping k are Q2.30; products are
 * 64-bit, rounded, and saturate back to Q4.28 (+-8). Float: the same arithmetic in float, clamped
 * to
 * +-8 and with denormals flushed, so the three builds behave alike.
 */
#include "pcl/pcl_svf.h"

#include "core/pcl_octave.h"
#include "pcl/pcl_math.h"

#define SVF_REF_HZ 1000 /* the cutoff smoother holds octaves above this */

#define SVF_OCT_RANGE 8 /* the smoother holds octaves / 8, so +-8 octaves fit pcl_coef_t */

/* f / fs above which a cutoff is refused (and the filter clamps): 0.45 * 2^16, truncated. */
#define SVF_MAX_RATIO_Q16 29491u

#define SVF_MAX_RATIO_F 0.45f

#define SVF_SQRT2 1.41421356237309505

#if PCL_SAMPLE_TYPE == PCL_FLOAT

#define SVF_LIMIT_F 8.0f

/*
 * Anti-denormal guard, added to each state every sample: a state can then never decay into the
 * subnormal range (its magnitude stays above ~1e-24, far above 1e-38), and the cost is one addition
 * instead of a compare. -480 dBFS of offset.
 */
#define SVF_DENORMAL_GUARD_F 1.0e-24f

/* Clamp to +-8; written so that a NaN fails both tests and is clamped too. */
static float svf_clamp(float v)
{
    if (!(v > -SVF_LIMIT_F))
        return -SVF_LIMIT_F;
    if (!(v < SVF_LIMIT_F))
        return SVF_LIMIT_F;
    return v;
}

static float svf_damping_from_resonance(pcl_coef_t r)
{
    const float q = 1.0f - r;
    return (float)SVF_SQRT2 * q * q;
}

static pcl_coef_t svf_u_from_hz(pcl_hz_t f)
{
    return pcl_hz_to_oct(f, (float)SVF_REF_HZ) * (1.0f / (float)SVF_OCT_RANGE);
}

static int svf_cutoff_valid(const pcl_svf_t *svf, pcl_hz_t f)
{
    return f >= (float)PCL_SVF_CUTOFF_MIN_HZ && f <= SVF_MAX_RATIO_F * (float)svf->sample_rate_hz;
}

static int svf_resonance_valid(pcl_coef_t r) { return r >= 0.0f && r <= 1.0f; }

static int svf_depth_valid(pcl_oct_t d) { return d >= 0.0f && d < 8.0f; }

/* Coefficients at the current smoothed cutoff and damping plus `mod_oct` octaves. */
static void svf_target(
    const pcl_svf_t *svf, pcl_oct_t mod_oct, pcl_wide_t *g, pcl_wide_t *c, pcl_coef_t *a1)
{
    const float oct = svf->cutoff.current * (float)SVF_OCT_RANGE + mod_oct;
    const float gg = pcl_tan_hz(pcl_hz_scale((float)SVF_REF_HZ, oct), svf->sample_rate_hz);
    const float cc = gg + svf->damping.current;

    *g = gg;
    *c = cc;
    *a1 = 1.0f / (1.0f + gg * cc);
}

static pcl_oct_t svf_mod_octaves(const pcl_svf_t *svf, pcl_sample_t mod)
{
    return mod * svf->mod_depth;
}

#else /* fixed point */

#define SVF_ROUND28 ((int64_t)1 << (PCL_Q28_SHIFT - 1))
#define SVF_ROUND30 ((int64_t)1 << (PCL_Q30_SHIFT - 1))

/*
 * Saturates to the Q4.28 range, which is the +-8 limit. Written as "convert, then check the
 * conversion was lossless" and made opaque, so the products it feeds are single 32x32 -> 64
 * multiplies.
 */
static int32_t svf_sat(int64_t v)
{
    int32_t r = (int32_t)v;

    if (v != (int64_t)r)
        r = v > 0 ? INT32_MAX : INT32_MIN;
    PCL_OPAQUE_I32(r);
    return r;
}

/* Rounded Q4.28 product, as a 64-bit value that has not been saturated. */
static int64_t svf_mul28(int32_t a, int32_t b)
{
    return ((int64_t)a * b + SVF_ROUND28) >> PCL_Q28_SHIFT;
}

static pcl_coef_t svf_damping_from_resonance(pcl_coef_t r)
{
    static const int32_t sqrt2_q30 = PCL_CONST_Q30(SVF_SQRT2);
    const int64_t        q = ((int64_t)1 << PCL_Q30_SHIFT) - r; /* 1 - r, 0 .. 1 */
    const int64_t        q2 = (q * q + SVF_ROUND30) >> PCL_Q30_SHIFT;

    return (pcl_coef_t)((q2 * sqrt2_q30 + SVF_ROUND30) >> PCL_Q30_SHIFT);
}

/* Octaves above 1 kHz, divided by 8, as Q2.30: Q4.28 >> 1. */
static pcl_coef_t svf_u_from_hz(pcl_hz_t f) { return pcl_hz_to_oct(f, PCL_HZ(SVF_REF_HZ)) >> 1; }

static int svf_cutoff_valid(const pcl_svf_t *svf, pcl_hz_t f)
{
    return f >= PCL_HZ(PCL_SVF_CUTOFF_MIN_HZ)
        && (uint64_t)f <= (uint64_t)svf->sample_rate_hz * SVF_MAX_RATIO_Q16;
}

static int svf_resonance_valid(pcl_coef_t r)
{
    return r >= 0 && r <= ((pcl_coef_t)1 << PCL_Q30_SHIFT);
}

static int svf_depth_valid(pcl_oct_t d) { return d >= 0; }

#define SVF_MAX_INC (SVF_MAX_RATIO_Q16 << 16) /* the limit as a phase increment */

static void svf_target(
    const pcl_svf_t *svf, pcl_oct_t mod_oct, pcl_wide_t *g, pcl_wide_t *c, pcl_coef_t *a1)
{
    const int64_t    oct = ((int64_t)svf->cutoff.current * 2) + mod_oct; /* Q4.28 */
    const uint32_t   inc = pcl_octave_scale_inc(svf->inc_1k, (int32_t)svf_sat(oct), SVF_MAX_INC);
    const pcl_wide_t gg = pcl_tan_inc(inc);
    const pcl_wide_t cc = gg + (svf->damping.current >> (PCL_Q30_SHIFT - PCL_Q28_SHIFT));
    /* 1 + g*c reaches about 50, outside Q4.28, so keep it in 64 bits: a1 = 2^58 / d. */
    const uint64_t d = ((uint64_t)1 << PCL_Q28_SHIFT) + (uint64_t)svf_mul28(gg, cc);

    *g = gg;
    *c = cc;
    *a1 = (pcl_coef_t)(((uint64_t)1 << (PCL_Q28_SHIFT + PCL_Q30_SHIFT)) / d);
}

static pcl_oct_t svf_mod_octaves(const pcl_svf_t *svf, pcl_sample_t mod)
{
#if PCL_SAMPLE_TYPE == PCL_Q15
    return pcl_mul_coef_q15(svf->mod_depth, mod);
#else
    return pcl_mul_coef_q31(svf->mod_depth, mod);
#endif
}

#endif

/* a1 * c: the coefficient of s1 in hp = a1*(x - s2) - (a1*c)*s1. At most about 1.5 whatever g is.
 */
static pcl_coef_t svf_ac(pcl_coef_t a1, pcl_wide_t c)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return a1 * c;
#else
    return (pcl_coef_t)(((int64_t)a1 * c + SVF_ROUND28) >> PCL_Q28_SHIFT);
#endif
}

/* ---- Setup ---- */

/* Jumps the coefficients to the current smoothed values with no modulation. */
static void svf_settle(pcl_svf_t *svf)
{
    pcl_svf_t *const s = svf;

    svf_target(s, 0, &s->g, &s->c, &s->a1);
    s->g_end = s->g;
    s->c_end = s->c;
    s->a1_end = s->a1;
    s->ac = svf_ac(s->a1, s->c);
    s->ac_end = s->ac;
    s->dg = 0;
    s->dc = 0;
    s->ramping = 0;
    s->pos = 0;
    s->mod_dirty = 0;
}

pcl_status_t pcl_svf_init(pcl_svf_t *svf, const pcl_svf_config_t *cfg)
{
    pcl_svf_t tmp;
    uint32_t  ramp;

    if (svf == NULL || cfg == NULL)
        return PCL_ERR_NULL_POINTER;
    if (cfg->sample_rate_hz < PCL_SAMPLE_RATE_MIN_HZ || cfg->sample_rate_hz > PCL_SAMPLE_RATE_MAX_HZ
        || !svf_resonance_valid(cfg->resonance) || !svf_depth_valid(cfg->mod_depth_oct))
        return PCL_ERR_OUT_OF_RANGE;
    tmp.sample_rate_hz = cfg->sample_rate_hz;
    if (!svf_cutoff_valid(&tmp, cfg->cutoff_hz))
        return PCL_ERR_OUT_OF_RANGE;

    ramp = pcl_smooth_ramp_samples(cfg->sample_rate_hz);
    pcl_smooth_init(&tmp.cutoff, svf_u_from_hz(cfg->cutoff_hz), ramp);
    pcl_smooth_init(&tmp.damping, svf_damping_from_resonance(cfg->resonance), ramp);
    tmp.s1 = 0;
    tmp.s2 = 0;
    tmp.mod_depth = cfg->mod_depth_oct;
    tmp.saturate = cfg->saturate != 0;
#if PCL_SAMPLE_TYPE != PCL_FLOAT
    tmp.inc_1k = (uint32_t)(((uint64_t)SVF_REF_HZ << 32) / cfg->sample_rate_hz);
#else
    tmp.inc_1k = 0u;
#endif
    svf_settle(&tmp);
    *svf = tmp;
    return PCL_OK;
}

void pcl_svf_reset(pcl_svf_t *svf)
{
    pcl_smooth_reset(&svf->cutoff);
    pcl_smooth_reset(&svf->damping);
    svf->s1 = 0;
    svf->s2 = 0;
    svf_settle(svf);
}

pcl_status_t pcl_svf_set_cutoff(pcl_svf_t *svf, pcl_hz_t cutoff_hz)
{
    if (svf == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!svf_cutoff_valid(svf, cutoff_hz))
        return PCL_ERR_OUT_OF_RANGE;
    pcl_smooth_set_target(&svf->cutoff, svf_u_from_hz(cutoff_hz));
    return PCL_OK;
}

pcl_status_t pcl_svf_set_resonance(pcl_svf_t *svf, pcl_coef_t resonance)
{
    if (svf == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!svf_resonance_valid(resonance))
        return PCL_ERR_OUT_OF_RANGE;
    pcl_smooth_set_target(&svf->damping, svf_damping_from_resonance(resonance));
    return PCL_OK;
}

pcl_status_t pcl_svf_set_mod_depth(pcl_svf_t *svf, pcl_oct_t octaves)
{
    if (svf == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!svf_depth_valid(octaves))
        return PCL_ERR_OUT_OF_RANGE;
    svf->mod_depth = octaves;
    return PCL_OK;
}

void pcl_svf_set_saturate(pcl_svf_t *svf, int saturate) { svf->saturate = saturate != 0; }

/*
 * d = 1 + g*c with g and c linear in the sample index is quadratic, so its per-sample change is a
 * first difference that itself changes by a constant second difference: g0*dc + c0*dg + dg*dc and
 * 2*dg*dc. Two additions per sample then give d for the Newton step, with no multiplication.
 */
static void svf_d_steps(pcl_svf_t *svf)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    svf->d = 1.0f + svf->g * svf->c;
    svf->d_step = svf->g * svf->dc + svf->c * svf->dg + svf->dg * svf->dc;
    svf->d_step2 = 2.0f * svf->dg * svf->dc;
#else
    /* Q4.28 * Q4.28 = Q8.56; to Q8.24 is a shift of 32. */
    const int64_t g = svf->g;
    const int64_t c = svf->c;
    const int64_t dg = svf->dg;
    const int64_t dc = svf->dc;
    const int64_t half = (int64_t)1 << 31;

    svf->d = (pcl_svf_d_t)(((int64_t)1 << PCL_Q24_SHIFT) + ((g * c + half) >> 32));
    svf->d_step = (pcl_svf_d_t)((g * dc + c * dg + dg * dc + half) >> 32);
    svf->d_step2 = (pcl_svf_d_t)((2 * dg * dc + half) >> 32);
#endif
}

/* One Newton step toward 1/d: a1 * (2 - d*a1). Converges quadratically from a close start. */
static pcl_coef_t svf_newton(pcl_coef_t a1, pcl_svf_d_t d)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    /* Newton from below never exceeds 1/d <= 1, so only the floor needs guarding. */
    const float a = a1 * (2.0f - d * a1);
    return a < 0.0f ? 0.0f : a;
#else
    const int64_t one = (int64_t)1 << PCL_Q30_SHIFT;
    /* d (Q8.24) * a1 (Q2.30) = Q10.54; >> 24 gives d*a1 as Q2.30, about 1. */
    const int64_t t = ((int64_t)d * a1 + ((int64_t)1 << 23)) >> 24;
    /* 2 - t is about 1; kept in 0 .. 2^31 - 1 so the product is a single 32x32 -> 64 multiply */
    const int64_t u = 2 * one - t;
    const int32_t u32 = (int32_t)(u < 0 ? 0 : (u > INT32_MAX ? INT32_MAX : u));
    const int64_t a = ((int64_t)a1 * u32 + SVF_ROUND30) >> PCL_Q30_SHIFT;

    return (pcl_coef_t)(a < 0 ? 0 : (a > one ? one : a));
#endif
}

/* ---- Control tick ---- */

/*
 * Runs at the start of every control period: advances the smoothers by one period, computes the
 * coefficients for the end of the period and the per-sample steps that get there. With nothing
 * moving (no modulation, settled smoothers, coefficients not left over from a modulated period) the
 * coefficients stay as they are.
 */
static void svf_tick(pcl_svf_t *svf, int have_mod, pcl_sample_t mod)
{
    const int moving
        = !pcl_smooth_is_settled(&svf->cutoff) || !pcl_smooth_is_settled(&svf->damping);

    if (!have_mod && !moving && !svf->mod_dirty)
    {
        svf->dg = 0;
        svf->dc = 0;
        svf->ramping = 0;
        return;
    }
    if (moving)
    {
        unsigned i;
        for (i = 0; i < PCL_CONTROL_DIV; i++)
        {
            (void)pcl_smooth_next(&svf->cutoff);
            (void)pcl_smooth_next(&svf->damping);
        }
    }
    svf_target(
        svf, have_mod ? svf_mod_octaves(svf, mod) : 0, &svf->g_end, &svf->c_end, &svf->a1_end);
    svf->ac_end = svf_ac(svf->a1_end, svf->c_end);
    svf->dg = (pcl_wide_t)((svf->g_end - svf->g) / (pcl_wide_t)PCL_CONTROL_DIV);
    svf->dc = (pcl_wide_t)((svf->c_end - svf->c) / (pcl_wide_t)PCL_CONTROL_DIV);
    svf->ramping = svf->dg != 0 || svf->dc != 0; /* a constant needs no Newton step */
    svf_d_steps(svf);
    svf->mod_dirty = have_mod != 0;
}

/* ---- The filter ---- */

typedef struct
{
    pcl_sample_t  *lp;
    pcl_sample_t  *bp;
    pcl_sample_t  *hp;
    pcl_sample_t  *md; /* the single-mode output, or NULL */
    pcl_svf_mode_t mode;
} svf_outputs_t;

#if PCL_SAMPLE_TYPE != PCL_FLOAT

/* A Q4.28 value (any int64) as a sample, saturating at full scale. */
static pcl_sample_t svf_out(int64_t v)
{
    const int64_t one = (int64_t)1 << PCL_Q28_SHIFT;
    const int32_t w = (int32_t)(v >= one ? one - 1 : (v <= -one ? -one : v));

#if PCL_SAMPLE_TYPE == PCL_Q15
    return pcl_sat_q15(
        (w + (1 << (PCL_Q28_SHIFT - PCL_Q15_SHIFT - 1))) >> (PCL_Q28_SHIFT - PCL_Q15_SHIFT));
#else
    return (pcl_sample_t)((uint32_t)w << (PCL_Q31_SHIFT - PCL_Q28_SHIFT));
#endif
}

/* A sample as Q4.28. */
static pcl_wide_t svf_in(pcl_sample_t x)
{
#if PCL_SAMPLE_TYPE == PCL_Q15
    return (pcl_wide_t)x * (1 << (PCL_Q28_SHIFT - PCL_Q15_SHIFT));
#else
    return (pcl_wide_t)(((int64_t)x + (1 << (PCL_Q31_SHIFT - PCL_Q28_SHIFT - 1)))
        >> (PCL_Q31_SHIFT - PCL_Q28_SHIFT));
#endif
}

#endif

/*
 * Filters `n` samples within one control period. The state lives in locals for the loop: the output
 * pointers could alias the struct's fields as far as the compiler knows (in Q31 both are int32_t).
 *
 * Fixed point: intermediate values stay in 64 bits and are saturated only where they must be. With
 * the states within +-8, |t*a1| < 21 and |g*hp| < 10 for every g, c and a1 the control tick can
 * produce, so nothing overflows; bp is saturated because it feeds a product, and the new states and
 * the outputs because they are stored.
 */
PCL_FORCE_INLINE void svf_chunk_impl(
    pcl_svf_t *svf, const pcl_sample_t *in, const svf_outputs_t *o, size_t n, const int ramping)
{
    pcl_wide_t        s1 = svf->s1;
    pcl_wide_t        s2 = svf->s2;
    pcl_wide_t        g = svf->g;
    pcl_wide_t        c = svf->c;
    pcl_coef_t        a1 = svf->a1;
    pcl_coef_t        ac = svf->ac;
    const pcl_wide_t  dg = svf->dg;
    const pcl_wide_t  dc = svf->dc;
    pcl_svf_d_t       d = svf->d;
    pcl_svf_d_t       d_step = svf->d_step;
    const pcl_svf_d_t d_step2 = svf->d_step2;
    const int         saturate = svf->saturate;
    size_t            i;

    for (i = 0; i < n; i++)
    {
        g += dg;
        c += dc;
        if (ramping)
        {
            d += d_step;
            d_step += d_step2;
            a1 = svf_newton(a1, d);
            ac = svf_ac(a1, c);
        }

#if PCL_SAMPLE_TYPE == PCL_FLOAT
        {
            const float x = in[i];
            const float t = x - c * s1 - s2;
            const float hp = a1 * t;
            const float gh = g * hp;
            const float bp = gh + s1;
            const float gb = g * bp;
            const float lp = gb + s2;

            s1 = svf_clamp(bp + gh) + SVF_DENORMAL_GUARD_F;
            s2 = svf_clamp(lp + gb) + SVF_DENORMAL_GUARD_F;
            if (saturate)
                s1 = pcl_tanh(s1);
            if (o->lp != NULL)
                o->lp[i] = lp;
            if (o->bp != NULL)
                o->bp[i] = bp;
            if (o->hp != NULL)
                o->hp[i] = hp;
            if (o->md != NULL)
            {
                /* k = c - g, exactly; the derived responses are sums of the three outputs. */
                const float kbp = (c - g) * bp;
                float       m;

                switch (o->mode)
                {
                case PCL_SVF_LP:
                    m = lp;
                    break;
                case PCL_SVF_BP:
                    m = bp;
                    break;
                case PCL_SVF_BP_NORM:
                    m = kbp;
                    break;
                case PCL_SVF_HP:
                    m = hp;
                    break;
                case PCL_SVF_NOTCH:
                    m = lp + hp;
                    break;
                case PCL_SVF_PEAK:
                    m = lp - hp;
                    break;
                default:
                    m = lp + hp - kbp;
                    break;
                }
                o->md[i] = m;
            }
        }
#else
        {
            /* hp = a1*(x - s2) - (a1*c)*s1 in one accumulator: both products are 32x32 -> 64
               multiplies, with no need to hold the large t = x - c*s1 - s2 (c reaches 7.7). */
            const pcl_wide_t xs = svf_sat((int64_t)svf_in(in[i]) - s2);
            const pcl_wide_t hp
                = svf_sat(((int64_t)a1 * xs - (int64_t)ac * s1 + SVF_ROUND30) >> PCL_Q30_SHIFT);
            const int64_t    gh = ((int64_t)g * hp + SVF_ROUND28) >> PCL_Q28_SHIFT;
            const pcl_wide_t bp = svf_sat(gh + s1);
            const int64_t    gb = svf_mul28(g, bp);
            const int64_t    lp = gb + s2;

            s1 = svf_sat((int64_t)bp + gh);
            s2 = svf_sat(lp + gb);
            if (saturate)
                s1 = pcl_sample_to_wide(pcl_tanh(s1));
            if (o->lp != NULL)
                o->lp[i] = svf_out(lp);
            if (o->bp != NULL)
                o->bp[i] = svf_out(bp);
            if (o->hp != NULL)
                o->hp[i] = svf_out(hp);
            if (o->md != NULL)
            {
                /* k = c - g, exactly; the derived responses are sums of the three outputs. */
                const int64_t kbp = svf_mul28(c - g, bp);
                int64_t       m;

                switch (o->mode)
                {
                case PCL_SVF_LP:
                    m = lp;
                    break;
                case PCL_SVF_BP:
                    m = bp;
                    break;
                case PCL_SVF_BP_NORM:
                    m = kbp;
                    break;
                case PCL_SVF_HP:
                    m = hp;
                    break;
                case PCL_SVF_NOTCH:
                    m = lp + hp;
                    break;
                case PCL_SVF_PEAK:
                    m = lp - hp;
                    break;
                default:
                    m = lp + hp - kbp;
                    break;
                }
                o->md[i] = svf_out(m);
            }
        }
#endif
    }

    svf->s1 = s1;
    svf->s2 = s2;
    svf->g = g;
    svf->c = c;
    svf->a1 = a1;
    svf->ac = ac;
    svf->d = d;
    svf->d_step = d_step;
}

/* The same body twice, so that the common case (coefficients not moving) carries no ramp state. */
static void svf_chunk_static(
    pcl_svf_t *svf, const pcl_sample_t *in, const svf_outputs_t *o, size_t n)
{
    svf_chunk_impl(svf, in, o, n, 0);
}

static void svf_chunk_ramp(pcl_svf_t *svf, const pcl_sample_t *in, const svf_outputs_t *o, size_t n)
{
    svf_chunk_impl(svf, in, o, n, 1);
}

static void svf_run(pcl_svf_t *svf, const pcl_sample_t *in, const svf_outputs_t *o,
    const pcl_sample_t *cutoff_mod, size_t frames)
{
    size_t n = 0;

    while (n < frames)
    {
        size_t        chunk = (size_t)PCL_CONTROL_DIV - svf->pos;
        svf_outputs_t sub;

        if (chunk > frames - n)
            chunk = frames - n;
        if (svf->pos == 0u)
            svf_tick(svf, cutoff_mod != NULL, cutoff_mod != NULL ? cutoff_mod[n] : (pcl_sample_t)0);

        sub.lp = o->lp != NULL ? o->lp + n : NULL;
        sub.bp = o->bp != NULL ? o->bp + n : NULL;
        sub.hp = o->hp != NULL ? o->hp + n : NULL;
        sub.md = o->md != NULL ? o->md + n : NULL;
        sub.mode = o->mode;
        if (svf->ramping)
            svf_chunk_ramp(svf, in + n, &sub, chunk);
        else
            svf_chunk_static(svf, in + n, &sub, chunk);

        svf->pos = (uint8_t)(svf->pos + chunk);
        if (svf->pos >= PCL_CONTROL_DIV)
        {
            /* The period is complete: land exactly on its coefficients, whatever the rounding. */
            svf->pos = 0u;
            svf->g = svf->g_end;
            svf->c = svf->c_end;
            svf->a1 = svf->a1_end;
            svf->ac = svf->ac_end;
        }
        n += chunk;
    }
}

void pcl_svf_process(pcl_svf_t *svf, const pcl_sample_t *in, pcl_sample_t *out_lp,
    pcl_sample_t *out_bp, pcl_sample_t *out_hp, const pcl_sample_t *cutoff_mod, size_t frames)
{
    svf_outputs_t o;

    o.lp = out_lp;
    o.bp = out_bp;
    o.hp = out_hp;
    o.md = NULL;
    o.mode = PCL_SVF_LP;
    svf_run(svf, in, &o, cutoff_mod, frames);
}

void pcl_svf_process_mode(pcl_svf_t *svf, const pcl_sample_t *in, pcl_sample_t *out,
    pcl_svf_mode_t mode, const pcl_sample_t *cutoff_mod, size_t frames)
{
    svf_outputs_t o;

    o.lp = NULL;
    o.bp = NULL;
    o.hp = NULL;
    o.md = out;
    o.mode = mode;
    svf_run(svf, in, &o, cutoff_mod, frames);
}
