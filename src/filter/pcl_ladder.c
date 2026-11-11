/*
 * pcl_ladder.c - see include/pcl/pcl_ladder.h and docs/modules/ladder.md.
 *
 * Zero-delay-feedback ladder (V. Zavalishin, "The Art of VA Filter Design", rev. 2.1.0, 2018, the
 * TPT ladder and its "cheap" nonlinear form). Each stage is the trapezoidal one-pole
 *     y = G x + (1 - G) s,  s' = 2 y - s,   G = g / (1 + g),  g = tan(pi fc / fs),
 * so the cascade output is y4 = G^4 u + S with S = (1-G) (G (G (G s1 + s2) + s3) + s4), and the
 * feedback u = x - k y4 solves to u = a (x - k S) with a = 1 / (1 + k G^4).
 *
 * Fixed point: states, x, u, S, k and the drive are Q4.28 (pcl_wide_t, +-8); G, 1-G and a are
 * Q2.30. The Horner sum for S reaches about 26 when the states sit at the clamp, so it runs in
 * Q6.26. Products are 32x32 -> 64, rounded, and saturated where a value is stored.
 *
 * Coefficients follow the SVF's control model (src/core/pcl_cutoff.h): G, k and the drive ramp
 * linearly across a control period, and a = 1/d is tracked by a Newton step every sample, never
 * interpolated (1/d is convex in time, so a straight line between its ends is negative damping:
 * see docs/modules/svf.md).
 */
#include "pcl/pcl_ladder.h"

#include "core/pcl_cutoff.h"

PCL_STATIC_ASSERT(PCL_LADDER_CUTOFF_MIN_HZ == PCL_CUTOFF_MIN_HZ, ladder_cutoff_min);

#define LADDER_STAGES 4

#if PCL_SAMPLE_TYPE == PCL_FLOAT

#define LADDER_LIMIT_F 8.0f
#define LADDER_K_PER_RES 4.0f /* k = 4 r: the feedback at which the loop gain at fc is 1 */
#define LADDER_DENORMAL_GUARD_F 1.0e-24f /* as in the SVF: -480 dBFS added to every state  */

/*
 * Below 1/64, tanh(u) = u within u^3/3 <= 1.3e-6, so the saturator passes u through. pcl_tanh's
 * first table segment has slope 0.99992 (8e-5 off the loop gain), and in Q15 its output is rounded
 * to 16 bits, which made a deadband that stopped a self-oscillation from starting.
 */
#define LADDER_TANH_LINEAR_F (1.0f / 64.0f)

/* Clamp to +-8; a NaN fails both tests and is clamped too. */
static float ladder_clamp(float v)
{
    if (!(v > -LADDER_LIMIT_F))
        return -LADDER_LIMIT_F;
    if (!(v < LADDER_LIMIT_F))
        return LADDER_LIMIT_F;
    return v;
}

static int ladder_resonance_valid(pcl_coef_t r)
{
    return r >= 0.0f && r <= (float)PCL_LADDER_RESONANCE_MAX;
}

static int ladder_drive_valid(pcl_wide_t d) { return d >= 1.0f && d < 8.0f; }

/* Coefficients for the current smoothed cutoff and resonance plus `mod_oct` octaves. */
static void ladder_target(const pcl_ladder_t *ld, pcl_oct_t mod_oct, pcl_coef_t *G, pcl_wide_t *k,
    pcl_coef_t *a, pcl_wide_t *ak)
{
    const float g = pcl_cutoff_g(ld->cutoff.current, mod_oct, ld->inc_1k, ld->sample_rate_hz);
    const float gg = g / (1.0f + g);
    const float kk = LADDER_K_PER_RES * ld->resonance.current;
    const float g2 = gg * gg;
    const float aa = 1.0f / (1.0f + kk * g2 * g2);

    *G = gg;
    *k = kk;
    *a = aa;
    *ak = aa * kk;
}

#else /* fixed point */

#define LADDER_ONE28 ((int32_t)1 << PCL_Q28_SHIFT)
#define LADDER_ONE30 ((int32_t)1 << PCL_Q30_SHIFT)
#define LADDER_TANH_LINEAR (LADDER_ONE28 / 64) /* see LADDER_TANH_LINEAR_F */

static int ladder_resonance_valid(pcl_coef_t r)
{
    return r >= 0 && r <= PCL_COEF(PCL_LADDER_RESONANCE_MAX);
}

/* Q4.28 holds anything below 8, so only the lower bound needs a test. */
static int ladder_drive_valid(pcl_wide_t d) { return d >= LADDER_ONE28; }

/* Rounded Q2.30 product of two Q2.30 values in 0 .. 1. */
static int32_t ladder_mul30(int32_t a, int32_t b)
{
    return (int32_t)(((int64_t)a * b + PCL_CUTOFF_ROUND30) >> PCL_Q30_SHIFT);
}

/* k G^4 + 1 as Q4.28 (at most 5.4): the denominator of a. */
static int32_t ladder_d28(pcl_coef_t G, pcl_wide_t k)
{
    const int32_t g2 = ladder_mul30(G, G);
    const int32_t g4 = ladder_mul30(g2, g2);

    return LADDER_ONE28 + (int32_t)(((int64_t)k * g4 + PCL_CUTOFF_ROUND30) >> PCL_Q30_SHIFT);
}

static void ladder_target(const pcl_ladder_t *ld, pcl_oct_t mod_oct, pcl_coef_t *G, pcl_wide_t *k,
    pcl_coef_t *a, pcl_wide_t *ak)
{
    const pcl_wide_t g = pcl_cutoff_g(ld->cutoff.current, mod_oct, ld->inc_1k, ld->sample_rate_hz);
    const int64_t    den = (int64_t)g + LADDER_ONE28; /* 1 + g, Q4.28 */
    const pcl_coef_t gg = (pcl_coef_t)((((int64_t)g << PCL_Q30_SHIFT) + den / 2) / den);
    const pcl_wide_t kk = ld->resonance.current; /* Q2.30 r == Q4.28 (4 r) */
    const int32_t    d = ladder_d28(gg, kk);
    /* a = 2^58 / d: Q2.30 from a Q4.28 denominator. */
    const pcl_coef_t aa
        = (pcl_coef_t)((((int64_t)1 << (PCL_Q28_SHIFT + PCL_Q30_SHIFT)) + d / 2) / d);

    *G = gg;
    *k = kk;
    *a = aa;
    *ak = (pcl_wide_t)(((int64_t)aa * kk + PCL_CUTOFF_ROUND30) >> PCL_Q30_SHIFT);
}

#endif

/* ---- Setup ---- */

static void ladder_settle(pcl_ladder_t *ld)
{
    ladder_target(ld, 0, &ld->G, &ld->k, &ld->a, &ld->ak);
    ld->G_end = ld->G;
    ld->k_end = ld->k;
    ld->a_end = ld->a;
    ld->ak_end = ld->ak;
    ld->drv = ld->drive.current;
    ld->drv_end = ld->drv;
    ld->dG = 0;
    ld->dk = 0;
    ld->ddrv = 0;
    ld->ramping = 0;
    ld->pos = 0;
    ld->mod_dirty = 0;
}

pcl_status_t pcl_ladder_init(pcl_ladder_t *ld, const pcl_ladder_config_t *cfg)
{
    pcl_ladder_t tmp;
    uint32_t     ramp;
    unsigned     i;

    if (ld == NULL || cfg == NULL)
        return PCL_ERR_NULL_POINTER;
    if (cfg->sample_rate_hz < PCL_SAMPLE_RATE_MIN_HZ || cfg->sample_rate_hz > PCL_SAMPLE_RATE_MAX_HZ
        || !pcl_cutoff_valid(cfg->cutoff_hz, cfg->sample_rate_hz)
        || !ladder_resonance_valid(cfg->resonance) || !pcl_cutoff_depth_valid(cfg->mod_depth_oct)
        || (unsigned)cfg->mode >= (unsigned)PCL_LADDER_MODE_COUNT
        || !ladder_drive_valid(cfg->drive))
        return PCL_ERR_OUT_OF_RANGE;

    ramp = pcl_smooth_ramp_samples(cfg->sample_rate_hz);
    tmp.sample_rate_hz = cfg->sample_rate_hz;
    tmp.inc_1k = pcl_cutoff_inc_ref(cfg->sample_rate_hz);
    pcl_smooth_init(&tmp.cutoff, pcl_cutoff_u_from_hz(cfg->cutoff_hz), ramp);
    pcl_smooth_init(&tmp.resonance, cfg->resonance, ramp);
    pcl_smooth_init(&tmp.drive, cfg->drive, ramp);
    for (i = 0; i < LADDER_STAGES; i++)
        tmp.s[i] = 0;
    tmp.mod_depth = cfg->mod_depth_oct;
    tmp.mode = (uint8_t)cfg->mode;
    tmp.saturate = cfg->saturate != 0;
    tmp.bass_comp = cfg->bass_comp != 0;
    ladder_settle(&tmp);
    *ld = tmp;
    return PCL_OK;
}

void pcl_ladder_reset(pcl_ladder_t *ld)
{
    unsigned i;

    pcl_smooth_reset(&ld->cutoff);
    pcl_smooth_reset(&ld->resonance);
    pcl_smooth_reset(&ld->drive);
    for (i = 0; i < LADDER_STAGES; i++)
        ld->s[i] = 0;
    ladder_settle(ld);
}

pcl_status_t pcl_ladder_set_cutoff(pcl_ladder_t *ld, pcl_hz_t cutoff_hz)
{
    if (ld == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!pcl_cutoff_valid(cutoff_hz, ld->sample_rate_hz))
        return PCL_ERR_OUT_OF_RANGE;
    pcl_smooth_set_target(&ld->cutoff, pcl_cutoff_u_from_hz(cutoff_hz));
    return PCL_OK;
}

pcl_status_t pcl_ladder_set_resonance(pcl_ladder_t *ld, pcl_coef_t resonance)
{
    if (ld == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!ladder_resonance_valid(resonance))
        return PCL_ERR_OUT_OF_RANGE;
    pcl_smooth_set_target(&ld->resonance, resonance);
    return PCL_OK;
}

pcl_status_t pcl_ladder_set_mod_depth(pcl_ladder_t *ld, pcl_oct_t octaves)
{
    if (ld == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!pcl_cutoff_depth_valid(octaves))
        return PCL_ERR_OUT_OF_RANGE;
    ld->mod_depth = octaves;
    return PCL_OK;
}

pcl_status_t pcl_ladder_set_mode(pcl_ladder_t *ld, pcl_ladder_mode_t mode)
{
    if (ld == NULL)
        return PCL_ERR_NULL_POINTER;
    if ((unsigned)mode >= (unsigned)PCL_LADDER_MODE_COUNT)
        return PCL_ERR_OUT_OF_RANGE;
    ld->mode = (uint8_t)mode;
    return PCL_OK;
}

pcl_status_t pcl_ladder_set_drive(pcl_ladder_t *ld, pcl_wide_t drive)
{
    if (ld == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!ladder_drive_valid(drive))
        return PCL_ERR_OUT_OF_RANGE;
    pcl_smooth_set_target(&ld->drive, drive);
    return PCL_OK;
}

void pcl_ladder_set_saturate(pcl_ladder_t *ld, int saturate) { ld->saturate = saturate != 0; }

void pcl_ladder_set_bass_comp(pcl_ladder_t *ld, int bass_comp) { ld->bass_comp = bass_comp != 0; }

/* ---- Control tick ---- */

/*
 * At the start of every control period: advance the smoothers a period, compute the coefficients
 * for the end of the period and the per-sample steps toward them. With nothing moving the
 * coefficients stay as they are.
 */
static void ladder_tick(pcl_ladder_t *ld, int have_mod, pcl_sample_t mod)
{
    const int moving = !pcl_smooth_is_settled(&ld->cutoff) || !pcl_smooth_is_settled(&ld->resonance)
        || !pcl_smooth_is_settled(&ld->drive);

    if (!have_mod && !moving && !ld->mod_dirty)
    {
        ld->dG = 0;
        ld->dk = 0;
        ld->ddrv = 0;
        ld->ramping = 0;
        return;
    }
    if (moving)
    {
        unsigned i;
        for (i = 0; i < PCL_CONTROL_DIV; i++)
        {
            (void)pcl_smooth_next(&ld->cutoff);
            (void)pcl_smooth_next(&ld->resonance);
            (void)pcl_smooth_next(&ld->drive);
        }
    }
    ladder_target(ld, have_mod ? pcl_cutoff_mod_octaves(ld->mod_depth, mod) : 0, &ld->G_end,
        &ld->k_end, &ld->a_end, &ld->ak_end);
    ld->drv_end = ld->drive.current;
    ld->dG = (pcl_coef_t)((ld->G_end - ld->G) / (pcl_coef_t)PCL_CONTROL_DIV);
    ld->dk = (pcl_wide_t)((ld->k_end - ld->k) / (pcl_wide_t)PCL_CONTROL_DIV);
    ld->ddrv = (pcl_wide_t)((ld->drv_end - ld->drv) / (pcl_wide_t)PCL_CONTROL_DIV);
    ld->ramping = ld->dG != 0 || ld->dk != 0 || ld->ddrv != 0;
    ld->mod_dirty = have_mod != 0;
}

/* ---- The filter ---- */

/*
 * Filters `n` samples within one control period. State and coefficients live in locals: the output
 * pointer could alias the struct as far as the compiler knows.
 */
PCL_FORCE_INLINE void ladder_chunk_impl(
    pcl_ladder_t *ld, const pcl_sample_t *in, pcl_sample_t *out, size_t n, const int ramping)
{
    pcl_wide_t       s1 = ld->s[0], s2 = ld->s[1], s3 = ld->s[2], s4 = ld->s[3];
    pcl_coef_t       G = ld->G;
    pcl_wide_t       k = ld->k;
    pcl_wide_t       drv = ld->drv;
    pcl_coef_t       a = ld->a;
    pcl_wide_t       ak = ld->ak;
    const pcl_coef_t dG = ld->dG;
    const pcl_wide_t dk = ld->dk;
    const pcl_wide_t ddrv = ld->ddrv;
    const int        saturate = ld->saturate;
    const int        bass_comp = ld->bass_comp;
    const unsigned   mode = ld->mode;
    size_t           i;

    for (i = 0; i < n; i++)
    {
#if PCL_SAMPLE_TYPE == PCL_FLOAT
        float x = in[i], S, u, y1, y2, y3, y4, H, m;

        if (ramping)
        {
            float g2;
            G += dG;
            k += dk;
            drv += ddrv;
            g2 = G * G;
            a = pcl_cutoff_newton(a, 1.0f + k * g2 * g2);
            ak = a * k;
        }
        H = 1.0f - G;
        if (bass_comp)
            x += k * x;
        if (saturate)
            x *= drv;
        S = H * (G * (G * (G * s1 + s2) + s3) + s4);
        u = ladder_clamp(a * x - ak * S);
        if (saturate && !(u > -LADDER_TANH_LINEAR_F && u < LADDER_TANH_LINEAR_F))
            u = pcl_tanh(u);

        y1 = G * u + H * s1;
        y2 = G * y1 + H * s2;
        y3 = G * y2 + H * s3;
        y4 = G * y3 + H * s4;
        s1 = ladder_clamp(2.0f * y1 - s1) + LADDER_DENORMAL_GUARD_F;
        s2 = ladder_clamp(2.0f * y2 - s2) + LADDER_DENORMAL_GUARD_F;
        s3 = ladder_clamp(2.0f * y3 - s3) + LADDER_DENORMAL_GUARD_F;
        s4 = ladder_clamp(2.0f * y4 - s4) + LADDER_DENORMAL_GUARD_F;

        switch (mode)
        {
        case PCL_LADDER_LP24:
            m = y4;
            break;
        case PCL_LADDER_LP18:
            m = y3;
            break;
        case PCL_LADDER_LP12:
            m = y2;
            break;
        case PCL_LADDER_LP6:
            m = y1;
            break;
        case PCL_LADDER_BP12:
            m = 2.0f * (y1 - y2);
            break;
        case PCL_LADDER_BP24:
            m = 4.0f * (y2 - 2.0f * y3 + y4);
            break;
        case PCL_LADDER_HP12:
            m = u - 2.0f * y1 + y2;
            break;
        default:
            m = u - 4.0f * y1 + 6.0f * y2 - 4.0f * y3 + y4;
            break;
        }
        out[i] = m;
#else
        pcl_wide_t x = pcl_sample_to_wide_rounded(in[i]);
        int32_t    H, p;
        pcl_wide_t S, u, y1, y2, y3, y4;
        int64_t    m;

        if (ramping)
        {
            G += dG;
            k += dk;
            drv += ddrv;
            /* d = 1 + k G^4 as Q8.24 for the Newton step. */
            a = pcl_cutoff_newton(a, (pcl_cutoff_d_t)((ladder_d28(G, k) + 8) >> 4));
            ak = (pcl_wide_t)(((int64_t)a * k + PCL_CUTOFF_ROUND30) >> PCL_Q30_SHIFT);
        }
        H = LADDER_ONE30 - G;
        if (bass_comp)
            x = pcl_wide_sat((int64_t)x + pcl_wide_mul(k, x));
        if (saturate)
            x = pcl_wide_sat(pcl_wide_mul(drv, x));

        /* Horner in Q6.26 (values up to ~26 when the states sit at the clamp). G * Q4.28 is Q6.58;
           G * Q6.26 is Q8.56. */
        p = (int32_t)(((int64_t)G * s1 + ((int64_t)s2 * ((int64_t)1 << PCL_Q30_SHIFT))
                          + ((int64_t)1 << 31))
            >> 32);
        p = (int32_t)(((int64_t)G * p + ((int64_t)s3 * ((int64_t)1 << PCL_Q28_SHIFT))
                          + PCL_CUTOFF_ROUND30)
            >> PCL_Q30_SHIFT);
        p = (int32_t)(((int64_t)G * p + ((int64_t)s4 * ((int64_t)1 << PCL_Q28_SHIFT))
                          + PCL_CUTOFF_ROUND30)
            >> PCL_Q30_SHIFT);
        S = pcl_wide_sat(((int64_t)H * p + PCL_CUTOFF_ROUND28) >> PCL_Q28_SHIFT);

        /* u = a x - (a k) S, in Q8.56: a x is Q6.58, (a k) S is Q8.56 and reaches about 35. */
        u = pcl_wide_sat(
            ((((int64_t)a * x) >> 2) - (int64_t)ak * S + PCL_CUTOFF_ROUND28) >> PCL_Q28_SHIFT);
        if (saturate && (u >= LADDER_TANH_LINEAR || u <= -LADDER_TANH_LINEAR))
            u = pcl_sample_to_wide(pcl_tanh(u));

        /* y = G x + (1 - G) s is a convex combination: within +-8 whenever x and s are. */
        y1 = (pcl_wide_t)(((int64_t)G * u + (int64_t)H * s1 + PCL_CUTOFF_ROUND30) >> PCL_Q30_SHIFT);
        y2 = (pcl_wide_t)(((int64_t)G * y1 + (int64_t)H * s2 + PCL_CUTOFF_ROUND30)
            >> PCL_Q30_SHIFT);
        y3 = (pcl_wide_t)(((int64_t)G * y2 + (int64_t)H * s3 + PCL_CUTOFF_ROUND30)
            >> PCL_Q30_SHIFT);
        y4 = (pcl_wide_t)(((int64_t)G * y3 + (int64_t)H * s4 + PCL_CUTOFF_ROUND30)
            >> PCL_Q30_SHIFT);
        s1 = pcl_wide_sat(2 * (int64_t)y1 - s1);
        s2 = pcl_wide_sat(2 * (int64_t)y2 - s2);
        s3 = pcl_wide_sat(2 * (int64_t)y3 - s3);
        s4 = pcl_wide_sat(2 * (int64_t)y4 - s4);

        switch (mode)
        {
        case PCL_LADDER_LP24:
            m = y4;
            break;
        case PCL_LADDER_LP18:
            m = y3;
            break;
        case PCL_LADDER_LP12:
            m = y2;
            break;
        case PCL_LADDER_LP6:
            m = y1;
            break;
        case PCL_LADDER_BP12:
            m = 2 * ((int64_t)y1 - y2);
            break;
        case PCL_LADDER_BP24:
            m = 4 * ((int64_t)y2 - 2 * (int64_t)y3 + y4);
            break;
        case PCL_LADDER_HP12:
            m = (int64_t)u - 2 * (int64_t)y1 + y2;
            break;
        default:
            m = (int64_t)u - 4 * (int64_t)y1 + 6 * (int64_t)y2 - 4 * (int64_t)y3 + y4;
            break;
        }
        out[i] = pcl_wide64_to_sample(m);
#endif
    }

    ld->s[0] = s1;
    ld->s[1] = s2;
    ld->s[2] = s3;
    ld->s[3] = s4;
    ld->G = G;
    ld->k = k;
    ld->drv = drv;
    ld->a = a;
    ld->ak = ak;
}

/* The same body twice, so the common case (coefficients not moving) carries no ramp state. */
static void ladder_chunk_static(
    pcl_ladder_t *ld, const pcl_sample_t *in, pcl_sample_t *out, size_t n)
{
    ladder_chunk_impl(ld, in, out, n, 0);
}

static void ladder_chunk_ramp(pcl_ladder_t *ld, const pcl_sample_t *in, pcl_sample_t *out, size_t n)
{
    ladder_chunk_impl(ld, in, out, n, 1);
}

void pcl_ladder_process(pcl_ladder_t *ld, const pcl_sample_t *in, pcl_sample_t *out,
    const pcl_sample_t *cutoff_mod, size_t frames)
{
    size_t n = 0;

    while (n < frames)
    {
        size_t chunk = (size_t)PCL_CONTROL_DIV - ld->pos;

        if (chunk > frames - n)
            chunk = frames - n;
        if (ld->pos == 0u)
            ladder_tick(
                ld, cutoff_mod != NULL, cutoff_mod != NULL ? cutoff_mod[n] : (pcl_sample_t)0);

        if (ld->ramping)
            ladder_chunk_ramp(ld, in + n, out + n, chunk);
        else
            ladder_chunk_static(ld, in + n, out + n, chunk);

        ld->pos = (uint8_t)(ld->pos + chunk);
        if (ld->pos >= PCL_CONTROL_DIV)
        {
            /* The period is complete: land exactly on its coefficients, whatever the rounding. */
            ld->pos = 0u;
            ld->G = ld->G_end;
            ld->k = ld->k_end;
            ld->drv = ld->drv_end;
            ld->a = ld->a_end;
            ld->ak = ld->ak_end;
        }
        n += chunk;
    }
}
