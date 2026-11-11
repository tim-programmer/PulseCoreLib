/* Tests for the ladder filter. */
#include <string.h>

#include "pcl_test.h"

#define FS 48000u
#define MAX_FRAMES (4u * 48000u)
#define PI 3.14159265358979323846

static pcl_sample_t g_in[MAX_FRAMES];
static pcl_sample_t g_out[MAX_FRAMES];
static pcl_sample_t g_out2[MAX_FRAMES];
static pcl_sample_t g_mod[MAX_FRAMES];

static pcl_ladder_config_t make_cfg(uint32_t fs, double fc, double res, pcl_ladder_mode_t mode)
{
    pcl_ladder_config_t cfg;
    cfg.sample_rate_hz = fs;
    cfg.cutoff_hz = PCL_HZ(fc);
    cfg.resonance = PCL_COEF(res);
    cfg.mod_depth_oct = PCL_OCT(2.0);
    cfg.mode = mode;
    cfg.saturate = 0;
    cfg.drive = PCL_WIDE(1.0);
    cfg.bass_comp = 0;
    return cfg;
}

/* Runtime values for the config (PCL_HZ etc. need constants in fixed point). */
static pcl_hz_t hz_of(double hz)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return (float)hz;
#else
    return (pcl_hz_t)(hz * 65536.0 + 0.5);
#endif
}

static pcl_coef_t coef_of(double v)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return (float)v;
#else
    return (pcl_coef_t)floor(v * PCL_TEST_Q30_ONE + 0.5);
#endif
}

static pcl_wide_t wide_of(double v)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return (float)v;
#else
    return (pcl_wide_t)floor(v * 268435456.0 + 0.5);
#endif
}

static double wide_to_double(pcl_wide_t v)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return (double)v;
#else
    return (double)v / 268435456.0;
#endif
}

static void make(pcl_ladder_t *ld, uint32_t fs, double fc, double res, pcl_ladder_mode_t mode)
{
    pcl_ladder_config_t cfg = make_cfg(fs, fc, res, mode);
    cfg.cutoff_hz = hz_of(fc);
    cfg.resonance = coef_of(res);
    PCL_CHECK_EQ_INT(pcl_ladder_init(ld, &cfg), PCL_OK);
}

static void fill_noise(pcl_sample_t *buf, size_t n, uint32_t seed, double amplitude)
{
    size_t i;
    for (i = 0; i < n; i++)
        buf[i] = pcl_test_from_double(pcl_test_rand(&seed) * amplitude);
}

static int no_denormals(const pcl_sample_t *x, size_t n)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    size_t i;
    for (i = 0; i < n; i++)
        if (!isfinite(x[i]) || (x[i] != 0.0f && fabsf(x[i]) < FLT_MIN))
            return 0;
#else
    (void)x;
    (void)n;
#endif
    return 1;
}

static double max_state(const pcl_ladder_t *ld)
{
    double m = 0.0;
    int    i;
    for (i = 0; i < 4; i++)
    {
        const double a = fabs(wide_to_double(ld->s[i]));
        if (a > m)
            m = a;
    }
    return m;
}

/* ---- analytic response ---- */

typedef struct
{
    double re, im;
} cplx;

static cplx c_mul(cplx a, cplx b)
{
    cplx r;
    r.re = a.re * b.re - a.im * b.im;
    r.im = a.re * b.im + a.im * b.re;
    return r;
}

static cplx c_div(cplx a, cplx b)
{
    const double d = b.re * b.re + b.im * b.im;
    cplx         r;
    r.re = (a.re * b.re + a.im * b.im) / d;
    r.im = (a.im * b.re - a.re * b.im) / d;
    return r;
}

static cplx c_add(cplx a, cplx b)
{
    cplx r;
    r.re = a.re + b.re;
    r.im = a.im + b.im;
    return r;
}

static cplx c_scale(cplx a, double s)
{
    cplx r;
    r.re = a.re * s;
    r.im = a.im * s;
    return r;
}

/* Exact response of the linear filter: stage H1 = 1/(1 + j tan(pi f/fs)/g), U = X/(1 + k H1^4). */
static cplx ladder_response(
    double f, double fc, double fs, double res, pcl_ladder_mode_t mode, int bass_comp)
{
    const double k = 4.0 * res;
    const cplx   one = { 1.0, 0.0 };
    cplx         h1, h[5], m;
    int          i;

    h1.re = 1.0;
    h1.im = tan(PI * f / fs) / tan(PI * fc / fs);
    h1 = c_div(one, h1);
    h[0] = one;
    for (i = 1; i <= 4; i++)
        h[i] = c_mul(h[i - 1], h1);
    {
        const cplx den = c_add(one, c_scale(h[4], k));
        const cplx u = c_div(bass_comp ? c_scale(one, 1.0 + k) : one, den);
        for (i = 0; i <= 4; i++)
            h[i] = c_mul(h[i], u);
    }
    switch (mode)
    {
    case PCL_LADDER_LP24:
        return h[4];
    case PCL_LADDER_LP18:
        return h[3];
    case PCL_LADDER_LP12:
        return h[2];
    case PCL_LADDER_LP6:
        return h[1];
    case PCL_LADDER_BP12:
        return c_scale(c_add(h[1], c_scale(h[2], -1.0)), 2.0);
    case PCL_LADDER_BP24:
        return c_scale(c_add(c_add(h[2], c_scale(h[3], -2.0)), h[4]), 4.0);
    case PCL_LADDER_HP12:
        return c_add(c_add(h[0], c_scale(h[1], -2.0)), h[2]);
    default:
        m = c_add(h[0], c_scale(h[1], -4.0));
        m = c_add(m, c_scale(h[2], 6.0));
        m = c_add(m, c_scale(h[3], -4.0));
        return c_add(m, h[4]);
    }
}

/* Complex amplitude of x at an integer frequency over `n` samples (one second: whole cycles). */
static cplx phasor(const pcl_sample_t *x, size_t n, double f, double fs)
{
    cplx   r = { 0.0, 0.0 };
    size_t i;
    for (i = 0; i < n; i++)
    {
        const double v = pcl_test_to_double(x[i]);
        const double ph = 2.0 * PI * f * (double)i / fs;
        r.re += v * cos(ph);
        r.im -= v * sin(ph);
    }
    return c_scale(r, 2.0 / (double)n);
}

/* ---- validation ---- */

static void test_init_validation(void)
{
    pcl_ladder_t        ld, before;
    pcl_ladder_config_t cfg = make_cfg(FS, 1000.0, 0.5, PCL_LADDER_LP24);

    PCL_CHECK_EQ_INT(pcl_ladder_init(&ld, &cfg), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_ladder_init(NULL, &cfg), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_ladder_init(&ld, NULL), PCL_ERR_NULL_POINTER);
    before = ld;

    cfg.sample_rate_hz = PCL_SAMPLE_RATE_MIN_HZ - 1u;
    PCL_CHECK_EQ_INT(pcl_ladder_init(&ld, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.sample_rate_hz = PCL_SAMPLE_RATE_MAX_HZ + 1u;
    PCL_CHECK_EQ_INT(pcl_ladder_init(&ld, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.sample_rate_hz = FS;

    cfg.cutoff_hz = PCL_HZ(4.9);
    PCL_CHECK_EQ_INT(pcl_ladder_init(&ld, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.cutoff_hz = PCL_HZ(21601.0);
    PCL_CHECK_EQ_INT(pcl_ladder_init(&ld, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.cutoff_hz = PCL_HZ(1000.0);

    cfg.resonance = PCL_COEF(-0.01);
    PCL_CHECK_EQ_INT(pcl_ladder_init(&ld, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.resonance = PCL_COEF(1.11);
    PCL_CHECK_EQ_INT(pcl_ladder_init(&ld, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.resonance = PCL_COEF(0.5);

    cfg.mod_depth_oct = PCL_OCT(-0.1);
    PCL_CHECK_EQ_INT(pcl_ladder_init(&ld, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.mod_depth_oct = PCL_OCT(2.0);

    cfg.mode = PCL_LADDER_MODE_COUNT;
    PCL_CHECK_EQ_INT(pcl_ladder_init(&ld, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.mode = PCL_LADDER_LP24;

    cfg.drive = PCL_WIDE(0.99);
    PCL_CHECK_EQ_INT(pcl_ladder_init(&ld, &cfg), PCL_ERR_OUT_OF_RANGE);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    cfg.drive = 8.0f;
    PCL_CHECK_EQ_INT(pcl_ladder_init(&ld, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.drive = NAN;
    PCL_CHECK_EQ_INT(pcl_ladder_init(&ld, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.drive = 1.0f;
    cfg.cutoff_hz = NAN;
    PCL_CHECK_EQ_INT(pcl_ladder_init(&ld, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.cutoff_hz = 1000.0f;
    cfg.resonance = NAN;
    PCL_CHECK_EQ_INT(pcl_ladder_init(&ld, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.resonance = 0.5f;
    cfg.mod_depth_oct = 8.0f;
    PCL_CHECK_EQ_INT(pcl_ladder_init(&ld, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.mod_depth_oct = 2.0f;
#endif
    cfg.drive = PCL_WIDE(1.0);
    PCL_CHECK(memcmp(&ld, &before, sizeof ld) == 0);

    /* Range edges. */
    cfg.cutoff_hz = PCL_HZ(5.0);
    cfg.resonance = PCL_COEF(0.0);
    PCL_CHECK_EQ_INT(pcl_ladder_init(&ld, &cfg), PCL_OK);
    cfg.cutoff_hz = PCL_HZ(21599.0);
    cfg.resonance = PCL_COEF(1.1);
    cfg.drive = PCL_WIDE(7.99);
    cfg.mode = PCL_LADDER_HP24;
    PCL_CHECK_EQ_INT(pcl_ladder_init(&ld, &cfg), PCL_OK);
}

static void test_setter_validation(void)
{
    pcl_ladder_t ld, before;

    make(&ld, FS, 1000.0, 0.5, PCL_LADDER_LP24);
    before = ld;
    PCL_CHECK_EQ_INT(pcl_ladder_set_cutoff(NULL, PCL_HZ(100.0)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_ladder_set_cutoff(&ld, PCL_HZ(3.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_ladder_set_cutoff(&ld, PCL_HZ(22000.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_ladder_set_resonance(NULL, PCL_COEF(0.1)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_ladder_set_resonance(&ld, PCL_COEF(1.2)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_ladder_set_mod_depth(NULL, PCL_OCT(1.0)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_ladder_set_mod_depth(&ld, PCL_OCT(-1.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_ladder_set_mode(NULL, PCL_LADDER_LP12), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_ladder_set_mode(&ld, PCL_LADDER_MODE_COUNT), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_ladder_set_drive(NULL, PCL_WIDE(2.0)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_ladder_set_drive(&ld, PCL_WIDE(0.5)), PCL_ERR_OUT_OF_RANGE);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    PCL_CHECK_EQ_INT(pcl_ladder_set_cutoff(&ld, NAN), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_ladder_set_resonance(&ld, NAN), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_ladder_set_drive(&ld, NAN), PCL_ERR_OUT_OF_RANGE);
#endif
    PCL_CHECK(memcmp(&ld, &before, sizeof ld) == 0);

    PCL_CHECK_EQ_INT(pcl_ladder_set_cutoff(&ld, PCL_HZ(200.0)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_ladder_set_resonance(&ld, PCL_COEF(1.1)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_ladder_set_mod_depth(&ld, PCL_OCT(7.0)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_ladder_set_mode(&ld, PCL_LADDER_BP24), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_ladder_set_drive(&ld, PCL_WIDE(4.0)), PCL_OK);
    pcl_ladder_set_saturate(&ld, 1);
    pcl_ladder_set_bass_comp(&ld, 1);
}

/* ---- response ---- */

static double g_worst_db, g_worst_rad;

/* Measures one point and checks it against the analytic response. */
static void response_case(uint32_t fs, double fc, double res, pcl_ladder_mode_t mode, int bass_comp,
    double f, double amp_max)
{
    const size_t settle = fs, n = fs; /* one second each: f is an integer, so whole cycles */
    pcl_ladder_t ld;
    size_t       i;
    cplx         x, y, h, want;
    double       err_db, err_rad, floor_lin;

    double amp;

    want = ladder_response(f, fc, fs, res, mode, bass_comp);
    /* Keep the output below full scale: fixed point saturates there, float does not. */
    amp = 0.7 / sqrt(want.re * want.re + want.im * want.im);
    if (amp > amp_max)
        amp = amp_max;
    make(&ld, fs, fc, res, mode);
    pcl_ladder_set_bass_comp(&ld, bass_comp);
    for (i = 0; i < settle + n; i++)
        g_in[i] = pcl_test_from_double(amp * sin(2.0 * PI * f * (double)i / (double)fs));
    pcl_ladder_process(&ld, g_in, g_out, NULL, settle + n);

    x = phasor(g_in + settle, n, f, fs);
    y = phasor(g_out + settle, n, f, fs);
    h = c_div(y, x);

    {
        const double mw = sqrt(want.re * want.re + want.im * want.im);
        const double mh = sqrt(h.re * h.re + h.im * h.im);
        /* Below the measurement floor (a few LSB of output) only the absolute level is checked. */
#if PCL_SAMPLE_TYPE == PCL_Q15
        floor_lin = 3e-3;
#else
        floor_lin = 3e-6;
#endif
        if (mw * amp < floor_lin * 10.0)
        {
            PCL_CHECK(fabs(mh - mw) * amp < floor_lin * 2.0);
            return;
        }
        err_db = fabs(20.0 * log10(mh / mw));
        {
            const cplx r = c_div(h, want);
            err_rad = fabs(atan2(r.im, r.re));
        }
        if (err_db > g_worst_db)
            g_worst_db = err_db;
        if (err_rad > g_worst_rad)
            g_worst_rad = err_rad;
#if PCL_SAMPLE_TYPE == PCL_Q15
        PCL_CHECK(err_db < 0.05 + 20.0 * log10(1.0 + floor_lin / (mw * amp)));
        PCL_CHECK(err_rad < 0.01 + floor_lin / (mw * amp));
#else
        PCL_CHECK(err_db < 0.01);
        PCL_CHECK(err_rad < 0.001);
#endif
        if (err_db > 0.05 && PCL_SAMPLE_TYPE != PCL_Q15)
            printf("    fs %u fc %g r %g mode %d f %g: %.4f dB\n", (unsigned)fs, fc, res, (int)mode,
                f, err_db);
    }
}

static void test_response_matches_analytic(void)
{
    static const double   fcs[] = { 100.0, 1000.0, 8000.0 };
    static const double   rs[] = { 0.0, 0.5, 0.9 };
    static const double   ratios[] = { 0.25, 0.5, 1.0, 2.0, 4.0 };
    static const uint32_t rates[] = { 48000u, 96000u };
    size_t                r, c, q, m, f;

    g_worst_db = 0.0;
    g_worst_rad = 0.0;
    for (r = 0; r < 2; r++)
        for (c = 0; c < 3; c++)
            for (q = 0; q < 3; q++)
                for (m = 0; m < (size_t)PCL_LADDER_MODE_COUNT; m++)
                    for (f = 0; f < 5; f++)
                    {
                        const double freq = floor(fcs[c] * ratios[f] + 0.5);
                        if (freq >= 0.45 * (double)rates[r])
                            continue;
                        response_case(rates[r], fcs[c], rs[q], (pcl_ladder_mode_t)m, 0, freq, 0.2);
                    }
    printf("    response: worst %.5f dB, %.5f rad\n", g_worst_db, g_worst_rad);
}

/* The low-pass DC gain is 1 / (1 + k) without compensation and 1 with it. */
static void test_bass_loss_and_compensation(void)
{
    static const double rs[] = { 0.0, 0.5, 0.9 };
    size_t              q;

    for (q = 0; q < 3; q++)
    {
        int comp;
        for (comp = 0; comp <= 1; comp++)
        {
            pcl_ladder_t ld;
            size_t       i;
            double       sum = 0.0;
            const double want = comp ? 1.0 : 1.0 / (1.0 + 4.0 * rs[q]);

            make(&ld, FS, 2000.0, rs[q], PCL_LADDER_LP24);
            pcl_ladder_set_bass_comp(&ld, comp);
            for (i = 0; i < FS; i++)
                g_in[i] = pcl_test_from_double(0.15);
            pcl_ladder_process(&ld, g_in, g_out, NULL, FS);
            for (i = FS / 2u; i < FS; i++)
                sum += pcl_test_to_double(g_out[i]);
            PCL_CHECK_NEAR(sum / (double)(FS / 2u) / pcl_test_to_double(g_in[0]), want, 2e-3);
        }
    }
}

/* Zero-crossing frequency of x[from .. to). */
static double crossing_hz(const pcl_sample_t *x, size_t from, size_t to, double fs)
{
    double first = -1.0, last = -1.0;
    size_t count = 0, i;
    for (i = from + 1u; i < to; i++)
    {
        const double a = pcl_test_to_double(x[i - 1u]), b = pcl_test_to_double(x[i]);
        if (a < 0.0 && b >= 0.0)
        {
            const double t = (double)(i - 1u) + a / (a - b);
            if (first < 0.0)
                first = t;
            last = t;
            count++;
        }
    }
    return count > 1u ? (double)(count - 1u) * fs / (last - first) : 0.0;
}

static double peak(const pcl_sample_t *x, size_t from, size_t to)
{
    double m = 0.0;
    size_t i;
    for (i = from; i < to; i++)
        if (fabs(pcl_test_to_double(x[i])) > m)
            m = fabs(pcl_test_to_double(x[i]));
    return m;
}

/* At resonance 1 the linear filter rings at exactly the cutoff (the zero-delay solution). */
static void test_resonance_one_rings_at_cutoff(void)
{
    static const double fcs[] = { 440.0, 1234.0, 5000.0 };
    size_t              c;

    for (c = 0; c < 3; c++)
    {
        pcl_ladder_t ld;
        double       hz, cents, p1, p2;

        make(&ld, FS, fcs[c], 1.0, PCL_LADDER_LP24);
        memset(g_in, 0, sizeof g_in);
        g_in[0] = pcl_test_from_double(0.9);
        pcl_ladder_process(&ld, g_in, g_out, NULL, 2u * FS);
        hz = crossing_hz(g_out, FS / 10u, 2u * FS, FS);
        cents = 1200.0 * log2(hz / fcs[c]);
        p1 = peak(g_out, FS / 10u, FS / 5u);
        p2 = peak(g_out, 2u * FS - FS / 10u, 2u * FS);
        printf("    r = 1, fc %g: rings at %.4f Hz (%.4f cents), level %.4g -> %.4g\n", fcs[c], hz,
            cents, p1, p2);
        PCL_CHECK(fabs(cents) < 0.5);
#if PCL_SAMPLE_TYPE != PCL_Q15
        /* Marginal stability: the ring neither grows nor dies within 2 s (to 10 %). */
        PCL_CHECK(p2 > 0.9 * p1 && p2 < 1.1 * p1);
#else
        PCL_CHECK(p2 > 0.5 * p1 && p2 < 1.5 * p1);
#endif
    }
}

/* Above 1, with saturate, the filter oscillates by itself at a steady level near the cutoff. */
static void test_self_oscillation_with_saturate(void)
{
    static const double fcs[] = { 220.0, 1000.0, 4000.0 };
    size_t              c;

    for (c = 0; c < 3; c++)
    {
        pcl_ladder_t ld;
        double       hz, cents, p1, p2;

        make(&ld, FS, fcs[c], 1.1, PCL_LADDER_LP24);
        pcl_ladder_set_saturate(&ld, 1);
        memset(g_in, 0, sizeof g_in);
        g_in[0] = pcl_test_from_double(0.001); /* a tiny kick: it must grow by itself */
        pcl_ladder_process(&ld, g_in, g_out, NULL, 3u * FS);
        hz = crossing_hz(g_out, 2u * FS, 3u * FS, FS);
        cents = 1200.0 * log2(hz / fcs[c]);
        p1 = peak(g_out, 2u * FS, 2u * FS + FS / 4u);
        p2 = peak(g_out, 3u * FS - FS / 4u, 3u * FS);
        printf("    r = 1.1 saturated, fc %g: %.3f Hz (%.2f cents), level %.4f / %.4f\n", fcs[c],
            hz, cents, p1, p2);
        /* tanh settles where its gain is 4/4.4, so u is about 0.55 and LP24 = u/4 at fc. */
        PCL_CHECK(p1 > 0.1 && p1 < 0.2);
        PCL_CHECK(fabs(p2 - p1) < 0.02 * p1);
        PCL_CHECK(fabs(cents) < 1.0);
        PCL_CHECK(max_state(&ld) < 2.0);
    }
}

/* ---- double-precision model of the same algorithm, static coefficients ---- */

typedef struct
{
    double s[4], G, k, a, drive;
    int    saturate, comp, mode;
} model_t;

static double model_step(model_t *m, double x)
{
    const double G = m->G, H = 1.0 - G;
    double       S, u, y[5];
    int          i;
    if (m->comp)
        x *= 1.0 + m->k;
    if (m->saturate)
        x *= m->drive;
    S = H * (G * (G * (G * m->s[0] + m->s[1]) + m->s[2]) + m->s[3]);
    u = m->a * (x - m->k * S);
    if (m->saturate && fabs(u) >= 1.0 / 64.0)
        u = tanh(u);
    y[0] = u;
    for (i = 0; i < 4; i++)
    {
        y[i + 1] = G * y[i] + H * m->s[i];
        m->s[i] = 2.0 * y[i + 1] - m->s[i];
    }
    switch (m->mode)
    {
    case PCL_LADDER_LP24:
        return y[4];
    case PCL_LADDER_BP12:
        return 2.0 * (y[1] - y[2]);
    case PCL_LADDER_HP24:
        return y[0] - 4.0 * y[1] + 6.0 * y[2] - 4.0 * y[3] + y[4];
    default:
        return y[2];
    }
}

static void model_case(double fc, double res, pcl_ladder_mode_t mode, int saturate, double drive,
    int comp, double amp, double tol)
{
    const size_t n = FS / 2u;
    pcl_ladder_t ld;
    model_t      m;
    double       worst = 0.0;
    size_t       i;

    make(&ld, FS, fc, res, mode);
    pcl_ladder_set_saturate(&ld, saturate);
    pcl_ladder_set_bass_comp(&ld, comp);
    if (drive != 1.0)
    {
        pcl_ladder_config_t cfg = make_cfg(FS, 1000.0, 0.5, mode);
        cfg.cutoff_hz = hz_of(fc);
        cfg.resonance = coef_of(res);
        cfg.saturate = saturate;
        cfg.bass_comp = comp;
        cfg.drive = wide_of(drive);
        PCL_CHECK_EQ_INT(pcl_ladder_init(&ld, &cfg), PCL_OK);
    }
    /* The filter's own G and k (white-box), so this checks the per-sample arithmetic; the tuning
       is the response test's job. a is recomputed exactly. */
    memset(&m, 0, sizeof m);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    m.G = (double)ld.G;
    m.k = (double)ld.k;
#else
    m.G = (double)ld.G / PCL_TEST_Q30_ONE;
    m.k = wide_to_double(ld.k);
#endif
    m.a = 1.0 / (1.0 + m.k * pow(m.G, 4.0));
    PCL_CHECK_NEAR(m.G, tan(PI * fc / FS) / (1.0 + tan(PI * fc / FS)), 5e-5 * m.G);
    m.drive = drive;
    m.saturate = saturate;
    m.comp = comp;
    m.mode = (int)mode;

    fill_noise(g_in, n, 99u, amp);
    pcl_ladder_process(&ld, g_in, g_out, NULL, n);
    for (i = 0; i < n; i++)
    {
        const double want = pcl_test_clip(model_step(&m, pcl_test_to_double(g_in[i])));
        const double err = fabs(pcl_test_to_double(g_out[i]) - want);
        if (err > worst)
            worst = err;
    }
    printf("    model fc %g r %g mode %d sat %d drive %g comp %d: worst %.3g\n", fc, res, (int)mode,
        saturate, drive, comp, worst);
    PCL_CHECK(worst < tol);
}

static void test_matches_double_model(void)
{
    /* About twice the worst measured error. Q15: half an LSB of output rounding (1.5e-5) plus u
       rounded to 16 bits when saturated; saturated: pcl_tanh is within 2.3e-5 of libm. */
#if PCL_SAMPLE_TYPE == PCL_Q15
    const double tol = 3.1e-5, tol_sat = 7e-5;
#elif PCL_SAMPLE_TYPE == PCL_Q31
    const double tol = 4e-7, tol_sat = 1.5e-5;
#else
    const double tol = 2e-6, tol_sat = 1.5e-5;
#endif
    model_case(300.0, 0.0, PCL_LADDER_LP24, 0, 1.0, 0, 0.5, tol);
    model_case(1500.0, 0.7, PCL_LADDER_LP24, 0, 1.0, 0, 0.3, tol);
    model_case(6000.0, 0.95, PCL_LADDER_BP12, 0, 1.0, 0, 0.2, tol);
    model_case(800.0, 0.5, PCL_LADDER_HP24, 0, 1.0, 1, 0.2, tol);
    model_case(18000.0, 0.8, PCL_LADDER_LP12, 0, 1.0, 0, 0.3, tol);
    /* Saturated: pcl_tanh is a table (2.3e-5 from libm), and Q15 rounds u to 16 bits. */
    model_case(1000.0, 0.9, PCL_LADDER_LP24, 1, 3.0, 0, 0.5, tol_sat);
    model_case(500.0, 1.05, PCL_LADDER_LP24, 1, 1.0, 1, 0.3, tol_sat);
}

/* ---- modulation ---- */

/* A constant modulation of +m octaves equals a filter set m octaves up (after the first tick). */
static void test_constant_modulation_equals_higher_cutoff(void)
{
    pcl_ladder_t a, b;
    size_t       i;
    double       worst = 0.0;

    make(&a, FS, 500.0, 0.6, PCL_LADDER_LP24);
    make(&b, FS, 1000.0, 0.6, PCL_LADDER_LP24);
    PCL_CHECK_EQ_INT(pcl_ladder_set_mod_depth(&a, PCL_OCT(2.0)), PCL_OK);
    for (i = 0; i < FS; i++)
        g_mod[i] = pcl_test_from_double(0.5); /* +1 octave */
    fill_noise(g_in, FS, 5u, 0.3);
    pcl_ladder_process(&a, g_in, g_out, g_mod, FS);
    pcl_ladder_process(&b, g_in, g_out2, NULL, FS);
    for (i = FS / 10u; i < FS; i++)
    {
        const double e = fabs(pcl_test_to_double(g_out[i]) - pcl_test_to_double(g_out2[i]));
        if (e > worst)
            worst = e;
    }
    printf("    +1 octave by modulation vs set: worst %.3g\n", worst);
    PCL_CHECK(worst < 2e-4);
}

/* When modulation stops, the coefficients come back to the unmodulated ones. */
static void test_cutoff_returns_after_modulation(void)
{
    pcl_ladder_t a, b;

    make(&a, FS, 700.0, 0.4, PCL_LADDER_LP24);
    make(&b, FS, 700.0, 0.4, PCL_LADDER_LP24);
    fill_noise(g_mod, 4096, 6u, 1.0);
    fill_noise(g_in, 4096, 7u, 0.3);
    pcl_ladder_process(&a, g_in, g_out, g_mod, 4096);
    pcl_ladder_process(&a, g_in, g_out, NULL, 64);
    PCL_CHECK(a.G == b.G && a.a == b.a && a.ak == b.ak && a.k == b.k);
    PCL_CHECK(!a.ramping && !a.mod_dirty);
}

/* Coefficients land exactly on their targets at the end of every period. */
static void test_coefficients_snap_at_period_end(void)
{
    pcl_ladder_t ld;
    size_t       p;

    make(&ld, FS, 300.0, 0.8, PCL_LADDER_LP24);
    fill_noise(g_mod, 4096, 8u, 1.0);
    fill_noise(g_in, 4096, 9u, 0.3);
    for (p = 0; p < 50; p++)
    {
        pcl_ladder_process(
            &ld, g_in + p * PCL_CONTROL_DIV, g_out, g_mod + p * PCL_CONTROL_DIV, PCL_CONTROL_DIV);
        PCL_CHECK(ld.G == ld.G_end && ld.a == ld.a_end && ld.k == ld.k_end && ld.ak == ld.ak_end);
    }
}

/*
 * The SVF's regression case: random jumps of up to +-4 or +-6 octaves every control period must
 * stay bounded (a linearly interpolated 1/d blows up here).
 */
static void test_large_cutoff_jumps_stay_bounded(void)
{
    static const double depths[] = { 4.0, 6.0 };
    static const double rs[] = { 0.0, 0.5 };
    size_t              d, q;

    for (d = 0; d < 2; d++)
        for (q = 0; q < 2; q++)
        {
            pcl_ladder_t ld;
            size_t       i, block;
            double       worst = 0.0;

            make(&ld, FS, 1000.0, rs[q], PCL_LADDER_LP24);
            PCL_CHECK_EQ_INT(pcl_ladder_set_mod_depth(&ld, wide_of(depths[d])), PCL_OK);
            for (block = 0; block < 10u; block++)
            {
                uint32_t seed = 1000u + (uint32_t)block;
                fill_noise(g_in, FS, 77u + (uint32_t)block, block < 5u ? 0.5 : 0.0);
                for (i = 0; i < FS; i++)
                {
                    if (i % PCL_CONTROL_DIV == 0u)
                        g_mod[i] = pcl_test_from_double(pcl_test_rand(&seed));
                    else
                        g_mod[i] = g_mod[i - 1u];
                }
                pcl_ladder_process(&ld, g_in, g_out, g_mod, FS);
                if (max_state(&ld) > worst)
                    worst = max_state(&ld);
                for (i = 0; i < FS; i++)
                    if (fabs(pcl_test_to_double(g_out[i])) > worst)
                        worst = fabs(pcl_test_to_double(g_out[i]));
            }
            printf("    jumps +-%g oct, r %g: peak %.3f\n", depths[d], rs[q], worst);
            PCL_CHECK(worst < 2.5);
        }
}

/* ---- determinism, blocking, aliasing ---- */

static void render_blocked(size_t block, pcl_sample_t *out, int use_mod)
{
    pcl_ladder_t ld;
    size_t       pos = 0;
    const size_t total = FS / 2u, change_at = 5000;

    make(&ld, FS, 800.0, 0.85, PCL_LADDER_LP24);
    pcl_ladder_set_saturate(&ld, 1);
    while (pos < total)
    {
        size_t n = total - pos < block ? total - pos : block;
        if (pos < change_at && pos + n > change_at)
            n = change_at - pos;
        if (pos == change_at)
        {
            PCL_CHECK_EQ_INT(pcl_ladder_set_cutoff(&ld, PCL_HZ(3000.0)), PCL_OK);
            PCL_CHECK_EQ_INT(pcl_ladder_set_resonance(&ld, PCL_COEF(0.3)), PCL_OK);
            PCL_CHECK_EQ_INT(pcl_ladder_set_drive(&ld, PCL_WIDE(2.5)), PCL_OK);
            PCL_CHECK_EQ_INT(pcl_ladder_set_mode(&ld, PCL_LADDER_BP24), PCL_OK);
        }
        pcl_ladder_process(&ld, g_in + pos, out + pos, use_mod ? g_mod + pos : NULL, n);
        pos += n;
    }
}

static void test_block_size_independence(void)
{
    static const size_t blocks[] = { 1, 7, 16, 17, 256 };
    int                 use_mod;

    fill_noise(g_in, FS, 10u, 0.5);
    fill_noise(g_mod, FS, 11u, 1.0);
    for (use_mod = 0; use_mod <= 1; use_mod++)
    {
        size_t b;
        render_blocked(FS, g_out, use_mod);
        for (b = 0; b < sizeof blocks / sizeof blocks[0]; b++)
        {
            size_t i;
            int    same = 1;
            render_blocked(blocks[b], g_out2, use_mod);
            for (i = 0; i < FS / 2u; i++)
                same &= g_out[i] == g_out2[i];
            PCL_CHECK(same);
        }
    }
}

static void test_reset_aliasing_and_zero_mod(void)
{
    pcl_ladder_t ld;
    size_t       i;
    int          same = 1;
    const size_t n = 8000;

    fill_noise(g_in, n, 12u, 0.5);
    fill_noise(g_mod, n, 13u, 1.0);
    make(&ld, FS, 1200.0, 0.7, PCL_LADDER_HP12);
    pcl_ladder_process(&ld, g_in, g_out, g_mod, n);
    pcl_ladder_reset(&ld);
    pcl_ladder_process(&ld, g_in, g_out2, g_mod, n);
    for (i = 0; i < n; i++)
        same &= g_out[i] == g_out2[i];
    PCL_CHECK(same);

    /* In place. */
    pcl_ladder_reset(&ld);
    memcpy(g_out2, g_in, n * sizeof g_in[0]);
    pcl_ladder_process(&ld, g_out2, g_out2, g_mod, n);
    same = 1;
    for (i = 0; i < n; i++)
        same &= g_out[i] == g_out2[i];
    PCL_CHECK(same);

    /* A modulation input of zeros equals none. */
    memset(g_mod, 0, n * sizeof g_mod[0]);
    pcl_ladder_reset(&ld);
    pcl_ladder_process(&ld, g_in, g_out, g_mod, n);
    pcl_ladder_reset(&ld);
    pcl_ladder_process(&ld, g_in, g_out2, NULL, n);
    same = 1;
    for (i = 0; i < n; i++)
        same &= g_out[i] == g_out2[i];
    PCL_CHECK(same);
}

/* ---- extremes ---- */

static void test_extremes_stay_finite_and_decay(void)
{
    static const pcl_ladder_mode_t modes[] = { PCL_LADDER_LP24, PCL_LADDER_HP24, PCL_LADDER_BP24 };
    size_t                         c;

    for (c = 0; c < 3; c++)
    {
        pcl_ladder_t        ld;
        pcl_ladder_config_t cfg = make_cfg(FS, 2000.0, 1.1, modes[c]);
        size_t              s;
        double              worst = 0.0;

        cfg.mod_depth_oct = PCL_OCT(7.5);
        cfg.drive = PCL_WIDE(7.9);
        cfg.bass_comp = 1;
        cfg.saturate = (int)(c == 2u);
        PCL_CHECK_EQ_INT(pcl_ladder_init(&ld, &cfg), PCL_OK);
        for (s = 0; s < 5u; s++)
        {
            fill_noise(g_in, FS, 20u + (uint32_t)s, 1.0);
            fill_noise(g_mod, FS, 30u + (uint32_t)s, 1.0);
            pcl_ladder_process(&ld, g_in, g_out, g_mod, FS);
            PCL_CHECK(no_denormals(g_out, FS));
            if (max_state(&ld) > worst)
                worst = max_state(&ld);
        }
        PCL_CHECK(worst <= 8.0);

        /* Then silence at resonance 0: decays to the floor, no denormals on the way. */
        PCL_CHECK_EQ_INT(pcl_ladder_set_resonance(&ld, PCL_COEF(0.0)), PCL_OK);
        memset(g_in, 0, sizeof g_in);
        for (s = 0; s < 4u; s++)
        {
            pcl_ladder_process(&ld, g_in, g_out, NULL, FS);
            PCL_CHECK(no_denormals(g_out, FS));
        }
        PCL_CHECK(peak(g_out, 0, FS) < 1e-4);
    }
}

static void test_saturate_limits_level(void)
{
    pcl_ladder_t        ld;
    pcl_ladder_config_t cfg = make_cfg(FS, 5000.0, 0.0, PCL_LADDER_LP24);
    size_t              i;

    cfg.saturate = 1;
    cfg.drive = PCL_WIDE(6.0);
    PCL_CHECK_EQ_INT(pcl_ladder_init(&ld, &cfg), PCL_OK);
    for (i = 0; i < FS; i++)
        g_in[i] = pcl_test_from_double(0.9 * sin(2.0 * PI * 100.0 * (double)i / FS));
    pcl_ladder_process(&ld, g_in, g_out, NULL, FS);
    /* Without saturation the output would be ~5.4 (drive 6 x 0.9). */
    PCL_CHECK(peak(g_out, FS / 2u, FS) < 1.05);
    PCL_CHECK(peak(g_out, FS / 2u, FS) > 0.9);
}

int main(void)
{
    PCL_RUN_TEST(test_init_validation);
    PCL_RUN_TEST(test_setter_validation);
    PCL_RUN_TEST(test_response_matches_analytic);
    PCL_RUN_TEST(test_bass_loss_and_compensation);
    PCL_RUN_TEST(test_resonance_one_rings_at_cutoff);
    PCL_RUN_TEST(test_self_oscillation_with_saturate);
    PCL_RUN_TEST(test_matches_double_model);
    PCL_RUN_TEST(test_constant_modulation_equals_higher_cutoff);
    PCL_RUN_TEST(test_cutoff_returns_after_modulation);
    PCL_RUN_TEST(test_coefficients_snap_at_period_end);
    PCL_RUN_TEST(test_large_cutoff_jumps_stay_bounded);
    PCL_RUN_TEST(test_block_size_independence);
    PCL_RUN_TEST(test_reset_aliasing_and_zero_mod);
    PCL_RUN_TEST(test_extremes_stay_finite_and_decay);
    PCL_RUN_TEST(test_saturate_limits_level);
    return pcl_test_summary("ladder");
}
