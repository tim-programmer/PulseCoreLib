/* Tests for the delay. */
#include <string.h>

#include "pcl_test.h"

#define FS 48000u
#define MAX_FRAMES (2u * 48000u)
#define PI 3.14159265358979323846
#define MEM_BYTES (2u * 48000u * 4u + 64u) /* 2 s of native float/Q31 line, plus slack */

static unsigned char g_mem[MEM_BYTES];
static unsigned char g_mem2[MEM_BYTES];
static pcl_sample_t  g_in[MAX_FRAMES];
static pcl_sample_t  g_out[MAX_FRAMES];
static pcl_sample_t  g_out2[MAX_FRAMES];
static pcl_sample_t  g_mod[MAX_FRAMES];

/* Runtime conversions (PCL_MS etc. need constants in fixed point). */
static pcl_ms_t ms_of(double ms)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return (float)ms;
#else
    return (pcl_ms_t)(ms * 65536.0 + 0.5);
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

static pcl_delay_config_t make_cfg(uint32_t max_ms, double time_ms)
{
    pcl_delay_config_t cfg;
    cfg.sample_rate_hz = FS;
    cfg.max_delay_ms = max_ms;
    cfg.time_ms = ms_of(time_ms);
    cfg.mod_depth_ms = PCL_MS(0.0);
    cfg.glide_ms = 0;
    cfg.feedback = PCL_COEF(0.0);
    cfg.tone_hz = PCL_HZ(0.0);
    cfg.saturate = 0;
    cfg.mix = PCL_COEF(1.0);
    cfg.interp = PCL_DELAY_LINEAR;
    cfg.storage = PCL_DELAY_STORE_NATIVE;
    return cfg;
}

static void init_on(pcl_delay_t *dl, const pcl_delay_config_t *cfg, unsigned char *mem)
{
    const size_t need = pcl_delay_required_bytes(cfg);
    PCL_CHECK(need > 0u && need <= MEM_BYTES);
    PCL_CHECK_EQ_INT(pcl_delay_init(dl, cfg, mem, need), PCL_OK);
}

static void fill_noise(pcl_sample_t *buf, size_t n, uint32_t seed, double amplitude)
{
    size_t i;
    for (i = 0; i < n; i++)
        buf[i] = pcl_test_from_double(pcl_test_rand(&seed) * amplitude);
}

static int finite_no_denormals(const pcl_sample_t *x, size_t n)
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

static double peak(const pcl_sample_t *x, size_t from, size_t to)
{
    double m = 0.0;
    size_t i;
    for (i = from; i < to; i++)
        if (fabs(pcl_test_to_double(x[i])) > m)
            m = fabs(pcl_test_to_double(x[i]));
    return m;
}

/* ---- validation ---- */

static void test_sizing_and_init_validation(void)
{
    pcl_delay_t        dl, before;
    pcl_delay_config_t cfg = make_cfg(100, 10.0);
    size_t             need = pcl_delay_required_bytes(&cfg);

    /* 4800 samples + 4 slack, plus alignment slack. */
    PCL_CHECK_EQ_INT(need, (4800u + 4u) * sizeof(pcl_sample_t) + sizeof(pcl_sample_t) - 1u);
    cfg.storage = PCL_DELAY_STORE_Q15;
    PCL_CHECK_EQ_INT(pcl_delay_required_bytes(&cfg), (4800u + 4u) * 2u + 1u);
    cfg.storage = PCL_DELAY_STORE_NATIVE;
    PCL_CHECK_EQ_INT(pcl_delay_required_bytes(NULL), 0);

    PCL_CHECK_EQ_INT(pcl_delay_init(&dl, &cfg, g_mem, need), PCL_OK);
    before = dl;
    PCL_CHECK_EQ_INT(pcl_delay_init(NULL, &cfg, g_mem, need), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_delay_init(&dl, NULL, g_mem, need), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_delay_init(&dl, &cfg, NULL, need), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_delay_init(&dl, &cfg, g_mem, need - 1u), PCL_ERR_BUFFER_TOO_SMALL);

#define BAD(field, value)                                                                          \
    do                                                                                             \
    {                                                                                              \
        pcl_delay_config_t b = cfg;                                                                \
        b.field = value;                                                                           \
        PCL_CHECK_EQ_INT(pcl_delay_required_bytes(&b), 0);                                         \
        PCL_CHECK_EQ_INT(pcl_delay_init(&dl, &b, g_mem, MEM_BYTES), PCL_ERR_OUT_OF_RANGE);         \
    } while (0)
    BAD(sample_rate_hz, PCL_SAMPLE_RATE_MIN_HZ - 1u);
    BAD(sample_rate_hz, PCL_SAMPLE_RATE_MAX_HZ + 1u);
    BAD(max_delay_ms, 0u);
    BAD(max_delay_ms, PCL_DELAY_MAX_MS + 1u);
    BAD(max_delay_ms, 22000u); /* 1 056 000 samples at 48 kHz: beyond the Q20.12 line */
    BAD(time_ms, PCL_MS(100.5));
    BAD(mod_depth_ms, PCL_MS(101.0));
    BAD(glide_ms, PCL_DELAY_MAX_GLIDE_MS + 1u);
    BAD(feedback, PCL_COEF(0.995));
    BAD(feedback, PCL_COEF(-0.995));
    BAD(tone_hz, PCL_HZ(19.0));
    BAD(tone_hz, PCL_HZ(21601.0));
    BAD(mix, PCL_COEF(1.01));
    BAD(mix, PCL_COEF(-0.01));
    BAD(interp, PCL_DELAY_INTERP_COUNT);
    BAD(storage, PCL_DELAY_STORE_COUNT);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    BAD(time_ms, -1.0f);
    BAD(time_ms, NAN);
    BAD(feedback, NAN);
    BAD(mix, NAN);
    BAD(tone_hz, NAN);
    BAD(mod_depth_ms, NAN);
#endif
#undef BAD
    PCL_CHECK(memcmp(&dl, &before, sizeof dl) == 0);

    /* Edges accepted: 21 s fits the line at 48 kHz; the time may equal the maximum. */
    cfg.max_delay_ms = 21000u;
    cfg.time_ms = PCL_MS(21000.0);
    PCL_CHECK(pcl_delay_required_bytes(&cfg) > 0u);
    cfg = make_cfg(100, 100.0);
    cfg.feedback = PCL_COEF(-0.99);
    cfg.tone_hz = PCL_HZ(21600.0);
    cfg.mod_depth_ms = PCL_MS(100.0);
    cfg.glide_ms = PCL_DELAY_MAX_GLIDE_MS;
    init_on(&dl, &cfg, g_mem);
}

static void test_setter_validation(void)
{
    pcl_delay_t        dl, before;
    pcl_delay_config_t cfg = make_cfg(100, 10.0);

    init_on(&dl, &cfg, g_mem);
    before = dl;
    PCL_CHECK_EQ_INT(pcl_delay_set_time(NULL, PCL_MS(1.0)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_delay_set_time(&dl, PCL_MS(100.5)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_delay_set_glide(NULL, 10), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_delay_set_glide(&dl, PCL_DELAY_MAX_GLIDE_MS + 1u), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_delay_set_mod_depth(NULL, PCL_MS(1.0)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_delay_set_mod_depth(&dl, PCL_MS(101.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_delay_set_feedback(NULL, PCL_COEF(0.1)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_delay_set_feedback(&dl, PCL_COEF(1.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_delay_set_tone(NULL, PCL_HZ(1000.0)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_delay_set_tone(&dl, PCL_HZ(10.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_delay_set_mix(NULL, PCL_COEF(0.5)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_delay_set_mix(&dl, PCL_COEF(1.5)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_delay_set_interp(NULL, PCL_DELAY_HERMITE), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_delay_set_interp(&dl, PCL_DELAY_INTERP_COUNT), PCL_ERR_OUT_OF_RANGE);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    PCL_CHECK_EQ_INT(pcl_delay_set_time(&dl, NAN), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_delay_set_mod_depth(&dl, -1.0f), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_delay_set_feedback(&dl, NAN), PCL_ERR_OUT_OF_RANGE);
#endif
    PCL_CHECK(memcmp(&dl, &before, sizeof dl) == 0);

    PCL_CHECK_EQ_INT(pcl_delay_set_time(&dl, PCL_MS(100.0)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_delay_set_glide(&dl, 50), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_delay_set_mod_depth(&dl, PCL_MS(100.0)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_delay_set_feedback(&dl, PCL_COEF(-0.99)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_delay_set_tone(&dl, PCL_HZ(0.0)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_delay_set_tone(&dl, PCL_HZ(2000.0)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_delay_set_mix(&dl, PCL_COEF(0.0)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_delay_set_interp(&dl, PCL_DELAY_HERMITE), PCL_OK);
    pcl_delay_set_saturate(&dl, 1);
}

/* ---- exact and fractional delay ---- */

/* An impulse comes out exactly `want` samples later and nowhere else. */
static void impulse_case(double time_ms, size_t want, pcl_delay_interp_t interp,
    pcl_delay_storage_t storage, unsigned char *mem)
{
    pcl_delay_config_t cfg = make_cfg(50, time_ms);
    pcl_delay_t        dl;
    const size_t       n = 4000;
    size_t             i;
    int                clean = 1;
    const double       amp = 0.75; /* exact in every format */

    cfg.interp = interp;
    cfg.storage = storage;
    init_on(&dl, &cfg, mem);
    memset(g_in, 0, n * sizeof g_in[0]);
    g_in[3] = pcl_test_from_double(amp);
    pcl_delay_process(&dl, g_in, g_out, NULL, n);
    for (i = 0; i < n; i++)
    {
        const double v = pcl_test_to_double(g_out[i]);
        const double e = i == want + 3u ? amp : 0.0;
        if (fabs(v - e) > 1e-20)
            clean = 0;
    }
    if (!clean)
        printf("    impulse %.4f ms (%s, %s) not at %zu\n", time_ms,
            interp == PCL_DELAY_HERMITE ? "hermite" : "linear",
            storage == PCL_DELAY_STORE_Q15 ? "q15" : "native", want);
    PCL_CHECK(clean);
}

static void test_exact_integer_delays(void)
{
    int interp, storage;
    for (interp = 0; interp < 2; interp++)
        for (storage = 0; storage < 2; storage++)
        {
            const pcl_delay_interp_t  ip = (pcl_delay_interp_t)interp;
            const pcl_delay_storage_t st = (pcl_delay_storage_t)storage;
            impulse_case(10.0, 480, ip, st, g_mem);
            impulse_case(50.0, 2400, ip, st, g_mem);            /* the maximum */
            impulse_case(0.0625, 3, ip, st, g_mem);             /* 3 samples */
            impulse_case(0.0, interp ? 2u : 1u, ip, st, g_mem); /* the minimum */
            impulse_case(10.0, 480, ip, st, g_mem2 + 1);        /* an odd address */
        }
}

/* Complex amplitude of x at f over n samples (whole cycles). */
static void phasor(const pcl_sample_t *x, size_t n, double f, double *re, double *im)
{
    size_t i;
    *re = 0.0;
    *im = 0.0;
    for (i = 0; i < n; i++)
    {
        const double v = pcl_test_to_double(x[i]);
        *re += v * cos(2.0 * PI * f * (double)i / FS);
        *im -= v * sin(2.0 * PI * f * (double)i / FS);
    }
}

/*
 * A delay of 100.5 samples (2.09375 ms, exact in every format) against the interpolators' analytic
 * response at t = 1/2: linear (1 + z^-1) / 2, Hermite (-1 + 9 z^-1 + 9 z^-2 - z^-3) / 16 * z (taps
 * x[n-i+1] .. x[n-i-2]).
 */
static void test_half_sample_delay_response(void)
{
    static const double freqs[] = { 1000.0, 6000.0, 12000.0, 18000.0 };
    int                 interp;

    for (interp = 0; interp < 2; interp++)
    {
        size_t f;
        for (f = 0; f < 4; f++)
        {
            pcl_delay_config_t cfg = make_cfg(50, 2.09375);
            pcl_delay_t        dl;
            const size_t       n = FS, settle = 1000;
            const double       w = 2.0 * PI * freqs[f] / FS;
            double             xr, xi, yr, yi, hr, hi, wr, wi, mag, ph, mag_err, ph_err;
            size_t             i;

            cfg.interp = (pcl_delay_interp_t)interp;
            init_on(&dl, &cfg, g_mem);
            for (i = 0; i < n + settle; i++)
                g_in[i] = pcl_test_from_double(0.5 * sin(w * (double)i));
            pcl_delay_process(&dl, g_in, g_out, NULL, n + settle);
            phasor(g_in + settle, n, freqs[f], &xr, &xi);
            phasor(g_out + settle, n, freqs[f], &yr, &yi);
            hr = (yr * xr + yi * xi) / (xr * xr + xi * xi);
            hi = (yi * xr - yr * xi) / (xr * xr + xi * xi);
            if (interp == 0)
            {
                /* e^{-j 100 w} (1 + e^{-jw}) / 2 */
                const double a = 0.5 + 0.5 * cos(w), b = -0.5 * sin(w);
                wr = a * cos(100.0 * w) + b * sin(100.0 * w);
                wi = b * cos(100.0 * w) - a * sin(100.0 * w);
            }
            else
            {
                /* e^{-j 100 w} (-e^{jw} + 9 + 9 e^{-jw} - e^{-2jw}) / 16 */
                const double a = (-cos(w) + 9.0 + 9.0 * cos(w) - cos(2.0 * w)) / 16.0;
                const double b = (-sin(w) - 9.0 * sin(w) + sin(2.0 * w)) / 16.0;
                wr = a * cos(100.0 * w) + b * sin(100.0 * w);
                wi = b * cos(100.0 * w) - a * sin(100.0 * w);
            }
            mag = sqrt(hr * hr + hi * hi);
            mag_err = fabs(20.0 * log10(mag / sqrt(wr * wr + wi * wi)));
            ph = atan2(hi * wr - hr * wi, hr * wr + hi * wi);
            ph_err = fabs(ph);
            printf("    %s, t = 1/2, %5.0f Hz: %+.3f dB (model %+.3f dB), error %.4f dB %.5f rad\n",
                interp ? "hermite" : "linear ", freqs[f], 20.0 * log10(mag),
                20.0 * log10(sqrt(wr * wr + wi * wi)), mag_err, ph_err);
            PCL_CHECK(mag_err < (PCL_SAMPLE_TYPE == PCL_Q15 ? 0.02 : 0.002));
            PCL_CHECK(ph_err < (PCL_SAMPLE_TYPE == PCL_Q15 ? 0.002 : 0.0002));
        }
    }
}

/* ---- feedback ---- */

static void test_feedback_echoes(void)
{
    static const double fbs[] = { 0.5, -0.5, 0.875 };
    size_t              k;

    for (k = 0; k < 3; k++)
    {
        pcl_delay_config_t cfg = make_cfg(50, 2.0); /* 96 samples */
        pcl_delay_t        dl;
        size_t             e;
        const double       amp = 0.5;

        cfg.feedback = coef_of(fbs[k]);
        init_on(&dl, &cfg, g_mem);
        memset(g_in, 0, 2000 * sizeof g_in[0]);
        g_in[0] = pcl_test_from_double(amp);
        pcl_delay_process(&dl, g_in, g_out, NULL, 2000);
        for (e = 1; e <= 8; e++)
        {
            const double want = amp * pow(fbs[k], (double)(e - 1u));
            PCL_CHECK_NEAR(
                pcl_test_to_double(g_out[96u * e]), want, 2.0 * PCL_TEST_SAMPLE_TOL * (double)e);
            PCL_CHECK(fabs(pcl_test_to_double(g_out[96u * e + 1u])) < 1e-9 + PCL_TEST_SAMPLE_TOL);
        }
    }
}

/* A double model of the whole loop at an integer delay: tone, saturate, feedback, mix. */
typedef struct
{
    double line[8192];
    size_t w, d;
    double fb, mix, G, s;
    int    tone, sat;
} model_t;

static double model_step(model_t *m, double x)
{
    const size_t r = (m->w + 8192u - m->d) % 8192u;
    const double y = m->line[r];
    double       v = y;
    if (m->sat && fabs(v) >= 1.0 / 64.0)
        v = tanh(v);
    if (m->tone)
    {
        const double yy = m->G * v + (1.0 - m->G) * m->s;
        m->s = 2.0 * yy - m->s;
        v = yy;
    }
    m->line[m->w] = pcl_test_clip(x + m->fb * v);
    m->w = (m->w + 1u) % 8192u;
    return x + m->mix * (y - x);
}

static void model_case(double fb, double tone_hz, int sat, double mix, double amp, double tol)
{
    pcl_delay_config_t cfg = make_cfg(50, 5.0); /* 240 samples */
    pcl_delay_t        dl;
    static model_t     m;
    const size_t       n = FS;
    size_t             i;
    double             worst = 0.0;

    cfg.feedback = coef_of(fb);
    cfg.saturate = sat;
    cfg.mix = coef_of(mix);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    cfg.tone_hz = (float)tone_hz;
#else
    cfg.tone_hz = (pcl_hz_t)(tone_hz * 65536.0 + 0.5);
#endif
    init_on(&dl, &cfg, g_mem);
    memset(&m, 0, sizeof m);
    m.d = 240;
    m.fb = pcl_test_coef_to_double(cfg.feedback);
    m.mix = pcl_test_coef_to_double(cfg.mix);
    m.tone = tone_hz > 0.0;
    m.sat = sat;
    m.G = tan(PI * tone_hz / FS) / (1.0 + tan(PI * tone_hz / FS));
    fill_noise(g_in, n, 3u, amp);
    pcl_delay_process(&dl, g_in, g_out, NULL, n);
    for (i = 0; i < n; i++)
    {
        const double want = pcl_test_clip(model_step(&m, pcl_test_to_double(g_in[i])));
        const double err = fabs(pcl_test_to_double(g_out[i]) - want);
        if (err > worst)
            worst = err;
    }
    printf(
        "    model fb %.2f tone %5.0f sat %d mix %.2f: worst %.3g\n", fb, tone_hz, sat, mix, worst);
    PCL_CHECK(worst < tol);
}

static void test_matches_double_model(void)
{
    /* About twice the worst measured error. Saturated: pcl_tanh (2.3e-5 from libm) inside a loop with
       gain up to 0.99 accumulates to about 1e-4. */
#if PCL_SAMPLE_TYPE == PCL_Q15
    const double tol = 1e-4, tol_sat = 4e-4;
#elif PCL_SAMPLE_TYPE == PCL_Q31
    const double tol = 2e-8, tol_sat = 2e-4;
#else
    const double tol = 3e-7, tol_sat = 2e-4;
#endif
    model_case(0.7, 0.0, 0, 1.0, 0.4, tol);
    model_case(-0.9, 0.0, 0, 0.5, 0.05, tol);
    model_case(0.95, 3000.0, 0, 0.3, 0.05, tol);
    model_case(0.8, 800.0, 1, 1.0, 0.5, tol_sat);
    model_case(0.99, 0.0, 1, 0.7, 0.3, tol_sat);
}

/* ---- modulation and glide ---- */

/* A constant time_mod equals the corresponding fixed time, bit for bit, once d has arrived. */
static void test_constant_modulation_equals_time(void)
{
    pcl_delay_config_t a_cfg = make_cfg(20, 1.0), b_cfg = make_cfg(20, 2.0);
    pcl_delay_t        a, b;
    size_t             i;
    int                same = 1;
    const size_t       n = 8000;

    a_cfg.mod_depth_ms = PCL_MS(2.0); /* 96 samples */
    a_cfg.feedback = b_cfg.feedback = PCL_COEF(0.5);
    a_cfg.interp = b_cfg.interp = PCL_DELAY_HERMITE;
    init_on(&a, &a_cfg, g_mem);
    init_on(&b, &b_cfg, g_mem2);
    for (i = 0; i < n; i++)
        g_mod[i] = pcl_test_from_double(0.5); /* +48 samples: 1 + 0.5 x 2 = 2 ms in total */
    fill_noise(g_in, n, 4u, 0.5);
    for (i = 0; i < 200; i++)
        g_in[i] = 0; /* nothing enters while a's delay is still moving */
    pcl_delay_process(&a, g_in, g_out, g_mod, n);
    pcl_delay_process(&b, g_in, g_out2, NULL, n);
    for (i = 0; i < n; i++)
        same &= g_out[i] == g_out2[i];
    PCL_CHECK(same);
}

/* Zero-crossing frequency of x over [from, to). */
static double crossing_hz(const pcl_sample_t *x, size_t from, size_t to)
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
    return count > 1u ? (double)(count - 1u) * FS / (last - first) : 0.0;
}

/*
 * Tape-style glide and Doppler: a 10 -> 20 ms glide over 1 s grows the delay by 0.01 sample per
 * sample, so a 1 kHz input comes out at 990 Hz during the glide and at 1 kHz after it.
 */
static void test_glide_bends_pitch(void)
{
    pcl_delay_config_t cfg = make_cfg(50, 10.0);
    pcl_delay_t        dl;
    size_t             i;
    double             during, after;

    cfg.glide_ms = 1000;
    cfg.interp = PCL_DELAY_HERMITE;
    init_on(&dl, &cfg, g_mem);
    for (i = 0; i < MAX_FRAMES; i++)
        g_in[i] = pcl_test_from_double(0.5 * sin(2.0 * PI * 1000.0 * (double)i / FS));
    pcl_delay_process(&dl, g_in, g_out, NULL, 4800);
    PCL_CHECK_EQ_INT(pcl_delay_set_time(&dl, PCL_MS(20.0)), PCL_OK);
    pcl_delay_process(&dl, g_in + 4800, g_out + 4800, NULL, MAX_FRAMES - 4800u);
    during = crossing_hz(g_out, 4800 + 4800, 4800 + 43200);
    after = crossing_hz(g_out, 4800 + 52000, MAX_FRAMES);
    printf("    glide: %.4f Hz during, %.4f Hz after\n", during, after);
    PCL_CHECK_NEAR(during, 990.0, 0.05);
    PCL_CHECK_NEAR(after, 1000.0, 0.01);
    PCL_CHECK(dl.d == 960u * 4096u); /* 20 ms in Q20.12 samples */
}

/* A mix change ramps over the smoothing time instead of jumping. */
static void test_mix_ramps(void)
{
    pcl_delay_config_t cfg = make_cfg(200, 100.0);
    pcl_delay_t        dl;
    const size_t       at = 1000;
    const size_t       ramp = (size_t)pcl_smooth_ramp_samples(FS);
    size_t             i;
    int                monotone = 1;

    cfg.mix = PCL_COEF(0.0);
    init_on(&dl, &cfg, g_mem);
    for (i = 0; i < 3000; i++)
        g_in[i] = pcl_test_from_double(0.5); /* the delayed signal is still 0 */
    pcl_delay_process(&dl, g_in, g_out, NULL, at);
    PCL_CHECK_EQ_INT(pcl_delay_set_mix(&dl, PCL_COEF(1.0)), PCL_OK);
    pcl_delay_process(&dl, g_in + at, g_out + at, NULL, 3000 - at);
    for (i = at + 1u; i < at + ramp + 10u; i++)
        monotone &= g_out[i] <= g_out[i - 1u];
    PCL_CHECK(monotone);
    PCL_CHECK_NEAR(pcl_test_to_double(g_out[at - 1u]), 0.5, PCL_TEST_SAMPLE_TOL);
    PCL_CHECK_NEAR(pcl_test_to_double(g_out[at + ramp / 2u]), 0.25, 0.01);
    PCL_CHECK_NEAR(pcl_test_to_double(g_out[at + ramp + 5u]), 0.0, PCL_TEST_SAMPLE_TOL);
}

/* ---- determinism, blocking, storage ---- */

static void render_blocked(size_t block, pcl_sample_t *out, unsigned char *mem)
{
    pcl_delay_config_t cfg = make_cfg(100, 12.0);
    pcl_delay_t        dl;
    const size_t       total = FS / 2u, change_at = 7000;
    size_t             pos = 0;

    cfg.mod_depth_ms = PCL_MS(3.0);
    cfg.glide_ms = 80;
    cfg.feedback = PCL_COEF(0.6);
    cfg.tone_hz = PCL_HZ(2500.0);
    cfg.saturate = 1;
    cfg.mix = PCL_COEF(0.5);
    cfg.interp = PCL_DELAY_HERMITE;
    init_on(&dl, &cfg, mem);
    while (pos < total)
    {
        size_t n = total - pos < block ? total - pos : block;
        if (pos < change_at && pos + n > change_at)
            n = change_at - pos;
        if (pos == change_at)
        {
            PCL_CHECK_EQ_INT(pcl_delay_set_time(&dl, PCL_MS(40.0)), PCL_OK);
            PCL_CHECK_EQ_INT(pcl_delay_set_feedback(&dl, PCL_COEF(-0.8)), PCL_OK);
            PCL_CHECK_EQ_INT(pcl_delay_set_mix(&dl, PCL_COEF(0.9)), PCL_OK);
            PCL_CHECK_EQ_INT(pcl_delay_set_tone(&dl, PCL_HZ(800.0)), PCL_OK);
        }
        pcl_delay_process(&dl, g_in + pos, out + pos, g_mod + pos, n);
        pos += n;
    }
}

static void test_block_size_independence(void)
{
    static const size_t blocks[] = { 1, 7, 16, 17, 256 };
    size_t              b, i;

    fill_noise(g_in, FS / 2u, 5u, 0.5);
    fill_noise(g_mod, FS / 2u, 6u, 1.0);
    render_blocked(FS, g_out, g_mem);
    for (b = 0; b < sizeof blocks / sizeof blocks[0]; b++)
    {
        int same = 1;
        render_blocked(blocks[b], g_out2, g_mem2);
        for (i = 0; i < FS / 2u; i++)
            same &= g_out[i] == g_out2[i];
        PCL_CHECK(same);
    }
}

static void test_reset_aliasing_and_zero_mod(void)
{
    pcl_delay_config_t cfg = make_cfg(50, 7.3);
    pcl_delay_t        dl;
    const size_t       n = 10000;
    size_t             i;
    int                same = 1;

    cfg.feedback = PCL_COEF(0.7);
    cfg.mod_depth_ms = PCL_MS(2.0);
    cfg.mix = PCL_COEF(0.6);
    init_on(&dl, &cfg, g_mem);
    fill_noise(g_in, n, 7u, 0.5);
    fill_noise(g_mod, n, 8u, 1.0);
    pcl_delay_process(&dl, g_in, g_out, g_mod, n);
    pcl_delay_reset(&dl);
    pcl_delay_process(&dl, g_in, g_out2, g_mod, n);
    for (i = 0; i < n; i++)
        same &= g_out[i] == g_out2[i];
    PCL_CHECK(same);

    pcl_delay_reset(&dl);
    memcpy(g_out2, g_in, n * sizeof g_in[0]);
    pcl_delay_process(&dl, g_out2, g_out2, g_mod, n);
    same = 1;
    for (i = 0; i < n; i++)
        same &= g_out[i] == g_out2[i];
    PCL_CHECK(same);

    memset(g_mod, 0, n * sizeof g_mod[0]);
    pcl_delay_reset(&dl);
    pcl_delay_process(&dl, g_in, g_out, g_mod, n);
    pcl_delay_reset(&dl);
    pcl_delay_process(&dl, g_in, g_out2, NULL, n);
    same = 1;
    for (i = 0; i < n; i++)
        same &= g_out[i] == g_out2[i];
    PCL_CHECK(same);
}

/* Q15 storage differs from native storage only by 16-bit rounding of the stored samples. */
static void test_q15_storage(void)
{
    pcl_delay_config_t cfg = make_cfg(50, 9.77);
    pcl_delay_t        a, b;
    const size_t       n = 20000;
    size_t             i;
    double             worst = 0.0;

    cfg.interp = PCL_DELAY_HERMITE;
    cfg.mod_depth_ms = PCL_MS(1.0);
    init_on(&a, &cfg, g_mem);
    cfg.storage = PCL_DELAY_STORE_Q15;
    init_on(&b, &cfg, g_mem2);
    fill_noise(g_in, n, 9u, 0.7);
    fill_noise(g_mod, n, 10u, 1.0);
    pcl_delay_process(&a, g_in, g_out, g_mod, n);
    pcl_delay_process(&b, g_in, g_out2, g_mod, n);
    for (i = 0; i < n; i++)
    {
        const double e = fabs(pcl_test_to_double(g_out[i]) - pcl_test_to_double(g_out2[i]));
        if (e > worst)
            worst = e;
    }
    printf("    Q15 storage vs native: worst %.3g\n", worst);
    /* Hermite weights sum to at most 1.25 in absolute value, so 1.25 x half a Q15 LSB, plus slack.
     */
    PCL_CHECK(worst <= 1.3 / 65536.0 + PCL_TEST_SAMPLE_TOL);
}

/* ---- extremes ---- */

static void test_extremes_stay_bounded(void)
{
    int sat;
    for (sat = 0; sat <= 1; sat++)
    {
        pcl_delay_config_t cfg = make_cfg(100, 30.0);
        pcl_delay_t        dl;
        size_t             s;
        double             worst = 0.0;

        cfg.feedback = PCL_COEF(0.99);
        cfg.mod_depth_ms = PCL_MS(30.0);
        cfg.saturate = sat;
        cfg.tone_hz = PCL_HZ(15000.0);
        cfg.interp = PCL_DELAY_HERMITE;
        init_on(&dl, &cfg, g_mem);
        for (s = 0; s < 10u; s++)
        {
            fill_noise(g_in, FS, 20u + (uint32_t)s, 1.0);
            fill_noise(g_mod, FS, 40u + (uint32_t)s, 1.0);
            pcl_delay_process(&dl, g_in, g_out, g_mod, FS);
            PCL_CHECK(finite_no_denormals(g_out, FS));
            if (peak(g_out, 0, FS) > worst)
                worst = peak(g_out, 0, FS);
        }
        printf("    feedback 0.99, saturate %d, 10 s of noise and +-30 ms modulation: peak %.3f\n",
            sat, worst);
        /* Float does not clip: the loop gain < 1 bounds it at 1 / (1 - 0.99) of the input. */
        PCL_CHECK(worst < (sat ? 2.5 : 101.0));

        /* Then silence at feedback 0.5: decays, with no denormals (float) on the way. */
        PCL_CHECK_EQ_INT(pcl_delay_set_feedback(&dl, PCL_COEF(0.5)), PCL_OK);
        for (s = 0; s < FS; s++)
            g_in[s] = 0;
        for (s = 0; s < 20u; s++)
        {
            pcl_delay_process(&dl, g_in, g_out, NULL, FS);
            PCL_CHECK(finite_no_denormals(g_out, FS));
        }
        PCL_CHECK(peak(g_out, 0, FS) < 1e-6);
    }
}

int main(void)
{
    PCL_RUN_TEST(test_sizing_and_init_validation);
    PCL_RUN_TEST(test_setter_validation);
    PCL_RUN_TEST(test_exact_integer_delays);
    PCL_RUN_TEST(test_half_sample_delay_response);
    PCL_RUN_TEST(test_feedback_echoes);
    PCL_RUN_TEST(test_matches_double_model);
    PCL_RUN_TEST(test_constant_modulation_equals_time);
    PCL_RUN_TEST(test_glide_bends_pitch);
    PCL_RUN_TEST(test_mix_ramps);
    PCL_RUN_TEST(test_block_size_independence);
    PCL_RUN_TEST(test_reset_aliasing_and_zero_mod);
    PCL_RUN_TEST(test_q15_storage);
    PCL_RUN_TEST(test_extremes_stay_bounded);
    return pcl_test_summary("delay");
}
