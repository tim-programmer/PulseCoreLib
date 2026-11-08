/*
 * pcl_svf tests: init and setters, the exact analogue response (magnitude, phase, tuning), a
 * time-domain double-precision model that follows the control-rate coefficient ramps, block-size
 * independence, modulation timing, smoothing in octaves, the derived modes, aliasing, and a
 * stability torture test (self-oscillation under fast cutoff modulation).
 */
#include <stdlib.h>
#include <string.h>

#include "pcl_test.h"

#if PCL_SAMPLE_TYPE == PCL_FLOAT
#define HZ_OF(x) ((float)(x))
#define HZ_D(x) ((double)(x))
#define COEF_OF(x) ((float)(x))
#define OCT_OF(x) ((float)(x))
#define OCT_D(x) ((double)(x))
#define WIDE_D(x) ((double)(x))
#define IS_FLOAT 1
#else
#define HZ_OF(x) ((uint32_t)floor((x) * 65536.0 + 0.5))
#define HZ_D(x) ((double)(x) / 65536.0)
#define COEF_OF(x) ((int32_t)floor((x) * 1073741824.0 + 0.5))
#define OCT_OF(x) ((int32_t)floor((x) * 268435456.0 + 0.5))
#define OCT_D(x) ((double)(x) / 268435456.0)
#define WIDE_D(x) ((double)(x) / 268435456.0)
#define IS_FLOAT 0
#endif

#define SMP_OF(x) pcl_test_from_double(x)
#define SMP_D(x) pcl_test_to_double(x)

static const double pi = 3.14159265358979323846;

/* Tolerances, from the measured error of each build (magnitude / phase / tuning). */
#if PCL_SAMPLE_TYPE == PCL_FLOAT
#define MAG_TOL_DB 0.003
#define PHASE_TOL 0.001
#define TUNE_TOL_CENTS 0.05
#define MODEL_TOL 3.0e-4
#define OUT_FLOOR 2.0e-6 /* below this expected amplitude the check is skipped */
#elif PCL_SAMPLE_TYPE == PCL_Q15
#define MAG_TOL_DB 0.012
#define PHASE_TOL 0.008
#define TUNE_TOL_CENTS 0.4
#define MODEL_TOL 2.0e-3
#define OUT_FLOOR 3.0e-3
#else
#define MAG_TOL_DB 0.003
#define PHASE_TOL 0.001
#define TUNE_TOL_CENTS 0.05
#define MODEL_TOL 3.0e-4
#define OUT_FLOOR 2.0e-6
#endif

/* ---- Helpers ---- */

static uint32_t g_rng = 0x2545F491u;

static double frand(void) /* uniform in [-1, 1) */
{
    g_rng ^= g_rng << 13;
    g_rng ^= g_rng >> 17;
    g_rng ^= g_rng << 5;
    return (double)(g_rng >> 8) / 8388608.0 - 1.0;
}

static pcl_svf_config_t make_cfg(uint32_t fs, double fc, double r, double depth_oct, int sat)
{
    pcl_svf_config_t c;
    c.sample_rate_hz = fs;
    c.cutoff_hz = HZ_OF(fc);
    c.resonance = COEF_OF(r);
    c.mod_depth_oct = OCT_OF(depth_oct);
    c.saturate = sat;
    return c;
}

static double damping_of(double r) { return sqrt(2.0) * (1.0 - r) * (1.0 - r); }

/* Runs `total` samples in blocks of `blk`; any output may be NULL; `mod` may be NULL. */
static void run_blocks(pcl_svf_t *f, const pcl_sample_t *in, pcl_sample_t *lp, pcl_sample_t *bp,
    pcl_sample_t *hp, const pcl_sample_t *mod, size_t total, size_t blk)
{
    size_t n = 0;
    while (n < total)
    {
        const size_t c = total - n < blk ? total - n : blk;
        pcl_svf_process(f, in + n, lp ? lp + n : NULL, bp ? bp + n : NULL, hp ? hp + n : NULL,
            mod ? mod + n : NULL, c);
        n += c;
    }
}

static pcl_sample_t *alloc_samples(size_t n)
{
    pcl_sample_t *p = (pcl_sample_t *)calloc(n, sizeof(pcl_sample_t));
    if (p == NULL)
        exit(2);
    return p;
}

static void fill_noise(pcl_sample_t *x, size_t n, double amp)
{
    size_t i;
    for (i = 0; i < n; i++)
        x[i] = SMP_OF(amp * frand());
}

/* ---- Init and setters ---- */

static void test_init_and_setters(void)
{
    pcl_svf_t        f;
    pcl_svf_t        keep;
    pcl_svf_config_t c = make_cfg(48000u, 1000.0, 0.5, 2.0, 0);

    PCL_CHECK_EQ_INT(pcl_svf_init(&f, &c), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_svf_init(NULL, &c), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_svf_init(&f, NULL), PCL_ERR_NULL_POINTER);

    /* An invalid config leaves the module untouched. */
    keep = f;
    c = make_cfg(7999u, 1000.0, 0.5, 2.0, 0);
    PCL_CHECK_EQ_INT(pcl_svf_init(&f, &c), PCL_ERR_OUT_OF_RANGE);
    c = make_cfg(768001u, 1000.0, 0.5, 2.0, 0);
    PCL_CHECK_EQ_INT(pcl_svf_init(&f, &c), PCL_ERR_OUT_OF_RANGE);
    c = make_cfg(48000u, 4.0, 0.5, 2.0, 0); /* below 5 Hz */
    PCL_CHECK_EQ_INT(pcl_svf_init(&f, &c), PCL_ERR_OUT_OF_RANGE);
    c = make_cfg(48000u, 21700.0, 0.5, 2.0, 0); /* above 0.45 fs = 21600 */
    PCL_CHECK_EQ_INT(pcl_svf_init(&f, &c), PCL_ERR_OUT_OF_RANGE);
    c = make_cfg(48000u, 1000.0, 1.01, 2.0, 0);
    PCL_CHECK_EQ_INT(pcl_svf_init(&f, &c), PCL_ERR_OUT_OF_RANGE);
    c = make_cfg(48000u, 1000.0, -0.01, 2.0, 0);
    PCL_CHECK_EQ_INT(pcl_svf_init(&f, &c), PCL_ERR_OUT_OF_RANGE);
    c = make_cfg(48000u, 1000.0, 0.5, -1.0, 0);
    PCL_CHECK_EQ_INT(pcl_svf_init(&f, &c), PCL_ERR_OUT_OF_RANGE);
#if IS_FLOAT
    /* Q4.28 cannot even express 8 octaves, so only the float build can be given one */
    c = make_cfg(48000u, 1000.0, 0.5, 8.5, 0);
    PCL_CHECK_EQ_INT(pcl_svf_init(&f, &c), PCL_ERR_OUT_OF_RANGE);
#endif
    PCL_CHECK(memcmp(&f, &keep, sizeof f) == 0);

    /* Limits are accepted. */
    c = make_cfg(48000u, 5.0, 0.0, 0.0, 0);
    PCL_CHECK_EQ_INT(pcl_svf_init(&f, &c), PCL_OK);
    c = make_cfg(48000u, 21599.0, 1.0, 7.99, 1);
    PCL_CHECK_EQ_INT(pcl_svf_init(&f, &c), PCL_OK);
    c = make_cfg(8000u, 3599.0, 1.0, 0.0, 0);
    PCL_CHECK_EQ_INT(pcl_svf_init(&f, &c), PCL_OK);

    /* Setters: errors keep the previous state. */
    c = make_cfg(48000u, 1000.0, 0.5, 2.0, 0);
    PCL_CHECK_EQ_INT(pcl_svf_init(&f, &c), PCL_OK);
    keep = f;
    PCL_CHECK_EQ_INT(pcl_svf_set_cutoff(NULL, HZ_OF(100.0)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_svf_set_cutoff(&f, HZ_OF(4.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_svf_set_cutoff(&f, HZ_OF(22000.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_svf_set_resonance(NULL, COEF_OF(0.5)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_svf_set_resonance(&f, COEF_OF(1.5)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_svf_set_resonance(&f, COEF_OF(-0.5)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_svf_set_mod_depth(NULL, OCT_OF(1.0)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_svf_set_mod_depth(&f, OCT_OF(-1.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK(memcmp(&f, &keep, sizeof f) == 0);
    PCL_CHECK_EQ_INT(pcl_svf_set_cutoff(&f, HZ_OF(5.0)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_svf_set_cutoff(&f, HZ_OF(21599.0)), PCL_OK); /* 0.45 fs, truncated */
#if !IS_FLOAT
    PCL_CHECK_EQ_INT(pcl_svf_set_cutoff(&f, HZ_OF(21600.0)), PCL_ERR_OUT_OF_RANGE);
#endif
    PCL_CHECK_EQ_INT(pcl_svf_set_cutoff(&f, HZ_OF(21601.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_svf_set_resonance(&f, COEF_OF(0.0)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_svf_set_resonance(&f, COEF_OF(1.0)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_svf_set_mod_depth(&f, OCT_OF(3.0)), PCL_OK);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    PCL_CHECK_EQ_INT(pcl_svf_set_cutoff(&f, NAN), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_svf_set_resonance(&f, NAN), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_svf_set_mod_depth(&f, NAN), PCL_ERR_OUT_OF_RANGE);
    c = make_cfg(48000u, 1000.0, 0.5, 2.0, 0);
    c.cutoff_hz = NAN;
    PCL_CHECK_EQ_INT(pcl_svf_init(&f, &c), PCL_ERR_OUT_OF_RANGE);
#endif
}

/* ---- The analogue response ---- */

typedef struct
{
    double amp[3]; /* lp, bp, hp amplitude */
    double ph[3];  /* phase in radians */
} resp_t;

/* Steady-state response to a sine at `ft` (a multiple of fs / window), by lock-in over a window. */
static resp_t measure(pcl_svf_t *f, double ft, double amp_in, double fs, double k, double fc_hz)
{
    enum
    {
        WIN = 32768,
        BLK = 256
    };
    static pcl_sample_t xb[BLK], lb[BLK], bb[BLK], hb[BLK];
    resp_t              r;
    double              si[3] = { 0, 0, 0 };
    double              sq[3] = { 0, 0, 0 };
    long                settle = (long)(14.0 * fs / (pi * fc_hz * (k > 0.01 ? k : 0.01)));
    long                n = 0;
    long                total;
    int                 o;

    if (settle > 700000)
        settle = 700000;
    if (settle < 4000)
        settle = 4000;
    total = settle + WIN;
    pcl_svf_reset(f);
    while (n < total)
    {
        const int c = (int)(total - n < BLK ? total - n : BLK);
        int       i;
        for (i = 0; i < c; i++)
            xb[i] = SMP_OF(amp_in * sin(2.0 * pi * ft * (double)(n + i) / fs));
        pcl_svf_process(f, xb, lb, bb, hb, NULL, (size_t)c);
        for (i = 0; i < c; i++)
        {
            const long nn = n + i;
            if (nn >= settle)
            {
                const double s = sin(2.0 * pi * ft * (double)nn / fs);
                const double co = cos(2.0 * pi * ft * (double)nn / fs);
                const double y[3] = { SMP_D(lb[i]), SMP_D(bb[i]), SMP_D(hb[i]) };
                for (o = 0; o < 3; o++)
                {
                    si[o] += y[o] * s;
                    sq[o] += y[o] * co;
                }
            }
        }
        n += c;
    }
    for (o = 0; o < 3; o++)
    {
        r.amp[o] = 2.0 / WIN * sqrt(si[o] * si[o] + sq[o] * sq[o]);
        r.ph[o] = atan2(sq[o], si[o]);
    }
    return r;
}

static double wrap_pi(double a)
{
    while (a > pi)
        a -= 2.0 * pi;
    while (a < -pi)
        a += 2.0 * pi;
    return a;
}

/* Exact responses of the TPT SVF: u = tan(pi f / fs) / g. */
static void analytic(double ft, double fs, double g, double k, double *mag, double *ph)
{
    const double u = tan(pi * ft / fs) / g;
    /* den = 1 - u^2 + j k u */
    const double dr = 1.0 - u * u;
    const double di = k * u;
    const double dn = dr * dr + di * di;
    /* lp = 1/den, bp = j u / den, hp = -u^2 / den */
    const double lr = dr / dn;
    const double li = -di / dn;
    mag[0] = sqrt(lr * lr + li * li);
    ph[0] = atan2(li, lr);
    mag[1] = u * mag[0];
    ph[1] = wrap_pi(ph[0] + pi / 2.0);
    mag[2] = u * u * mag[0];
    ph[2] = wrap_pi(ph[0] + pi);
}

static void test_response_matches_analogue_prototype(void)
{
    static const uint32_t rates[] = { 48000u, 44100u };
    static const double   fcs[] = { 20.0, 100.0, 1000.0, 8000.0, 19000.0 };
    static const double   res[] = { 0.0, 0.5, 0.8 };
    static const double   ratios[] = { 0.1, 0.5, 0.9, 1.0, 1.1, 2.0, 4.0 };
    size_t                ri, a, b, q;

    for (ri = 0; ri < sizeof rates / sizeof rates[0]; ri++)
    {
        const double fs = (double)rates[ri];
        for (a = 0; a < sizeof fcs / sizeof fcs[0]; a++)
            for (b = 0; b < sizeof res / sizeof res[0]; b++)
            {
                const pcl_svf_config_t cfg = make_cfg(rates[ri], fcs[a], res[b], 0.0, 0);
                pcl_svf_t              f;
                const double           fcq = HZ_D(cfg.cutoff_hz);
                const double           rq = pcl_test_coef_to_double(cfg.resonance);
                const double           k = damping_of(rq);
                const double           g = tan(pi * fcq / fs);

                if (ri > 0 && (a != 2 && a != 3))
                    continue; /* the second rate only at two cutoffs */
                PCL_CHECK_EQ_INT(pcl_svf_init(&f, &cfg), PCL_OK);
                for (q = 0; q < sizeof ratios / sizeof ratios[0]; q++)
                {
                    int    m = (int)floor(fcq * ratios[q] / fs * 32768.0 + 0.5);
                    double ft;
                    double mag[3], ph[3], mx, amp;
                    resp_t got;
                    int    o;

                    if (m < 1)
                        m = 1;
                    ft = fs * (double)m / 32768.0; /* a whole number of cycles in the window */
                    if (ft > 0.45 * fs)
                        continue;
                    analytic(ft, fs, g, k, mag, ph);
                    mx = fmax(1.0, fmax(mag[0], fmax(mag[1], mag[2])));
                    amp = 0.5 / mx;
                    got = measure(&f, ft, amp, fs, k, fcq);
                    for (o = 0; o < 3; o++)
                    {
                        const double exp_amp = amp * mag[o];
                        if (exp_amp < OUT_FLOOR)
                            continue;
                        PCL_CHECK_NEAR(20.0 * log10(got.amp[o] / exp_amp), 0.0, MAG_TOL_DB);
                        if (exp_amp > 0.01)
                            PCL_CHECK_NEAR(wrap_pi(got.ph[o] - ph[o]), 0.0, PHASE_TOL);
                    }
                }
            }
    }
}

/* The band-pass is in phase at the cutoff and its gain is Q: the cutoff is where it says it is. */
static void test_cutoff_tuning(void)
{
    static const double fcs[] = { 20.0, 60.0, 440.0, 3000.0, 12000.0, 19500.0 };
    size_t              a;

    for (a = 0; a < sizeof fcs / sizeof fcs[0]; a++)
    {
        const double           fs = 48000.0;
        const pcl_svf_config_t cfg = make_cfg(48000u, fcs[a], 0.8, 0.0, 0);
        pcl_svf_t              f;
        const double           fcq = HZ_D(cfg.cutoff_hz);
        const double           k = damping_of(pcl_test_coef_to_double(cfg.resonance));
        const double           ft = fs * floor(fcq / fs * 32768.0 + 0.5) / 32768.0;
        double                 mag[3], ph[3];
        resp_t                 got;
        double                 g = tan(pi * fcq / fs);
        double                 cents;

        PCL_CHECK_EQ_INT(pcl_svf_init(&f, &cfg), PCL_OK);
        analytic(ft, fs, g, k, mag, ph);
        got = measure(&f, ft, 0.04, fs, k, fcq);
        /* A band-pass phase error d_phi at resonance is a cutoff error of d_phi * k / 2. */
        cents = fabs(wrap_pi(got.ph[1] - ph[1])) * k / 2.0 * 1200.0 / log(2.0);
        PCL_CHECK(cents < TUNE_TOL_CENTS);
        /* Gain Q at the cutoff, to the step of the measurement grid. */
        PCL_CHECK_NEAR(got.amp[1] / (0.04 * mag[1]), 1.0, 0.01);
    }
}

/* ---- Derived modes ---- */

static void test_modes_are_sums_of_the_three_outputs(void)
{
    enum
    {
        N = 6000
    };
    static const pcl_svf_mode_t modes[] = { PCL_SVF_LP, PCL_SVF_BP, PCL_SVF_BP_NORM, PCL_SVF_HP,
        PCL_SVF_NOTCH, PCL_SVF_PEAK, PCL_SVF_ALLPASS };
    pcl_sample_t               *x = alloc_samples(N);
    pcl_sample_t               *lp = alloc_samples(N);
    pcl_sample_t               *bp = alloc_samples(N);
    pcl_sample_t               *hp = alloc_samples(N);
    pcl_sample_t               *m = alloc_samples(N);
    const pcl_svf_config_t      cfg = make_cfg(48000u, 1500.0, 0.6, 0.0, 0);
    const double                k = damping_of(pcl_test_coef_to_double(cfg.resonance));
    size_t                      mi;
    int                         i;

    fill_noise(x, N, 0.2);
    for (mi = 0; mi < sizeof modes / sizeof modes[0]; mi++)
    {
        pcl_svf_t f3, f1;
        PCL_CHECK_EQ_INT(pcl_svf_init(&f3, &cfg), PCL_OK);
        PCL_CHECK_EQ_INT(pcl_svf_init(&f1, &cfg), PCL_OK);
        pcl_svf_process(&f3, x, lp, bp, hp, NULL, N);
        pcl_svf_process_mode(&f1, x, m, modes[mi], NULL, N);
        for (i = 0; i < N; i++)
        {
            const double l = SMP_D(lp[i]), b = SMP_D(bp[i]), h = SMP_D(hp[i]);
            double       want;
            switch (modes[mi])
            {
            case PCL_SVF_LP:
                want = l;
                break;
            case PCL_SVF_BP:
                want = b;
                break;
            case PCL_SVF_BP_NORM:
                want = k * b;
                break;
            case PCL_SVF_HP:
                want = h;
                break;
            case PCL_SVF_NOTCH:
                want = l + h;
                break;
            case PCL_SVF_PEAK:
                want = l - h;
                break;
            default:
                want = l + h - k * b;
                break;
            }
            PCL_CHECK_NEAR(SMP_D(m[i]), pcl_test_clip(want), 6.0 * PCL_TEST_SAMPLE_TOL + 1e-6);
        }
    }
    /* LP, BP and HP are the same samples as the three-output call, bit for bit. */
    {
        pcl_svf_t f3, f1;
        PCL_CHECK_EQ_INT(pcl_svf_init(&f3, &cfg), PCL_OK);
        PCL_CHECK_EQ_INT(pcl_svf_init(&f1, &cfg), PCL_OK);
        pcl_svf_process(&f3, x, lp, bp, hp, NULL, N);
        pcl_svf_process_mode(&f1, x, m, PCL_SVF_BP, NULL, N);
        PCL_CHECK(memcmp(m, bp, N * sizeof(pcl_sample_t)) == 0);
    }
    free(x);
    free(lp);
    free(bp);
    free(hp);
    free(m);
}

/* The all-pass has unit gain at every frequency. */
static void test_allpass_is_flat(void)
{
    enum
    {
        WIN = 32768
    };
    static const double    ratios[] = { 0.1, 0.7, 1.0, 1.6, 4.0 };
    const double           fs = 48000.0;
    const pcl_svf_config_t cfg = make_cfg(48000u, 2000.0, 0.7, 0.0, 0);
    pcl_sample_t          *x = alloc_samples(WIN + 20000);
    pcl_sample_t          *y = alloc_samples(WIN + 20000);
    size_t                 q;

    for (q = 0; q < sizeof ratios / sizeof ratios[0]; q++)
    {
        pcl_svf_t    f;
        const double ft = fs * floor(2000.0 * ratios[q] / fs * WIN + 0.5) / WIN;
        double       si = 0, sq = 0;
        int          n;

        PCL_CHECK_EQ_INT(pcl_svf_init(&f, &cfg), PCL_OK);
        for (n = 0; n < WIN + 20000; n++)
            x[n] = SMP_OF(0.4 * sin(2.0 * pi * ft * n / fs));
        pcl_svf_process_mode(&f, x, y, PCL_SVF_ALLPASS, NULL, WIN + 20000);
        for (n = 20000; n < WIN + 20000; n++)
        {
            si += SMP_D(y[n]) * sin(2.0 * pi * ft * n / fs);
            sq += SMP_D(y[n]) * cos(2.0 * pi * ft * n / fs);
        }
        PCL_CHECK_NEAR(2.0 / WIN * sqrt(si * si + sq * sq) / 0.4, 1.0, 0.003);
    }
    free(x);
    free(y);
}

/* ---- A time-domain model that follows the control-rate ramps ---- */

typedef struct
{
    double fs, depth;
    double u0, u1, k0, k1; /* smoother ends, u = octaves above 1 kHz / 8 */
    long   adv_u, adv_k, ramp;
    double g, c, a1, g_end, c_end, a1_end, dg, dc;
    double s1, s2;
    int    pos, dirty, sat, ramping, exact;
} model_t;

static double model_u(const model_t *m)
{
    const double t = m->adv_u >= m->ramp ? 1.0 : (double)m->adv_u / (double)m->ramp;
    return m->u0 + (m->u1 - m->u0) * t;
}

static double model_k(const model_t *m)
{
    const double t = m->adv_k >= m->ramp ? 1.0 : (double)m->adv_k / (double)m->ramp;
    return m->k0 + (m->k1 - m->k0) * t;
}

static void model_target(const model_t *m, double mod_oct, double *g, double *c, double *a1)
{
    double f = 1000.0 * pow(2.0, 8.0 * model_u(m) + mod_oct);
    if (f < 1000.0 / 256.0)
        f = 1000.0 / 256.0;
    if (f > 0.45 * m->fs)
        f = 0.45 * m->fs;
    *g = tan(pi * f / m->fs);
    *c = *g + model_k(m);
    *a1 = 1.0 / (1.0 + *g * *c);
}

static void model_init(model_t *m, const pcl_svf_config_t *cfg)
{
    m->fs = (double)cfg->sample_rate_hz;
    m->depth = OCT_D(cfg->mod_depth_oct);
    m->u0 = m->u1 = log2(HZ_D(cfg->cutoff_hz) / 1000.0) / 8.0;
    m->k0 = m->k1 = damping_of(pcl_test_coef_to_double(cfg->resonance));
    m->ramp = (long)pcl_smooth_ramp_samples(cfg->sample_rate_hz);
    m->adv_u = m->adv_k = m->ramp;
    model_target(m, 0.0, &m->g, &m->c, &m->a1);
    m->g_end = m->g;
    m->c_end = m->c;
    m->a1_end = m->a1;
    m->dg = m->dc = 0.0;
    m->ramping = m->exact = 0;
    m->s1 = m->s2 = 0.0;
    m->pos = 0;
    m->dirty = 0;
    m->sat = cfg->saturate;
}

static void model_set_cutoff(model_t *m, double hz)
{
    m->u0 = model_u(m);
    m->u1 = log2(hz / 1000.0) / 8.0;
    m->adv_u = 0;
}

static void model_set_resonance(model_t *m, double r)
{
    m->k0 = model_k(m);
    m->k1 = damping_of(r);
    m->adv_k = 0;
}

static double mclamp(double v) { return v > 8.0 ? 8.0 : (v < -8.0 ? -8.0 : v); }

static void model_tick(model_t *m, int have_mod, double mod)
{
    const int moving = m->adv_u < m->ramp || m->adv_k < m->ramp;

    if (!have_mod && !moving && !m->dirty)
    {
        m->dg = m->dc = 0.0;
        m->ramping = 0;
        return;
    }
    if (moving)
    {
        m->adv_u += PCL_CONTROL_DIV;
        m->adv_k += PCL_CONTROL_DIV;
    }
    model_target(m, have_mod ? mod * m->depth : 0.0, &m->g_end, &m->c_end, &m->a1_end);
    m->dg = (m->g_end - m->g) / PCL_CONTROL_DIV;
    m->dc = (m->c_end - m->c) / PCL_CONTROL_DIV;
    m->ramping = m->dg != 0.0 || m->dc != 0.0;
    m->dirty = have_mod;
}

static void model_process(model_t *m, const pcl_sample_t *in, const pcl_sample_t *mod, double *lp,
    double *bp, double *hp, size_t n)
{
    size_t i;
    for (i = 0; i < n; i++)
    {
        double x, t, h, gh, b, gb, l;
        if (m->pos == 0)
            model_tick(m, mod != NULL, mod ? SMP_D(mod[i]) : 0.0);
        m->g += m->dg;
        m->c += m->dc;
        if (m->exact)
            m->a1 = 1.0 / (1.0 + m->g * m->c);
        else if (m->ramping)
        {
            /* the module's Newton step, with the same clamp */
            const double d = 1.0 + m->g * m->c;
            const double a = m->a1 * (2.0 - d * m->a1);
            m->a1 = a < 0.0 ? 0.0 : (a > 1.0 ? 1.0 : a);
        }
        x = SMP_D(in[i]);
        t = x - m->c * m->s1 - m->s2;
        h = m->a1 * t;
        gh = m->g * h;
        b = mclamp(gh + m->s1);
        gb = m->g * b;
        l = mclamp(gb + m->s2);
        m->s1 = mclamp(b + gh);
        m->s2 = mclamp(l + gb);
        if (m->sat)
            m->s1 = tanh(m->s1);
        lp[i] = l;
        bp[i] = b;
        hp[i] = h;
        if (++m->pos == PCL_CONTROL_DIV)
        {
            m->pos = 0;
            m->g = m->g_end;
            m->c = m->c_end;
            m->a1 = m->a1_end;
        }
    }
}

/* Relative rms difference between the module's output and a model's, from sample 2000 on. */
static double rel_diff(const pcl_sample_t *y, const double *ref, size_t n)
{
    double e = 0.0, p = 0.0;
    size_t i;
    for (i = 2000; i < n; i++)
    {
        const double d = SMP_D(y[i]) - pcl_test_clip(ref[i]);
        e += d * d;
        p += ref[i] * ref[i];
    }
    return sqrt(e / p);
}

/*
 * Random input, random cutoff modulation, setter calls between blocks. The module is compared with
 * a model that mirrors its arithmetic (Newton step for a1) tightly, and with a model that computes
 * a1 exactly per sample loosely: that bounds what the Newton step gives up.
 */
static void model_case(double fc, double r, double depth, int use_mod, int sat, double exact_tol)
{
    enum
    {
        BLK = 256,
        N = 188 * 256 /* a whole number of blocks, about 1 s */
    };
    const pcl_svf_config_t cfg = make_cfg(48000u, fc, r, depth, sat);
    pcl_svf_t              f;
    model_t                m, mx;
    pcl_sample_t          *x = alloc_samples(N);
    pcl_sample_t          *mod = alloc_samples(N);
    pcl_sample_t          *lp = alloc_samples(N);
    pcl_sample_t          *bp = alloc_samples(N);
    pcl_sample_t          *hp = alloc_samples(N);
    double                *ml = (double *)calloc(N, sizeof(double));
    double                *mb = (double *)calloc(N, sizeof(double));
    double                *mh = (double *)calloc(N, sizeof(double));
    double                *xl = (double *)calloc(N, sizeof(double));
    double                *xb = (double *)calloc(N, sizeof(double));
    double                *xh = (double *)calloc(N, sizeof(double));
    const double           tol = MODEL_TOL * (sat ? 40.0 : 1.0);
    size_t                 n;
    int                    i;

    if (ml == NULL || mb == NULL || mh == NULL || xl == NULL || xb == NULL || xh == NULL)
        exit(2);
    PCL_CHECK_EQ_INT(pcl_svf_init(&f, &cfg), PCL_OK);
    model_init(&m, &cfg);
    model_init(&mx, &cfg);
    mx.exact = 1;
    fill_noise(x, N, 0.4);
    for (i = 0; i < N; i++)
    {
        /* a slow wave plus noise, so the modulation is both smooth and rough */
        mod[i] = SMP_OF(0.6 * sin(2.0 * pi * 3.0 * i / 48000.0) + 0.3 * frand());
    }
    for (n = 0; n < N; n += BLK)
    {
        if (n == 12 * BLK)
        {
            const double hz = fc * 3.0 < 18000.0 ? fc * 3.0 : 18000.0;
            PCL_CHECK_EQ_INT(pcl_svf_set_cutoff(&f, HZ_OF(hz)), PCL_OK);
            model_set_cutoff(&m, HZ_D(HZ_OF(hz)));
            model_set_cutoff(&mx, HZ_D(HZ_OF(hz)));
        }
        if (n == 40 * BLK)
        {
            PCL_CHECK_EQ_INT(pcl_svf_set_resonance(&f, COEF_OF(r * 0.5)), PCL_OK);
            model_set_resonance(&m, pcl_test_coef_to_double(COEF_OF(r * 0.5)));
            model_set_resonance(&mx, pcl_test_coef_to_double(COEF_OF(r * 0.5)));
        }
        pcl_svf_process(&f, x + n, lp + n, bp + n, hp + n, use_mod ? mod + n : NULL, BLK);
        model_process(&m, x + n, use_mod ? mod + n : NULL, ml + n, mb + n, mh + n, BLK);
        model_process(&mx, x + n, use_mod ? mod + n : NULL, xl + n, xb + n, xh + n, BLK);
    }
    PCL_CHECK(rel_diff(lp, ml, N) < tol);
    PCL_CHECK(rel_diff(bp, mb, N) < tol);
    PCL_CHECK(rel_diff(hp, mh, N) < tol);
    {
        const double et
            = exact_tol > MODEL_TOL ? exact_tol : MODEL_TOL; /* never tighter than the build */
        PCL_CHECK(rel_diff(lp, xl, N) < et);
        PCL_CHECK(rel_diff(bp, xb, N) < et);
        PCL_CHECK(rel_diff(hp, xh, N) < et);
    }
    free(x);
    free(mod);
    free(lp);
    free(bp);
    free(hp);
    free(ml);
    free(mb);
    free(mh);
    free(xl);
    free(xb);
    free(xh);
}

static void test_matches_time_domain_model(void)
{
    /* Cases whose output is well conditioned (moderate damping and modulation). A high-Q filter
       pumped by violent random modulation is chaotic: tiny differences grow, so it cannot be
       compared sample by sample; test_heavy_modulation_needs_no_clamp covers that regime by its
       bounds. The last argument is how far the Newton step may be from exact per-sample
       coefficients. */
    model_case(500.0, 0.0, 0.0, 0, 0, 1.0e-4);
    model_case(2000.0, 0.5, 0.0, 0, 0, 1.0e-4);
    model_case(300.0, 0.7, 3.0, 1, 0, 2.0e-2);
    model_case(1500.0, 0.4, 2.0, 1, 0, 2.0e-2);
    model_case(120.0, 0.6, 4.0, 1, 0, 2.0e-2);
    model_case(1000.0, 0.4, 1.0, 1, 0, 1.0e-2);
}

static void test_saturate_matches_model(void)
{
    /* the model has an exact tanh; the build's table (and Q15's 15-bit state) is looser */
    model_case(800.0, 0.3, 1.0, 1, 1, 5.0e-2);
    model_case(2500.0, 0.2, 0.0, 0, 1, 1.0e-3);
}

/* ---- Determinism, block sizes, aliasing, NULL ---- */

static void test_block_size_independence(void)
{
    enum
    {
        N = 5000
    };
    static const size_t blocks[] = { 1, 7, 16, 17, 100, 256, 1000 };
    pcl_sample_t       *x = alloc_samples(N);
    pcl_sample_t       *mod = alloc_samples(N);
    pcl_sample_t       *ref[3];
    pcl_sample_t       *got[3];
    size_t              bi;
    int                 with_mod;
    int                 o;

    for (o = 0; o < 3; o++)
    {
        ref[o] = alloc_samples(N);
        got[o] = alloc_samples(N);
    }
    fill_noise(x, N, 0.4);
    fill_noise(mod, N, 1.0);
    for (with_mod = 0; with_mod < 2; with_mod++)
    {
        const pcl_svf_config_t cfg = make_cfg(48000u, 700.0, 0.8, 3.0, 0);
        pcl_svf_t              fr;
        PCL_CHECK_EQ_INT(pcl_svf_init(&fr, &cfg), PCL_OK);
        PCL_CHECK_EQ_INT(pcl_svf_set_cutoff(&fr, HZ_OF(3000.0)), PCL_OK); /* a ramp in progress */
        run_blocks(&fr, x, ref[0], ref[1], ref[2], with_mod ? mod : NULL, N, N);
        for (bi = 0; bi < sizeof blocks / sizeof blocks[0]; bi++)
        {
            pcl_svf_t f;
            PCL_CHECK_EQ_INT(pcl_svf_init(&f, &cfg), PCL_OK);
            PCL_CHECK_EQ_INT(pcl_svf_set_cutoff(&f, HZ_OF(3000.0)), PCL_OK);
            run_blocks(&f, x, got[0], got[1], got[2], with_mod ? mod : NULL, N, blocks[bi]);
            for (o = 0; o < 3; o++)
                PCL_CHECK(memcmp(ref[o], got[o], N * sizeof(pcl_sample_t)) == 0);
        }
    }
    free(x);
    free(mod);
    for (o = 0; o < 3; o++)
    {
        free(ref[o]);
        free(got[o]);
    }
}

static void test_reset_is_deterministic(void)
{
    enum
    {
        N = 4000
    };
    const pcl_svf_config_t cfg = make_cfg(48000u, 1200.0, 0.9, 2.0, 1);
    pcl_svf_t              f;
    pcl_sample_t          *x = alloc_samples(N);
    pcl_sample_t          *mod = alloc_samples(N);
    pcl_sample_t          *a = alloc_samples(N);
    pcl_sample_t          *b = alloc_samples(N);

    fill_noise(x, N, 0.5);
    fill_noise(mod, N, 1.0);
    PCL_CHECK_EQ_INT(pcl_svf_init(&f, &cfg), PCL_OK);
    run_blocks(&f, x, a, NULL, NULL, mod, N, 256);
    /* disturb everything, then reset: the same input must give the same output */
    PCL_CHECK_EQ_INT(pcl_svf_set_cutoff(&f, HZ_OF(90.0)), PCL_OK);
    run_blocks(&f, x, b, NULL, NULL, mod, 777, 100);
    PCL_CHECK_EQ_INT(pcl_svf_set_cutoff(&f, cfg.cutoff_hz), PCL_OK);
    run_blocks(&f, x, b, NULL, NULL, mod, 3000, 256);
    pcl_svf_reset(&f);
    run_blocks(&f, x, b, NULL, NULL, mod, N, 256);
    PCL_CHECK(memcmp(a, b, N * sizeof(pcl_sample_t)) == 0);
    free(x);
    free(mod);
    free(a);
    free(b);
}

static void test_aliasing_and_null_outputs(void)
{
    enum
    {
        N = 3000
    };
    const pcl_svf_config_t cfg = make_cfg(48000u, 900.0, 0.6, 0.0, 0);
    pcl_svf_t              f0, f1;
    pcl_sample_t          *x = alloc_samples(N);
    pcl_sample_t          *lp = alloc_samples(N);
    pcl_sample_t          *bp = alloc_samples(N);
    pcl_sample_t          *hp = alloc_samples(N);
    pcl_sample_t          *w = alloc_samples(N);
    int                    which;

    fill_noise(x, N, 0.4);
    PCL_CHECK_EQ_INT(pcl_svf_init(&f0, &cfg), PCL_OK);
    pcl_svf_process(&f0, x, lp, bp, hp, NULL, N);
    for (which = 0; which < 3; which++)
    {
        pcl_sample_t *ref = which == 0 ? lp : (which == 1 ? bp : hp);
        PCL_CHECK_EQ_INT(pcl_svf_init(&f1, &cfg), PCL_OK);
        memcpy(w, x, N * sizeof(pcl_sample_t));
        /* the input buffer doubles as one output; the other two are not computed */
        pcl_svf_process(
            &f1, w, which == 0 ? w : NULL, which == 1 ? w : NULL, which == 2 ? w : NULL, NULL, N);
        PCL_CHECK(memcmp(w, ref, N * sizeof(pcl_sample_t)) == 0);
    }
    /* each output alone is the same as with all three */
    PCL_CHECK_EQ_INT(pcl_svf_init(&f1, &cfg), PCL_OK);
    memset(w, 0, N * sizeof(pcl_sample_t));
    pcl_svf_process(&f1, x, NULL, w, NULL, NULL, N);
    PCL_CHECK(memcmp(w, bp, N * sizeof(pcl_sample_t)) == 0);
    /* single-mode call in place */
    PCL_CHECK_EQ_INT(pcl_svf_init(&f1, &cfg), PCL_OK);
    memcpy(w, x, N * sizeof(pcl_sample_t));
    pcl_svf_process_mode(&f1, w, w, PCL_SVF_HP, NULL, N);
    PCL_CHECK(memcmp(w, hp, N * sizeof(pcl_sample_t)) == 0);
    /* a modulation input of zeros is the same as none */
    {
        pcl_sample_t *z = alloc_samples(N);
        PCL_CHECK_EQ_INT(pcl_svf_init(&f1, &cfg), PCL_OK);
        PCL_CHECK_EQ_INT(pcl_svf_set_mod_depth(&f1, OCT_OF(4.0)), PCL_OK);
        pcl_svf_process(&f1, x, w, NULL, NULL, z, N);
        PCL_CHECK(memcmp(w, lp, N * sizeof(pcl_sample_t)) == 0);
        free(z);
    }
    free(x);
    free(lp);
    free(bp);
    free(hp);
    free(w);
}

/* ---- Modulation and smoothing ---- */

static double eff_cutoff_hz(const pcl_svf_t *f) { return 48000.0 / pi * atan(WIDE_D(f->g)); }

static void test_modulation_timing(void)
{
    enum
    {
        N = 400
    };
    const pcl_svf_config_t cfg = make_cfg(48000u, 500.0, 0.6, 3.0, 0);
    pcl_svf_t              fa, fb;
    pcl_sample_t          *x = alloc_samples(N);
    pcl_sample_t          *m = alloc_samples(N);
    pcl_sample_t          *a = alloc_samples(N);
    pcl_sample_t          *b = alloc_samples(N);
    int                    step_at;
    int                    i;

    fill_noise(x, N, 0.4);
    /* the modulation input steps from 0 to full scale mid-period: a control point only samples it
       at the start of the next period, so nothing changes before then, bit for bit */
    for (step_at = 70; step_at <= 80; step_at += 10)
    {
        for (i = 0; i < N; i++)
            m[i] = i >= step_at ? SMP_OF(0.9) : SMP_OF(0.0);
        PCL_CHECK_EQ_INT(pcl_svf_init(&fa, &cfg), PCL_OK);
        PCL_CHECK_EQ_INT(pcl_svf_init(&fb, &cfg), PCL_OK);
        pcl_svf_process(&fa, x, a, NULL, NULL, NULL, N);
        pcl_svf_process(&fb, x, b, NULL, NULL, m, N);
        {
            const int first_tick
                = ((step_at + PCL_CONTROL_DIV - 1) / PCL_CONTROL_DIV) * PCL_CONTROL_DIV;
            PCL_CHECK(memcmp(a, b, (size_t)first_tick * sizeof(pcl_sample_t)) == 0);
            /* ...and it has changed by the end of the period that follows */
            PCL_CHECK(memcmp(a + first_tick + PCL_CONTROL_DIV, b + first_tick + PCL_CONTROL_DIV,
                          64 * sizeof(pcl_sample_t))
                != 0);
        }
    }
    /* A constant modulation of 0.5 with a 2-octave depth is a cutoff one octave higher. */
    {
        const pcl_svf_config_t base = make_cfg(48000u, 500.0, 0.4, 2.0, 0);
        const pcl_svf_config_t up = make_cfg(48000u, 1000.0, 0.4, 0.0, 0);
        enum
        {
            M = 20000
        };
        pcl_sample_t *xm = alloc_samples(M);
        pcl_sample_t *mm = alloc_samples(M);
        pcl_sample_t *l1 = alloc_samples(M);
        pcl_sample_t *l2 = alloc_samples(M);
        double        e = 0, p = 0;

        fill_noise(xm, M, 0.4);
        for (i = 0; i < M; i++)
            mm[i] = SMP_OF(0.5);
        PCL_CHECK_EQ_INT(pcl_svf_init(&fa, &base), PCL_OK);
        PCL_CHECK_EQ_INT(pcl_svf_init(&fb, &up), PCL_OK);
        pcl_svf_process(&fa, xm, l1, NULL, NULL, mm, M);
        pcl_svf_process(&fb, xm, l2, NULL, NULL, NULL, M);
        for (i = 1000; i < M; i++)
        {
            const double d = SMP_D(l1[i]) - SMP_D(l2[i]);
            e += d * d;
            p += SMP_D(l2[i]) * SMP_D(l2[i]);
        }
        PCL_CHECK(sqrt(e / p) < (IS_FLOAT ? 1.0e-3 : 2.0e-3));
        free(xm);
        free(mm);
        free(l1);
        free(l2);
    }
    free(x);
    free(m);
    free(a);
    free(b);
}

static void test_setters_glide_in_octaves(void)
{
    enum
    {
        PERIODS = 7
    };
    const pcl_svf_config_t cfg = make_cfg(48000u, 200.0, 0.3, 0.0, 0);
    pcl_svf_t              f;
    pcl_sample_t           x[PCL_CONTROL_DIV * PERIODS];
    pcl_sample_t           y[PCL_CONTROL_DIV * PERIODS];
    const long             ramp = (long)pcl_smooth_ramp_samples(48000u);
    double                 frac, want;
    size_t                 i;

    for (i = 0; i < sizeof x / sizeof x[0]; i++)
        x[i] = 0;
    PCL_CHECK_EQ_INT(pcl_svf_init(&f, &cfg), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_svf_set_cutoff(&f, HZ_OF(3200.0)), PCL_OK); /* four octaves up */
    pcl_svf_process(&f, x, y, NULL, NULL, NULL, sizeof x / sizeof x[0]);
    /* After 7 control points the smoother has advanced 7 periods: the cutoff is that fraction of
       the way in OCTAVES (the geometric mean at half), not in Hz. */
    frac = (double)(PERIODS * PCL_CONTROL_DIV) / (double)ramp;
    want = HZ_D(cfg.cutoff_hz) * pow(16.0, frac);
    PCL_CHECK_NEAR(1200.0 * log2(eff_cutoff_hz(&f) / want), 0.0, 3.0); /* cents */
    /* it lands exactly after the whole ramp */
    {
        pcl_sample_t z[256];
        memset(z, 0, sizeof z);
        pcl_svf_process(&f, z, NULL, NULL, NULL, NULL, 256);
        PCL_CHECK_NEAR(1200.0 * log2(eff_cutoff_hz(&f) / HZ_D(HZ_OF(3200.0))), 0.0, 0.3);
    }
}

/* A sweep through four octaves while a sine plays: no sample-to-sample jump beyond the signal's
 * own. */
static void test_sweeps_do_not_click(void)
{
    enum
    {
        N = 24000
    };
    const pcl_svf_config_t cfg = make_cfg(48000u, 200.0, 0.0, 4.0, 0);
    pcl_svf_t              f;
    pcl_sample_t          *x = alloc_samples(N);
    pcl_sample_t          *m = alloc_samples(N);
    pcl_sample_t          *y = alloc_samples(N);
    double                 maxstep = 0.0;
    int                    i;

    for (i = 0; i < N; i++)
    {
        x[i] = SMP_OF(0.5 * sin(2.0 * pi * 800.0 * i / 48000.0));
        m[i] = SMP_OF(sin(2.0 * pi * 4.0 * i / 48000.0)); /* a 4 Hz triangle-ish sweep */
    }
    PCL_CHECK_EQ_INT(pcl_svf_init(&f, &cfg), PCL_OK);
    pcl_svf_process(&f, x, y, NULL, NULL, m, N);
    for (i = 1; i < N; i++)
    {
        const double d = fabs(SMP_D(y[i]) - SMP_D(y[i - 1]));
        if (d > maxstep)
            maxstep = d;
    }
    /* r = 0 never exceeds unity gain, so the output slope is bounded by the input's: 0.5*2pi*800/fs
     */
    PCL_CHECK(maxstep < 0.5 * 2.0 * pi * 800.0 / 48000.0 * 1.3);
    free(x);
    free(m);
    free(y);
}

static void test_cutoff_clamps(void)
{
    enum
    {
        N = 2000
    };
    const pcl_svf_config_t cfg = make_cfg(48000u, 15000.0, 1.0, 7.9, 0);
    pcl_svf_t              f;
    pcl_sample_t          *x = alloc_samples(N);
    pcl_sample_t          *m = alloc_samples(N);
    pcl_sample_t          *y = alloc_samples(N);
    int                    i;

    fill_noise(x, N, 0.3);
    for (i = 0; i < N; i++)
        m[i] = SMP_OF((i / 160) % 2 ? 0.999 : -0.999); /* far above and far below the range */
    PCL_CHECK_EQ_INT(pcl_svf_init(&f, &cfg), PCL_OK);
    pcl_svf_process(&f, x, y, NULL, NULL, m, N);
    PCL_CHECK(WIDE_D(f.g) > 0.0 && WIDE_D(f.g) < 6.4); /* tan(0.45 pi) = 6.31 */
    for (i = 0; i < N; i++)
        PCL_CHECK(fabs(SMP_D(y[i])) <= 8.0);
    free(x);
    free(m);
    free(y);
}

/* After a modulated passage, the cutoff must come back to its base value (stale coefficients). */
static void test_cutoff_returns_after_modulation(void)
{
    enum
    {
        N = 6000
    };
    const pcl_svf_config_t cfg = make_cfg(48000u, 500.0, 0.3, 3.0, 0);
    pcl_svf_t              f;
    pcl_sample_t          *x = alloc_samples(N);
    pcl_sample_t          *m = alloc_samples(N);
    pcl_sample_t          *y = alloc_samples(N);
    int                    i;

    for (i = 0; i < N; i++)
        m[i] = SMP_OF(0.9);
    PCL_CHECK_EQ_INT(pcl_svf_init(&f, &cfg), PCL_OK);
    PCL_CHECK_NEAR(eff_cutoff_hz(&f), HZ_D(cfg.cutoff_hz), HZ_D(cfg.cutoff_hz) * 1e-3);
    pcl_svf_process(&f, x, y, NULL, NULL, m, N);
    PCL_CHECK(eff_cutoff_hz(&f) > 2.0 * HZ_D(cfg.cutoff_hz)); /* it moved, by 2.7 octaves */
    pcl_svf_process(&f, x, y, NULL, NULL, NULL, 64); /* and with no input at all it comes back */
    PCL_CHECK_NEAR(1200.0 * log2(eff_cutoff_hz(&f) / HZ_D(cfg.cutoff_hz)), 0.0, 1.0); /* cents */
    free(x);
    free(m);
    free(y);
}

/* At the end of every control period the coefficients are exactly the targets the tick computed. */
static void test_coefficients_land_on_their_targets(void)
{
    enum
    {
        N = 16 * 400
    };
    const pcl_svf_config_t cfg = make_cfg(48000u, 700.0, 0.6, 3.0, 0);
    pcl_svf_t              f;
    pcl_sample_t          *x = alloc_samples(16);
    pcl_sample_t          *m = alloc_samples(N);
    pcl_sample_t           y[16];
    int                    bad = 0;
    int                    p;

    fill_noise(m, N, 1.0);
    PCL_CHECK_EQ_INT(pcl_svf_init(&f, &cfg), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_svf_set_cutoff(&f, HZ_OF(5000.0)), PCL_OK); /* the smoother runs too */
    for (p = 0; p < N / 16; p++)
    {
        pcl_svf_process(&f, x, y, NULL, NULL, m + 16 * p, 16);
        if (f.g != f.g_end || f.c != f.c_end || f.a1 != f.a1_end || f.ac != f.ac_end)
            bad++;
    }
    PCL_CHECK_EQ_INT(bad, 0);
    free(x);
    free(m);
}

/* ---- Resonance ---- */

static void test_resonance_mapping(void)
{
    /* Q at the cutoff equals 1/k: Butterworth at r = 0, 2.83 at 0.5, 17.7 at 0.8 */
    static const double rs[] = { 0.0, 0.5, 0.8 };
    static const double qs[] = { 0.70710678, 2.8284271, 17.67767 };
    size_t              i;

    for (i = 0; i < 3; i++)
    {
        const pcl_svf_config_t cfg = make_cfg(48000u, 2000.0, rs[i], 0.0, 0);
        pcl_svf_t              f;
        const double           fs = 48000.0;
        const double           ft = fs * floor(2000.0 / fs * 32768.0 + 0.5) / 32768.0;
        double                 mag[3], ph[3];
        resp_t                 got;

        PCL_CHECK_EQ_INT(pcl_svf_init(&f, &cfg), PCL_OK);
        analytic(ft, fs, tan(pi * HZ_D(cfg.cutoff_hz) / fs), damping_of(rs[i]), mag, ph);
        got = measure(&f, ft, 0.03, fs, damping_of(rs[i]), 2000.0);
        PCL_CHECK_NEAR(got.amp[0] / 0.03, qs[i], qs[i] * 0.01);
    }
}

static void test_self_oscillation(void)
{
    enum
    {
        N = 96000
    };
    const pcl_svf_config_t cfg = make_cfg(48000u, 1000.0, 1.0, 0.0, 0);
    pcl_svf_t              f;
    pcl_sample_t          *x = alloc_samples(N);
    pcl_sample_t          *y = alloc_samples(N);
    double                 a1 = 0.0, a2 = 0.0;
    int                    crossings = 0;
    int                    i;

    x[0] = SMP_OF(0.9); /* a single impulse */
    PCL_CHECK_EQ_INT(pcl_svf_init(&f, &cfg), PCL_OK);
    run_blocks(&f, x, y, NULL, NULL, NULL, N, 256);
    for (i = 4800; i < 9600; i++)
        a1 = fmax(a1, fabs(SMP_D(y[i])));
    for (i = N - 4800; i < N; i++)
        a2 = fmax(a2, fabs(SMP_D(y[i])));
    PCL_CHECK(a1 > 0.02);                      /* it is ringing... */
    PCL_CHECK(a2 > 0.9 * a1 && a2 < 1.1 * a1); /* ...and neither dies away nor grows in 2 s */
    for (i = 48000 + 1; i < 96000; i++)
        if ((SMP_D(y[i - 1]) < 0.0) != (SMP_D(y[i]) < 0.0))
            crossings++;
    /* 1000 Hz for 1 s: 2000 zero crossings */
    PCL_CHECK(abs(crossings - 2000) <= 2);
    free(x);
    free(y);
}

/* ---- Saturation ---- */

static void test_saturate_small_signal_is_linear(void)
{
    enum
    {
        N = 20000
    };
    const pcl_svf_config_t lin = make_cfg(48000u, 1000.0, 0.5, 0.0, 0);
    const pcl_svf_config_t sat = make_cfg(48000u, 1000.0, 0.5, 0.0, 1);
    pcl_svf_t              fl, fs_;
    pcl_sample_t          *x = alloc_samples(N);
    pcl_sample_t          *a = alloc_samples(N);
    pcl_sample_t          *b = alloc_samples(N);
    double                 e = 0, p = 0;
    int                    i;

    for (i = 0; i < N; i++)
        x[i] = SMP_OF(0.02 * sin(2.0 * pi * 700.0 * i / 48000.0));
    PCL_CHECK_EQ_INT(pcl_svf_init(&fl, &lin), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_svf_init(&fs_, &sat), PCL_OK);
    pcl_svf_process(&fl, x, a, NULL, NULL, NULL, N);
    pcl_svf_process(&fs_, x, b, NULL, NULL, NULL, N);
    for (i = 2000; i < N; i++)
    {
        const double d = SMP_D(a[i]) - SMP_D(b[i]);
        e += d * d;
        p += SMP_D(a[i]) * SMP_D(a[i]);
    }
    PCL_CHECK(sqrt(e / p) < (PCL_SAMPLE_TYPE == PCL_Q15 ? 0.03 : 0.003));
    free(x);
    free(a);
    free(b);
}

/* ---- Stability ---- */

#if PCL_SAMPLE_TYPE == PCL_FLOAT
#define STATE_D(s) ((double)(s))
#else
#define STATE_D(s) ((double)(s) / 268435456.0)
#endif

/* Self-oscillating filter, fast random cutoff modulation, loud noise bursts: nothing runs away. */
static void stability_case(
    double r, double depth, int sat, long seconds, double lp_bound, double s1_bound)
{
    enum
    {
        BLK = 256
    };
    const pcl_svf_config_t cfg = make_cfg(48000u, 900.0, r, depth, sat);
    pcl_svf_t              f;
    pcl_sample_t           x[BLK], m[BLK], lp[BLK], bp[BLK], hp[BLK];
    long                   blocks = seconds * 48000L / BLK;
    long                   b;
    double                 peak_lp = 0.0, peak_s1 = 0.0, peak_s2 = 0.0;
    int                    bad = 0;

    PCL_CHECK_EQ_INT(pcl_svf_init(&f, &cfg), PCL_OK);
    for (b = 0; b < blocks; b++)
    {
        int i;
        for (i = 0; i < BLK; i++)
        {
            /* a loud burst every ~0.4 s, otherwise silence; the modulation is full-scale noise */
            x[i] = (b % 64) < 4 ? SMP_OF(0.9 * frand()) : SMP_OF(0.0);
            m[i] = SMP_OF(frand());
        }
        pcl_svf_process(&f, x, lp, bp, hp, m, BLK);
        for (i = 0; i < BLK; i++)
        {
            const double v = fabs(SMP_D(lp[i]));
#if PCL_SAMPLE_TYPE == PCL_FLOAT
            if (!isfinite(lp[i]) || !isfinite(bp[i]) || !isfinite(hp[i]))
                bad++;
#endif
            if (v > peak_lp)
                peak_lp = v;
        }
        if (fabs(STATE_D(f.s1)) > peak_s1)
            peak_s1 = fabs(STATE_D(f.s1));
        if (fabs(STATE_D(f.s2)) > peak_s2)
            peak_s2 = fabs(STATE_D(f.s2));
    }
    PCL_CHECK_EQ_INT(bad, 0);
    PCL_CHECK(peak_s1 <= s1_bound);
    PCL_CHECK(peak_s2 <= 8.0);
    PCL_CHECK(peak_lp <= lp_bound);
}

static void test_stability_under_fast_modulation(void)
{
    /* clamp only: the states stay within +-8 whatever happens. Float outputs are not clamped; they
       followed the states to 8.9 at worst (provably under 130). Fixed-point outputs saturate. */
    stability_case(1.0, 6.0, 0, 600, 20.0, 8.0);
    stability_case(0.97, 7.5, 0, 60, 20.0, 8.0);
    /* with the tanh limiter the band-pass state is held to +-1 and the output near full scale */
    stability_case(1.0, 6.0, 1, 60, 20.0, 1.0001);
    stability_case(0.999, 4.0, 1, 60, 20.0, 1.0001);
}

/*
 * With any damping the filter must stay well inside its +-8 limit even when the cutoff jumps by
 * several octaves every control period. (Interpolating a1 linearly across the period broke exactly
 * this: the chord of the convex 1/(1+g*c) overestimates a1, which is negative damping, and the
 * filter ran into the clamp even at Butterworth damping.)
 */
static void heavy_modulation_case(double r, double depth, double bound)
{
    enum
    {
        BLK = 256
    };
    const pcl_svf_config_t cfg = make_cfg(48000u, 3000.0, r, depth, 0);
    pcl_svf_t              f;
    pcl_sample_t           x[BLK], m[BLK], lp[BLK];
    double                 peak = 0.0;
    int                    b, i;

    PCL_CHECK_EQ_INT(pcl_svf_init(&f, &cfg), PCL_OK);
    for (b = 0; b < 40 * 48000 / BLK; b++)
    {
        for (i = 0; i < BLK; i++)
        {
            x[i]
                = b < 20 * 48000 / BLK ? SMP_OF(0.4 * frand()) : SMP_OF(0.0); /* noise, then ring */
            m[i] = SMP_OF(frand());
        }
        pcl_svf_process(&f, x, lp, NULL, NULL, m, BLK);
        if (fabs(STATE_D(f.s1)) > peak)
            peak = fabs(STATE_D(f.s1));
        if (fabs(STATE_D(f.s2)) > peak)
            peak = fabs(STATE_D(f.s2));
    }
    PCL_CHECK(peak < bound);
}

static void test_heavy_modulation_needs_no_clamp(void)
{
    /* Observed peaks of the states, float / Q15 / Q31 (the exact per-sample filter is the same):
       1.9 / 1.6 / 1.9 for Butterworth. The old linearly interpolated a1 ran into the +-8 limit at
       r = 0. Higher resonance under this violence is not a fair test: white-noise modulation of
       +-7 octaves every 16 samples drives even the exact filter into the limit by parametric
       pumping, and where it lands depends on the random sequence. */
    heavy_modulation_case(0.0, 4.0, 2.5);
    heavy_modulation_case(0.0, 7.5, 2.5);
    heavy_modulation_case(0.2, 6.0, 4.0);
}

#if PCL_SAMPLE_TYPE == PCL_FLOAT
static void test_float_denormals_are_flushed(void)
{
    enum
    {
        BLK = 256
    };
    const pcl_svf_config_t cfg = make_cfg(48000u, 3000.0, 0.5, 0.0, 0);
    pcl_svf_t              f;
    pcl_sample_t           x[BLK], lp[BLK], bp[BLK], hp[BLK];
    int                    b, i;
    int                    subnormal = 0;

    PCL_CHECK_EQ_INT(pcl_svf_init(&f, &cfg), PCL_OK);
    for (i = 0; i < BLK; i++)
        x[i] = 0.5f * (float)frand();
    pcl_svf_process(&f, x, lp, bp, hp, NULL, BLK);
    for (i = 0; i < BLK; i++)
        x[i] = 0.0f;
    /* 20 s of silence: the decay passes through the denormal range on its way to zero */
    for (b = 0; b < 20 * 48000 / BLK; b++)
    {
        pcl_svf_process(&f, x, lp, bp, hp, NULL, BLK);
        for (i = 0; i < BLK; i++)
            if (fpclassify(lp[i]) == FP_SUBNORMAL || fpclassify(bp[i]) == FP_SUBNORMAL
                || fpclassify(hp[i]) == FP_SUBNORMAL)
                subnormal++;
        if (fpclassify(f.s1) == FP_SUBNORMAL || fpclassify(f.s2) == FP_SUBNORMAL)
            subnormal++;
    }
    PCL_CHECK_EQ_INT(subnormal, 0);
    /* and it really did reach silence: only the anti-denormal guard (1e-24) is left */
    PCL_CHECK(fabsf(f.s1) < 1.0e-20f && fabsf(f.s2) < 1.0e-20f);
}

static void test_float_garbage_in_recovers(void)
{
    const pcl_svf_config_t cfg = make_cfg(48000u, 3000.0, 0.9, 0.0, 0);
    pcl_svf_t              f;
    pcl_sample_t           x[12]
        = { 0.0f, NAN, INFINITY, -INFINITY, 1.0e30f, 0.0f, 0.3f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    pcl_sample_t lp[12], bp[12], hp[12];
    int          i;

    /* Non-finite input is not supported (the output follows it), but it must not poison the filter:
       the states are clamped, so the first finite samples afterwards come out finite again. */
    PCL_CHECK_EQ_INT(pcl_svf_init(&f, &cfg), PCL_OK);
    pcl_svf_process(&f, x, lp, bp, hp, NULL, 12);
    PCL_CHECK(isfinite(f.s1) && isfinite(f.s2));
    for (i = 6; i < 12; i++)
        PCL_CHECK(isfinite(lp[i]) && isfinite(bp[i]) && isfinite(hp[i]));
}
#endif

/* After the input stops, a damped filter decays to the quantisation floor. */
static void test_decays_to_the_floor(void)
{
    enum
    {
        BLK = 256
    };
    const pcl_svf_config_t cfg = make_cfg(48000u, 1000.0, 0.7, 0.0, 0);
    pcl_svf_t              f;
    pcl_sample_t           x[BLK], lp[BLK], bp[BLK], hp[BLK];
    int                    b, i;
    double                 tail = 0.0;

    PCL_CHECK_EQ_INT(pcl_svf_init(&f, &cfg), PCL_OK);
    for (i = 0; i < BLK; i++)
        x[i] = SMP_OF(0.5 * frand());
    pcl_svf_process(&f, x, lp, bp, hp, NULL, BLK);
    for (i = 0; i < BLK; i++)
        x[i] = SMP_OF(0.0);
    for (b = 0; b < 5 * 48000 / BLK; b++)
        pcl_svf_process(&f, x, lp, bp, hp, NULL, BLK);
    for (i = 0; i < BLK; i++)
        tail = fmax(tail, fmax(fabs(SMP_D(lp[i])), fmax(fabs(SMP_D(bp[i])), fabs(SMP_D(hp[i])))));
#if PCL_SAMPLE_TYPE == PCL_Q15
    PCL_CHECK(tail <= 2.0 / 32768.0);
#else
    PCL_CHECK(tail < 1.0e-6); /* -120 dBFS */
#endif
}

int main(void)
{
    PCL_RUN_TEST(test_init_and_setters);
    PCL_RUN_TEST(test_response_matches_analogue_prototype);
    PCL_RUN_TEST(test_cutoff_tuning);
    PCL_RUN_TEST(test_resonance_mapping);
    PCL_RUN_TEST(test_modes_are_sums_of_the_three_outputs);
    PCL_RUN_TEST(test_allpass_is_flat);
    PCL_RUN_TEST(test_matches_time_domain_model);
    PCL_RUN_TEST(test_saturate_matches_model);
    PCL_RUN_TEST(test_block_size_independence);
    PCL_RUN_TEST(test_reset_is_deterministic);
    PCL_RUN_TEST(test_aliasing_and_null_outputs);
    PCL_RUN_TEST(test_modulation_timing);
    PCL_RUN_TEST(test_setters_glide_in_octaves);
    PCL_RUN_TEST(test_sweeps_do_not_click);
    PCL_RUN_TEST(test_cutoff_clamps);
    PCL_RUN_TEST(test_cutoff_returns_after_modulation);
    PCL_RUN_TEST(test_coefficients_land_on_their_targets);
    PCL_RUN_TEST(test_self_oscillation);
    PCL_RUN_TEST(test_saturate_small_signal_is_linear);
    PCL_RUN_TEST(test_stability_under_fast_modulation);
    PCL_RUN_TEST(test_heavy_modulation_needs_no_clamp);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    PCL_RUN_TEST(test_float_denormals_are_flushed);
    PCL_RUN_TEST(test_float_garbage_in_recovers);
#endif
    PCL_RUN_TEST(test_decays_to_the_floor);
    return pcl_test_summary("svf");
}
