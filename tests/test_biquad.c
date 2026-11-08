/*
 * pcl_biquad tests: init and setters, the coefficients against the RBJ cookbook, the response of
 * the running filter against the cookbook, gains at the landmarks, precision (SNR) at low cutoffs,
 * block-size independence, aliasing, smoothing, type changes, modulation timing, a time-domain
 * model that follows the control-rate ramps, and stability under smooth and violent modulation.
 */
#include <complex.h>
#include <stdlib.h>
#include <string.h>

#include "pcl_test.h"

#if PCL_SAMPLE_TYPE == PCL_FLOAT
#define HZ_OF(x) ((float)(x))
#define HZ_D(x) ((double)(x))
#define Q_OF(x) ((float)(x))
#define Q_D(x) ((double)(x))
#define DB_OF(x) ((float)(x))
#define DB_D(x) ((double)(x))
#define OCT_OF(x) ((float)(x))
#define OCT_D(x) ((double)(x))
#define IS_FLOAT 1
#else
#define HZ_OF(x) ((uint32_t)floor((x) * 65536.0 + 0.5))
#define HZ_D(x) ((double)(x) / 65536.0)
#define Q_OF(x) ((uint32_t)floor((x) * 65536.0 + 0.5))
#define Q_D(x) ((double)(x) / 65536.0)
#define DB_OF(x) ((int32_t)floor((x) * 16777216.0 + 0.5))
#define DB_D(x) ((double)(x) / 16777216.0)
#define OCT_OF(x) ((int32_t)floor((x) * 268435456.0 + 0.5))
#define OCT_D(x) ((double)(x) / 268435456.0)
#define IS_FLOAT 0
#endif

#define SMP_OF(x) pcl_test_from_double(x)
#define SMP_D(x) pcl_test_to_double(x)

static const double pi = 3.14159265358979323846;

static const char *const g_type_name[]
    = { "lp", "hp", "bp", "notch", "allpass", "peak", "lowshelf", "highshelf" };

/* ---- Helpers ---- */

static uint32_t g_rng = 0x2545F491u;

static double frand(void) /* uniform in [-1, 1) */
{
    g_rng ^= g_rng << 13;
    g_rng ^= g_rng >> 17;
    g_rng ^= g_rng << 5;
    return (double)(g_rng >> 8) / 8388608.0 - 1.0;
}

static pcl_biquad_config_t make_cfg(
    uint32_t fs, pcl_biquad_type_t type, double fc, double q, double db, double depth_oct)
{
    pcl_biquad_config_t c;

    c.sample_rate_hz = fs;
    c.type = type;
    c.cutoff_hz = HZ_OF(fc);
    c.q = Q_OF(q);
    c.gain_db = DB_OF(db);
    c.mod_depth_oct = OCT_OF(depth_oct);
    return c;
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

static void run_blocks(pcl_biquad_t *f, const pcl_sample_t *in, pcl_sample_t *out,
    const pcl_sample_t *mod, size_t total, size_t blk)
{
    size_t n = 0;

    while (n < total)
    {
        const size_t c = total - n < blk ? total - n : blk;

        pcl_biquad_process(f, in + n, out + n, mod ? mod + n : NULL, c);
        n += c;
    }
}

/* The RBJ cookbook, digital formulas, normalised to a0 = 1: b0 b1 b2 a1 a2. */
static void rbj(int t, double f, double fs, double q, double db, double *c)
{
    const double w0 = 2.0 * pi * f / fs;
    const double al = sin(w0) / (2.0 * q);
    const double co = cos(w0);
    const double A = pow(10.0, db / 40.0);
    double       b[3], a[3];

    switch (t)
    {
    case PCL_BIQUAD_LP:
        b[0] = (1 - co) / 2, b[1] = 1 - co, b[2] = (1 - co) / 2;
        a[0] = 1 + al, a[1] = -2 * co, a[2] = 1 - al;
        break;
    case PCL_BIQUAD_HP:
        b[0] = (1 + co) / 2, b[1] = -(1 + co), b[2] = (1 + co) / 2;
        a[0] = 1 + al, a[1] = -2 * co, a[2] = 1 - al;
        break;
    case PCL_BIQUAD_BP:
        b[0] = al, b[1] = 0, b[2] = -al;
        a[0] = 1 + al, a[1] = -2 * co, a[2] = 1 - al;
        break;
    case PCL_BIQUAD_NOTCH:
        b[0] = 1, b[1] = -2 * co, b[2] = 1;
        a[0] = 1 + al, a[1] = -2 * co, a[2] = 1 - al;
        break;
    case PCL_BIQUAD_ALLPASS:
        b[0] = 1 - al, b[1] = -2 * co, b[2] = 1 + al;
        a[0] = 1 + al, a[1] = -2 * co, a[2] = 1 - al;
        break;
    case PCL_BIQUAD_PEAK:
        b[0] = 1 + al * A, b[1] = -2 * co, b[2] = 1 - al * A;
        a[0] = 1 + al / A, a[1] = -2 * co, a[2] = 1 - al / A;
        break;
    case PCL_BIQUAD_LOWSHELF:
    {
        const double sq = 2.0 * sqrt(A) * al;

        b[0] = A * ((A + 1) - (A - 1) * co + sq);
        b[1] = 2 * A * ((A - 1) - (A + 1) * co);
        b[2] = A * ((A + 1) - (A - 1) * co - sq);
        a[0] = (A + 1) + (A - 1) * co + sq;
        a[1] = -2 * ((A - 1) + (A + 1) * co);
        a[2] = (A + 1) + (A - 1) * co - sq;
        break;
    }
    default: /* PCL_BIQUAD_HIGHSHELF */
    {
        const double sq = 2.0 * sqrt(A) * al;

        b[0] = A * ((A + 1) + (A - 1) * co + sq);
        b[1] = -2 * A * ((A - 1) + (A + 1) * co);
        b[2] = A * ((A + 1) + (A - 1) * co - sq);
        a[0] = (A + 1) - (A - 1) * co + sq;
        a[1] = 2 * ((A - 1) - (A + 1) * co);
        a[2] = (A + 1) - (A - 1) * co - sq;
        break;
    }
    }
    c[0] = b[0] / a[0];
    c[1] = b[1] / a[0];
    c[2] = b[2] / a[0];
    c[3] = a[1] / a[0];
    c[4] = a[2] / a[0];
}

static double complex response(const double *c, double w)
{
    const double complex z = cexp(I * w);

    return (c[0] + c[1] / z + c[2] / (z * z)) / (1.0 + c[3] / z + c[4] / (z * z));
}

/* The module's coefficients as doubles. */
static void module_coefs(const pcl_biquad_t *f, double *c)
{
    unsigned i;

    for (i = 0; i < 5; i++)
#if PCL_SAMPLE_TYPE == PCL_FLOAT
        c[i] = (double)f->c[i];
#else
        c[i] = (double)f->c[i] / (i < 3 ? 134217728.0 : 1073741824.0);
#endif
}

/* ---- Init and setters ---- */

static void test_init_and_setters(void)
{
    pcl_biquad_t        f, keep;
    pcl_biquad_config_t c = make_cfg(48000u, PCL_BIQUAD_PEAK, 1000.0, 2.0, 6.0, 2.0);

    PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &c), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_biquad_init(NULL, &c), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_biquad_init(&f, NULL), PCL_ERR_NULL_POINTER);

    keep = f;
    c = make_cfg(7999u, PCL_BIQUAD_PEAK, 1000.0, 2.0, 6.0, 2.0);
    PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &c), PCL_ERR_OUT_OF_RANGE);
    c = make_cfg(768001u, PCL_BIQUAD_PEAK, 1000.0, 2.0, 6.0, 2.0);
    PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &c), PCL_ERR_OUT_OF_RANGE);
    c = make_cfg(48000u, PCL_BIQUAD_PEAK, 4.0, 2.0, 6.0, 2.0);
    PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &c), PCL_ERR_OUT_OF_RANGE);
    c = make_cfg(48000u, PCL_BIQUAD_PEAK, 21700.0, 2.0, 6.0, 2.0);
    PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &c), PCL_ERR_OUT_OF_RANGE);
    c = make_cfg(48000u, PCL_BIQUAD_PEAK, 1000.0, 0.15, 6.0, 2.0);
    PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &c), PCL_ERR_OUT_OF_RANGE);
    c = make_cfg(48000u, PCL_BIQUAD_PEAK, 1000.0, 120.0, 6.0, 2.0);
    PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &c), PCL_ERR_OUT_OF_RANGE);
    c = make_cfg(48000u, PCL_BIQUAD_PEAK, 1000.0, 2.0, 18.5, 2.0);
    PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &c), PCL_ERR_OUT_OF_RANGE);
    c = make_cfg(48000u, PCL_BIQUAD_PEAK, 1000.0, 2.0, -18.5, 2.0);
    PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &c), PCL_ERR_OUT_OF_RANGE);
    c = make_cfg(48000u, PCL_BIQUAD_PEAK, 1000.0, 2.0, 6.0, -1.0);
    PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &c), PCL_ERR_OUT_OF_RANGE);
    c = make_cfg(48000u, (pcl_biquad_type_t)8, 1000.0, 2.0, 6.0, 2.0);
    PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &c), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK(memcmp(&f, &keep, sizeof f) == 0);

    /* limits are accepted */
    c = make_cfg(48000u, PCL_BIQUAD_LOWSHELF, 5.0, 0.2, -18.0, 0.0);
    PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &c), PCL_OK);
    c = make_cfg(48000u, PCL_BIQUAD_HIGHSHELF, 21599.0, 100.0, 18.0, 7.9);
    PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &c), PCL_OK);
    c = make_cfg(8000u, PCL_BIQUAD_LP, 3599.0, 0.7071, 0.0, 0.0);
    PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &c), PCL_OK);

    /* setters keep the module unchanged on error */
    c = make_cfg(48000u, PCL_BIQUAD_PEAK, 1000.0, 2.0, 6.0, 2.0);
    PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &c), PCL_OK);
    keep = f;
    PCL_CHECK_EQ_INT(pcl_biquad_set_cutoff(NULL, HZ_OF(100.0)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_biquad_set_cutoff(&f, HZ_OF(4.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_biquad_set_cutoff(&f, HZ_OF(22000.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_biquad_set_q(NULL, Q_OF(1.0)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_biquad_set_q(&f, Q_OF(0.1)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_biquad_set_q(&f, Q_OF(101.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_biquad_set_gain(NULL, DB_OF(1.0)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_biquad_set_gain(&f, DB_OF(19.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_biquad_set_gain(&f, DB_OF(-19.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_biquad_set_type(NULL, PCL_BIQUAD_LP), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_biquad_set_type(&f, (pcl_biquad_type_t)9), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_biquad_set_mod_depth(NULL, OCT_OF(1.0)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_biquad_set_mod_depth(&f, OCT_OF(-1.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK(memcmp(&f, &keep, sizeof f) == 0);
    PCL_CHECK_EQ_INT(pcl_biquad_set_cutoff(&f, HZ_OF(5.0)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_biquad_set_q(&f, Q_OF(0.2)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_biquad_set_q(&f, Q_OF(100.0)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_biquad_set_gain(&f, DB_OF(-18.0)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_biquad_set_gain(&f, DB_OF(18.0)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_biquad_set_type(&f, PCL_BIQUAD_HIGHSHELF), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_biquad_set_mod_depth(&f, OCT_OF(3.0)), PCL_OK);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    PCL_CHECK_EQ_INT(pcl_biquad_set_cutoff(&f, NAN), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_biquad_set_q(&f, NAN), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_biquad_set_gain(&f, NAN), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_biquad_set_mod_depth(&f, NAN), PCL_ERR_OUT_OF_RANGE);
#endif
}

/* ---- The coefficients against the cookbook ---- */

/*
 * Worst |H_module - H_cookbook| / (|H_cookbook| + 0.001), in dB, over a log grid of frequencies.
 * The 0.001 floor (-60 dB) keeps the deep stopband from dominating: the feedforward coefficients
 * have 7.5e-9 resolution, which leaves a DC leak around -84 dB in a high-pass.
 */
static double coef_error_db(const pcl_biquad_t *f, int t, double fc, double q, double db, double fs)
{
    double c_mod[5], c_ref[5], worst = 0.0;
    int    k;

    module_coefs(f, c_mod);
    rbj(t, fc, fs, q, db, c_ref);
    for (k = 0; k < 120; k++)
    {
        const double         w = 2.0 * pi * 2.0 / fs * pow(0.49 * fs / 2.0, (double)k / 119.0);
        const double complex h1 = response(c_mod, w);
        const double complex h2 = response(c_ref, w);
        const double         e = 20.0 * log10(1.0 + cabs(h1 - h2) / (cabs(h2) + 1.0e-3));

        if (e > worst)
            worst = e;
    }
    return worst;
}

/* Tolerance (dB) by cutoff, from the measured error of each build; see docs/modules/biquad.md. */
static double coef_tol_db(double fc, int t)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    (void)t;
#endif
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    /* float coefficients: a1 sits near -2 and cannot place the poles precisely at low cutoffs */
    if (fc >= 5000.0)
        return 0.01;
    if (fc >= 900.0)
        return 0.02;
    if (fc >= 90.0)
        return 0.6;
    return 40.0; /* 5 Hz in float: the coefficients cannot place the poles */
#else
    if (fc >= 900.0)
        return t == PCL_BIQUAD_NOTCH ? 0.01 : 0.012;
    if (fc >= 90.0)
        return 0.03;
    if (fc >= 18.0)
        return 1.0;
    return 3.0;
#endif
}

static void test_coefficients_match_cookbook(void)
{
    static const double fcs[] = { 5.0, 20.0, 100.0, 1000.0, 8000.0, 19000.0 };
    static const double qs[] = { 0.2, 0.7071, 2.0, 10.0 };
    static const double dbs[] = { -18.0, -6.0, 6.0, 18.0 };
    int                 t;

    for (t = 0; t < 8; t++)
    {
        size_t a, q, g;

        for (a = 0; a < sizeof fcs / sizeof fcs[0]; a++)
            for (q = 0; q < sizeof qs / sizeof qs[0]; q++)
                for (g = 0; g < (t >= PCL_BIQUAD_PEAK ? sizeof dbs / sizeof dbs[0] : 1u); g++)
                {
                    const double              db = t >= PCL_BIQUAD_PEAK ? dbs[g] : 0.0;
                    const pcl_biquad_config_t c
                        = make_cfg(48000u, (pcl_biquad_type_t)t, fcs[a], qs[q], db, 0.0);
                    pcl_biquad_t f;
                    double       e;

                    PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &c), PCL_OK);
                    e = coef_error_db(&f, t, HZ_D(c.cutoff_hz), Q_D(c.q), DB_D(c.gain_db), 48000.0);
                    if (!(e <= coef_tol_db(fcs[a], t)))
                        printf("    %s fc=%g q=%g dB=%g: %.4f dB (tol %.3f)\n", g_type_name[t],
                            fcs[a], qs[q], db, e, coef_tol_db(fcs[a], t));
                    PCL_CHECK(e <= coef_tol_db(fcs[a], t));
                }
    }
}

/* Same at other sample rates, at a cutoff where every build is accurate. */
static void test_coefficients_at_other_rates(void)
{
    static const uint32_t rates[] = { 8000u, 44100u, 96000u, 192000u };
    size_t                r;
    int                   t;

    for (r = 0; r < sizeof rates / sizeof rates[0]; r++)
        for (t = 0; t < 8; t++)
        {
            const double              fs = (double)rates[r];
            const double              fc = 0.1 * fs;
            const pcl_biquad_config_t c = make_cfg(
                rates[r], (pcl_biquad_type_t)t, fc, 1.5, t >= PCL_BIQUAD_PEAK ? 9.0 : 0.0, 0.0);
            pcl_biquad_t f;

            if (HZ_D(HZ_OF(fc)) > 65000.0)
                continue; /* outside pcl_hz_t */
            PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &c), PCL_OK);
            /* a notch's null is the response most sensitive to the cutoff */
            PCL_CHECK(coef_error_db(&f, t, HZ_D(c.cutoff_hz), Q_D(c.q), DB_D(c.gain_db), fs)
                <= (t == PCL_BIQUAD_NOTCH ? 0.03 : 0.012));
        }
}

/* ---- Gain landmarks (from the module's coefficients) ---- */

static void test_gain_landmarks(void)
{
    static const double dbs[] = { -18.0, -9.0, -3.0, 3.0, 9.0, 18.0 };
    size_t              g;

    for (g = 0; g < sizeof dbs / sizeof dbs[0]; g++)
    {
        const double        db = dbs[g];
        double              c[5];
        pcl_biquad_t        f;
        pcl_biquad_config_t cfg;

        /* peaking: exactly gain_db at the centre, 0 dB far away */
        cfg = make_cfg(48000u, PCL_BIQUAD_PEAK, 2000.0, 3.0, db, 0.0);
        PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &cfg), PCL_OK);
        module_coefs(&f, c);
        PCL_CHECK_NEAR(20.0 * log10(cabs(response(c, 2.0 * pi * 2000.0 / 48000.0))), db, 0.01);
        PCL_CHECK_NEAR(20.0 * log10(cabs(response(c, 2.0 * pi * 20.0 / 48000.0))), 0.0, 0.05);
        PCL_CHECK_NEAR(20.0 * log10(cabs(response(c, pi * 0.98))), 0.0, 0.3);
        /* low shelf: gain_db at DC, 0 dB at Nyquist; high shelf: the reverse */
        cfg = make_cfg(48000u, PCL_BIQUAD_LOWSHELF, 1000.0, 0.7071, db, 0.0);
        PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &cfg), PCL_OK);
        module_coefs(&f, c);
        PCL_CHECK_NEAR(20.0 * log10(cabs(response(c, 2.0 * pi * 20.0 / 48000.0))), db, 0.05);
        PCL_CHECK_NEAR(20.0 * log10(cabs(response(c, pi * 0.999))), 0.0, 0.05);
        cfg = make_cfg(48000u, PCL_BIQUAD_HIGHSHELF, 4000.0, 0.7071, db, 0.0);
        PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &cfg), PCL_OK);
        module_coefs(&f, c);
        PCL_CHECK_NEAR(20.0 * log10(cabs(response(c, pi * 0.999))), db, 0.05);
        PCL_CHECK_NEAR(20.0 * log10(cabs(response(c, 2.0 * pi * 40.0 / 48000.0))), 0.0, 0.05);
    }
    /* band-pass: 0 dB at the centre whatever the Q; all-pass: 0 dB everywhere; notch: a null */
    {
        static const double qs[] = { 0.5, 2.0, 10.0, 50.0 };
        size_t              q;

        for (q = 0; q < sizeof qs / sizeof qs[0]; q++)
        {
            double              c[5];
            pcl_biquad_t        f;
            int                 k;
            pcl_biquad_config_t cfg = make_cfg(48000u, PCL_BIQUAD_BP, 1500.0, qs[q], 0.0, 0.0);

            PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &cfg), PCL_OK);
            module_coefs(&f, c);
            PCL_CHECK_NEAR(cabs(response(c, 2.0 * pi * HZ_D(cfg.cutoff_hz) / 48000.0)), 1.0, 2e-3);
            cfg.type = PCL_BIQUAD_ALLPASS;
            PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &cfg), PCL_OK);
            module_coefs(&f, c);
            for (k = 1; k < 60; k++)
                PCL_CHECK_NEAR(cabs(response(c, pi * (double)k / 61.0)), 1.0, 3e-3);
            cfg.type = PCL_BIQUAD_NOTCH;
            PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &cfg), PCL_OK);
            module_coefs(&f, c);
            PCL_CHECK(cabs(response(c, 2.0 * pi * HZ_D(cfg.cutoff_hz) / 48000.0)) < 2e-3);
        }
    }
}

/* ---- The running filter against the cookbook ---- */

typedef struct
{
    double amp;
    double ph;
} resp_t;

/* Steady-state response to a sine at `ft` (a multiple of fs / WIN), by lock-in over a window. */
static resp_t measure(pcl_biquad_t *f, double ft, double amp_in, double fs, double settle_s)
{
    enum
    {
        WIN = 32768,
        BLK = 256
    };
    static pcl_sample_t xb[BLK], yb[BLK];
    resp_t              r;
    double              si = 0.0, sq = 0.0;
    const long          settle = (long)(settle_s * fs);
    const long          total = settle + WIN;
    long                n = 0;

    pcl_biquad_reset(f);
    while (n < total)
    {
        const int c = (int)(total - n < BLK ? total - n : BLK);
        int       i;

        for (i = 0; i < c; i++)
            xb[i] = SMP_OF(amp_in * sin(2.0 * pi * ft * (double)(n + i) / fs));
        pcl_biquad_process(f, xb, yb, NULL, (size_t)c);
        for (i = 0; i < c; i++)
        {
            const long nn = n + i;

            if (nn >= settle)
            {
                si += SMP_D(yb[i]) * sin(2.0 * pi * ft * (double)nn / fs);
                sq += SMP_D(yb[i]) * cos(2.0 * pi * ft * (double)nn / fs);
            }
        }
        n += c;
    }
    r.amp = 2.0 / WIN * sqrt(si * si + sq * sq);
    r.ph = atan2(sq, si);
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

#if PCL_SAMPLE_TYPE == PCL_Q15
#define SINE_MAG_TOL_DB 0.03
#define SINE_PHASE_TOL 0.01
#else
#define SINE_MAG_TOL_DB 0.012
#define SINE_PHASE_TOL 0.002
#endif

static void test_sine_response_matches_cookbook(void)
{
    static const double ratios[] = { 0.1, 0.5, 0.9, 1.0, 1.1, 2.0, 4.0 };
    const double        fs = 48000.0;
    int                 t;

    for (t = 0; t < 8; t++)
    {
        const double              db = t >= PCL_BIQUAD_PEAK ? 9.0 : 0.0;
        const pcl_biquad_config_t cfg
            = make_cfg(48000u, (pcl_biquad_type_t)t, 2000.0, 3.0, db, 0.0);
        pcl_biquad_t f;
        double       c_ref[5];
        size_t       q;

        PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &cfg), PCL_OK);
        rbj(t, HZ_D(cfg.cutoff_hz), fs, Q_D(cfg.q), DB_D(cfg.gain_db), c_ref);
        for (q = 0; q < sizeof ratios / sizeof ratios[0]; q++)
        {
            const int            m = (int)floor(2000.0 * ratios[q] / fs * 32768.0 + 0.5);
            const double         ft = fs * (double)m / 32768.0;
            const double complex h = response(c_ref, 2.0 * pi * ft / fs);
            const double         amp = 0.5 / fmax(1.0, cabs(h));
            resp_t               got = measure(&f, ft, amp, fs, 0.4);

            if (cabs(h) * amp < 3.0e-3)
                continue; /* below the Q15 noise floor */
            PCL_CHECK_NEAR(20.0 * log10(got.amp / (amp * cabs(h))), 0.0, SINE_MAG_TOL_DB);
            if (cabs(h) * amp > 0.02)
                PCL_CHECK_NEAR(wrap_pi(got.ph - carg(h)), 0.0, SINE_PHASE_TOL);
        }
    }
}

/* ---- Precision ---- */

/* SNR of the low-pass output against a double-precision filter with the module's own coefficients.
 */
static double snr_db(double fc, double q, double *max_err)
{
    enum
    {
        N = 120000
    };
    const pcl_biquad_config_t cfg = make_cfg(48000u, PCL_BIQUAD_LP, fc, q, 0.0, 0.0);
    pcl_biquad_t              f;
    pcl_sample_t             *x = alloc_samples(N);
    pcl_sample_t             *y = alloc_samples(N);
    double                    c[5], x1 = 0, x2 = 0, y1 = 0, y2 = 0, e = 0, p = 0, worst = 0;
    int                       n;

    PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &cfg), PCL_OK);
    module_coefs(&f, c);
    for (n = 0; n < N; n++)
        x[n] = SMP_OF(0.25 * frand() + 0.25 * sin(2.0 * pi * 300.0 * n / 48000.0));
    run_blocks(&f, x, y, NULL, N, 256);
    for (n = 0; n < N; n++)
    {
        const double xi = SMP_D(x[n]);
        const double r = c[0] * xi + c[1] * x1 + c[2] * x2 - c[3] * y1 - c[4] * y2;

        x2 = x1, x1 = xi, y2 = y1, y1 = r;
        if (n > 20000)
        {
            const double d = SMP_D(y[n]) - r;

            e += d * d;
            p += r * r;
            if (fabs(d) > worst)
                worst = fabs(d);
        }
    }
    free(x);
    free(y);
    *max_err = worst;
    return 10.0 * log10(p / (e + 1e-300));
}

static void test_precision_at_low_cutoffs(void)
{
    double m;

#if PCL_SAMPLE_TYPE == PCL_Q15
    /* a 16-bit output cannot show an SNR: what matters is that the output is the correctly rounded
       result, within 1.5 LSB, even for a 20 Hz low-pass whose output is a few LSB deep */
    (void)snr_db(20.0, 0.7071, &m);
    PCL_CHECK(m <= 1.5 / 32768.0);
    (void)snr_db(100.0, 8.0, &m);
    PCL_CHECK(m <= 1.5 / 32768.0);
    (void)snr_db(1000.0, 0.7071, &m);
    PCL_CHECK(m <= 1.5 / 32768.0);
#elif PCL_SAMPLE_TYPE == PCL_Q31
    /* error feedback: 111 dB at 20 Hz and 127-131 dB at 100 Hz (59 / 89-94 without it) */
    PCL_CHECK(snr_db(20.0, 0.7071, &m) > 105.0);
    PCL_CHECK(snr_db(20.0, 8.0, &m) > 105.0);
    PCL_CHECK(snr_db(100.0, 0.7071, &m) > 120.0);
    PCL_CHECK(snr_db(100.0, 8.0, &m) > 120.0);
    PCL_CHECK(snr_db(1000.0, 0.7071, &m) > 150.0);
#else
    /* plain float32 Direct Form I: the documented weak spot (45 dB at 20 Hz) */
    PCL_CHECK(snr_db(20.0, 0.7071, &m) > 38.0);
    PCL_CHECK(snr_db(100.0, 0.7071, &m) > 78.0);
    PCL_CHECK(snr_db(1000.0, 0.7071, &m) > 108.0);
    PCL_CHECK(snr_db(8000.0, 0.7071, &m) > 130.0);
#endif
}

#if PCL_SAMPLE_TYPE != PCL_FLOAT
/*
 * The 28-bit history is more precise than any output (a Q15 output is 16 bits), so the error
 * feedback can only be seen in the history itself: after every sample, compare y1 with a
 * double-precision filter that uses the module's coefficients.
 */
static void test_history_precision(void)
{
    enum
    {
        N = 100000
    };
    const pcl_biquad_config_t cfg = make_cfg(48000u, PCL_BIQUAD_LP, 20.0, 0.7071, 0.0, 0.0);
    pcl_biquad_t              f;
    pcl_sample_t              x, y;
    double                    c[5], x1 = 0, x2 = 0, y1 = 0, y2 = 0, e = 0, p = 0;
    int                       n;

    PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &cfg), PCL_OK);
    module_coefs(&f, c);
    for (n = 0; n < N; n++)
    {
        double xi, r;

        x = SMP_OF(0.25 * frand() + 0.25 * sin(2.0 * pi * 300.0 * n / 48000.0));
        xi = SMP_D(x);
        pcl_biquad_process(&f, &x, &y, NULL, 1);
        r = c[0] * xi + c[1] * x1 + c[2] * x2 - c[3] * y1 - c[4] * y2;
        x2 = x1, x1 = xi, y2 = y1, y1 = r;
        if (n > 20000)
        {
            const double d = (double)f.y1 / 268435456.0 - r;

            e += d * d;
            p += r * r;
        }
    }
    PCL_CHECK(10.0 * log10(p / e) > 105.0); /* 111 dB with the error feedback, 59 dB without it */
}
#endif

/* ---- Determinism, block sizes, aliasing ---- */

static void test_block_size_independence(void)
{
    enum
    {
        N = 5000
    };
    static const size_t blocks[] = { 1, 7, 16, 17, 100, 256, 1000 };
    pcl_sample_t       *x = alloc_samples(N);
    pcl_sample_t       *mod = alloc_samples(N);
    pcl_sample_t       *ref = alloc_samples(N);
    pcl_sample_t       *got = alloc_samples(N);
    int                 with_mod;
    size_t              bi;

    fill_noise(x, N, 0.4);
    fill_noise(mod, N, 1.0);
    for (with_mod = 0; with_mod < 2; with_mod++)
    {
        const pcl_biquad_config_t cfg = make_cfg(48000u, PCL_BIQUAD_PEAK, 1200.0, 2.0, 9.0, 2.0);
        pcl_biquad_t              fr;

        PCL_CHECK_EQ_INT(pcl_biquad_init(&fr, &cfg), PCL_OK);
        PCL_CHECK_EQ_INT(
            pcl_biquad_set_cutoff(&fr, HZ_OF(3000.0)), PCL_OK); /* a ramp in progress */
        PCL_CHECK_EQ_INT(pcl_biquad_set_gain(&fr, DB_OF(-6.0)), PCL_OK);
        run_blocks(&fr, x, ref, with_mod ? mod : NULL, N, N);
        for (bi = 0; bi < sizeof blocks / sizeof blocks[0]; bi++)
        {
            pcl_biquad_t f;

            PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &cfg), PCL_OK);
            PCL_CHECK_EQ_INT(pcl_biquad_set_cutoff(&f, HZ_OF(3000.0)), PCL_OK);
            PCL_CHECK_EQ_INT(pcl_biquad_set_gain(&f, DB_OF(-6.0)), PCL_OK);
            run_blocks(&f, x, got, with_mod ? mod : NULL, N, blocks[bi]);
            PCL_CHECK(memcmp(ref, got, N * sizeof(pcl_sample_t)) == 0);
        }
    }
    free(x);
    free(mod);
    free(ref);
    free(got);
}

static void test_reset_and_aliasing(void)
{
    enum
    {
        N = 4000
    };
    const pcl_biquad_config_t cfg = make_cfg(48000u, PCL_BIQUAD_LP, 900.0, 4.0, 0.0, 2.0);
    pcl_biquad_t              f, g;
    pcl_sample_t             *x = alloc_samples(N);
    pcl_sample_t             *mod = alloc_samples(N);
    pcl_sample_t             *a = alloc_samples(N);
    pcl_sample_t             *b = alloc_samples(N);

    fill_noise(x, N, 0.5);
    fill_noise(mod, N, 1.0);
    PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &cfg), PCL_OK);
    run_blocks(&f, x, a, mod, N, 256);
    /* disturb everything, then reset: the same input must give the same output */
    PCL_CHECK_EQ_INT(pcl_biquad_set_cutoff(&f, HZ_OF(90.0)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_biquad_set_type(&f, PCL_BIQUAD_HP), PCL_OK);
    run_blocks(&f, x, b, mod, 777, 100);
    PCL_CHECK_EQ_INT(pcl_biquad_set_cutoff(&f, cfg.cutoff_hz), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_biquad_set_type(&f, PCL_BIQUAD_LP), PCL_OK);
    run_blocks(&f, x, b, mod, 3000, 256);
    pcl_biquad_reset(&f);
    run_blocks(&f, x, b, mod, N, 256);
    PCL_CHECK(memcmp(a, b, N * sizeof(pcl_sample_t)) == 0);

    /* in place gives the same samples as separate buffers */
    PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &cfg), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_biquad_init(&g, &cfg), PCL_OK);
    memcpy(b, x, N * sizeof(pcl_sample_t));
    pcl_biquad_process(&f, x, a, NULL, N);
    pcl_biquad_process(&g, b, b, NULL, N);
    PCL_CHECK(memcmp(a, b, N * sizeof(pcl_sample_t)) == 0);

    /* a modulation input of zeros is the same as none */
    PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &cfg), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_biquad_init(&g, &cfg), PCL_OK);
    {
        pcl_sample_t *z = alloc_samples(N);

        pcl_biquad_process(&f, x, a, NULL, N);
        pcl_biquad_process(&g, x, b, z, N);
        PCL_CHECK(memcmp(a, b, N * sizeof(pcl_sample_t)) == 0);
        free(z);
    }
    free(x);
    free(mod);
    free(a);
    free(b);
}

/* ---- Smoothing, type changes, modulation ---- */

/* For a low-pass, K^2 = (1 + a1 + a2) / (1 - a1 + a2) whatever the Q: the cutoff the coefficients
 * encode. */
static double lp_cutoff_hz(const pcl_biquad_t *f)
{
    double c[5];

    module_coefs(f, c);
    return 48000.0 / pi * atan(sqrt((1.0 + c[3] + c[4]) / (1.0 - c[3] + c[4])));
}

static void test_setters_glide_in_octaves(void)
{
    enum
    {
        PERIODS = 7
    };
    const pcl_biquad_config_t cfg = make_cfg(48000u, PCL_BIQUAD_LP, 400.0, 1.0, 0.0, 0.0);
    pcl_biquad_t              f;
    pcl_sample_t              x[PCL_CONTROL_DIV * PERIODS], y[PCL_CONTROL_DIV * PERIODS];
    const long                ramp = (long)pcl_smooth_ramp_samples(48000u);
    size_t                    i;
    double                    frac, want;

    for (i = 0; i < sizeof x / sizeof x[0]; i++)
        x[i] = 0;
    PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &cfg), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_biquad_set_cutoff(&f, HZ_OF(3200.0)), PCL_OK); /* three octaves up */
    pcl_biquad_process(&f, x, y, NULL, sizeof x / sizeof x[0]);
    /* after 7 control points: that fraction of the way in OCTAVES, not in Hz */
    frac = (double)(PERIODS * PCL_CONTROL_DIV) / (double)ramp;
    want = HZ_D(cfg.cutoff_hz) * pow(8.0, frac);
    PCL_CHECK_NEAR(1200.0 * log2(lp_cutoff_hz(&f) / want), 0.0, IS_FLOAT ? 8.0 : 4.0); /* cents */
    {
        pcl_sample_t z[256];

        memset(z, 0, sizeof z);
        pcl_biquad_process(&f, z, z, NULL, 256);
        PCL_CHECK_NEAR(
            1200.0 * log2(lp_cutoff_hz(&f) / HZ_D(HZ_OF(3200.0))), 0.0, IS_FLOAT ? 3.0 : 0.3);
    }
}

static void test_q_and_gain_glide(void)
{
    const pcl_biquad_config_t cfg = make_cfg(48000u, PCL_BIQUAD_PEAK, 2000.0, 1.0, 0.0, 0.0);
    pcl_biquad_t              f;
    pcl_sample_t              z[512];
    double                    c[5];

    memset(z, 0, sizeof z);
    PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &cfg), PCL_OK);
    /* gain: 0 dB, then +12 dB: the gain at the centre reaches it, passing through the middle */
    PCL_CHECK_EQ_INT(pcl_biquad_set_gain(&f, DB_OF(12.0)), PCL_OK);
    pcl_biquad_process(&f, z, z, NULL, 112); /* 7 control periods of the 240-sample ramp */
    module_coefs(&f, c);
    {
        const double g_mid = 20.0 * log10(cabs(response(c, 2.0 * pi * 2000.0 / 48000.0)));

        PCL_CHECK_NEAR(g_mid, 12.0 * 112.0 / 240.0, 0.1); /* linear in dB */
    }
    pcl_biquad_process(&f, z, z, NULL, 256);
    module_coefs(&f, c);
    PCL_CHECK_NEAR(20.0 * log10(cabs(response(c, 2.0 * pi * 2000.0 / 48000.0))), 12.0, 0.02);
    /* Q: a band-pass bandwidth halves when Q doubles; after the ramp the coefficients are those of
     * Q = 8 */
    PCL_CHECK_EQ_INT(pcl_biquad_set_type(&f, PCL_BIQUAD_BP), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_biquad_set_q(&f, Q_OF(8.0)), PCL_OK);
    pcl_biquad_process(&f, z, z, NULL, 512);
    {
        double ref[5];
        int    k;

        rbj(PCL_BIQUAD_BP, HZ_D(cfg.cutoff_hz), 48000.0, 8.0, 0.0, ref);
        module_coefs(&f, c);
        for (k = 1; k < 40; k++)
        {
            const double w = pi * (double)k / 41.0;

            PCL_CHECK_NEAR(cabs(response(c, w)), cabs(response(ref, w)), 4e-3);
        }
    }
}

static void test_type_change_keeps_state_and_ramps(void)
{
    const pcl_biquad_config_t cfg = make_cfg(48000u, PCL_BIQUAD_LP, 1000.0, 0.7071, 0.0, 0.0);
    pcl_biquad_t              f;
    pcl_sample_t              x[64], y[64], zero[PCL_CONTROL_DIV];
    double                    before[5], after[5], ref[5];
    int                       i;

    memset(zero, 0, sizeof zero);
    for (i = 0; i < 64; i++)
        x[i] = SMP_OF(0.4 * sin(2.0 * pi * 400.0 * i / 48000.0));
    PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &cfg), PCL_OK);
    pcl_biquad_process(&f, x, y, NULL, 64);
    module_coefs(&f, before);
    {
        const pcl_wide_t y1 = f.y1, y2 = f.y2, x1 = f.x1, x2 = f.x2;

        PCL_CHECK_EQ_INT(pcl_biquad_set_type(&f, PCL_BIQUAD_HP), PCL_OK);
        /* nothing happens until the next control point... */
        module_coefs(&f, after);
        PCL_CHECK(memcmp(before, after, sizeof before) == 0);
        PCL_CHECK(
            f.y1 == y1 && f.y2 == y2 && f.x1 == x1 && f.x2 == x2); /* ...and the history stays */
    }
    pcl_biquad_process(&f, zero, y, NULL, PCL_CONTROL_DIV); /* one control period */
    module_coefs(&f, after);
    rbj(PCL_BIQUAD_HP, HZ_D(cfg.cutoff_hz), 48000.0, Q_D(cfg.q), 0.0, ref);
    for (i = 0; i < 5; i++)
        PCL_CHECK_NEAR(after[i], ref[i], IS_FLOAT ? 3e-6 : 1e-6);
    /* the output is continuous through the change: no step beyond the signal's own slope */
    PCL_CHECK(fabs(SMP_D(y[0])) < 0.5);
}

static void test_modulation_timing(void)
{
    enum
    {
        N = 400
    };
    const pcl_biquad_config_t cfg = make_cfg(48000u, PCL_BIQUAD_PEAK, 500.0, 2.0, 9.0, 3.0);
    pcl_biquad_t              fa, fb;
    pcl_sample_t             *x = alloc_samples(N);
    pcl_sample_t             *m = alloc_samples(N);
    pcl_sample_t             *a = alloc_samples(N);
    pcl_sample_t             *b = alloc_samples(N);
    int                       step_at, i;

    fill_noise(x, N, 0.4);
    /* the modulation steps mid-period: a control point only samples it at the start of the next
       period, so nothing changes before then, bit for bit */
    for (step_at = 70; step_at <= 80; step_at += 10)
    {
        const int first_tick
            = ((step_at + PCL_CONTROL_DIV - 1) / PCL_CONTROL_DIV) * PCL_CONTROL_DIV;

        for (i = 0; i < N; i++)
            m[i] = i >= step_at ? SMP_OF(0.9) : SMP_OF(0.0);
        PCL_CHECK_EQ_INT(pcl_biquad_init(&fa, &cfg), PCL_OK);
        PCL_CHECK_EQ_INT(pcl_biquad_init(&fb, &cfg), PCL_OK);
        pcl_biquad_process(&fa, x, a, NULL, N);
        pcl_biquad_process(&fb, x, b, m, N);
        PCL_CHECK(memcmp(a, b, (size_t)first_tick * sizeof(pcl_sample_t)) == 0);
        PCL_CHECK(memcmp(a + first_tick + PCL_CONTROL_DIV, b + first_tick + PCL_CONTROL_DIV,
                      64 * sizeof(pcl_sample_t))
            != 0);
    }
    free(x);
    free(m);
    free(a);
    free(b);

    /* a constant modulation of 0.5 with a 2-octave depth is a cutoff one octave higher */
    {
        enum
        {
            M = 20000
        };
        const pcl_biquad_config_t base = make_cfg(48000u, PCL_BIQUAD_LP, 500.0, 2.0, 0.0, 2.0);
        const pcl_biquad_config_t up = make_cfg(48000u, PCL_BIQUAD_LP, 1000.0, 2.0, 0.0, 0.0);
        pcl_sample_t             *xm = alloc_samples(M);
        pcl_sample_t             *mm = alloc_samples(M);
        pcl_sample_t             *l1 = alloc_samples(M);
        pcl_sample_t             *l2 = alloc_samples(M);
        double                    e = 0, p = 0;

        fill_noise(xm, M, 0.4);
        for (i = 0; i < M; i++)
            mm[i] = SMP_OF(0.5);
        PCL_CHECK_EQ_INT(pcl_biquad_init(&fa, &base), PCL_OK);
        PCL_CHECK_EQ_INT(pcl_biquad_init(&fb, &up), PCL_OK);
        pcl_biquad_process(&fa, xm, l1, mm, M);
        pcl_biquad_process(&fb, xm, l2, NULL, M);
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
}

static void test_cutoff_returns_and_coefficients_land(void)
{
    enum
    {
        N = 16 * 300
    };
    const pcl_biquad_config_t cfg = make_cfg(48000u, PCL_BIQUAD_LP, 500.0, 0.7071, 0.0, 3.0);
    pcl_biquad_t              f;
    pcl_sample_t             *x = alloc_samples(16);
    pcl_sample_t             *m = alloc_samples(N);
    pcl_sample_t              y[16];
    int                       p, bad = 0;

    fill_noise(m, N, 1.0);
    PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &cfg), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_biquad_set_cutoff(&f, HZ_OF(4000.0)), PCL_OK);
    /* at the end of every control period the coefficients are exactly the targets */
    for (p = 0; p < N / 16; p++)
    {
        unsigned i;

        pcl_biquad_process(&f, x, y, m + 16 * p, 16);
        for (i = 0; i < 5; i++)
            if (f.c[i] != f.c_end[i])
                bad++;
    }
    PCL_CHECK_EQ_INT(bad, 0);
    /* when the modulation stops, the cutoff comes back to the base value */
    {
        pcl_sample_t zero[16];

        memset(zero, 0, sizeof zero);
        for (p = 0; p < 40; p++)
            pcl_biquad_process(&f, x, y, NULL, 16);
        PCL_CHECK_NEAR(
            1200.0 * log2(lp_cutoff_hz(&f) / HZ_D(HZ_OF(4000.0))), 0.0, IS_FLOAT ? 3.0 : 0.3);
    }
    free(x);
    free(m);
}

/* ---- A time-domain model that follows the control-rate ramps ---- */

typedef struct
{
    double fs, depth;
    int    type;
    double u0, u1, q0, q1, g0,
        g1; /* smoother ends: octaves above 1 kHz / 8, log2(Q) / 8, dB / 18 */
    long   adv, ramp;
    double c[5], c_end[5], step[5];
    double x1, x2, y1, y2;
    int    pos, dirty;
} bqmodel_t;

static double lerp_adv(double a, double b, long adv, long ramp)
{
    return a + (b - a) * (adv >= ramp ? 1.0 : (double)adv / (double)ramp);
}

static void bqmodel_design(const bqmodel_t *m, double mod_oct, double *c)
{
    double f = 1000.0 * pow(2.0, 8.0 * lerp_adv(m->u0, m->u1, m->adv, m->ramp) + mod_oct);

    if (f < 1000.0 / 256.0)
        f = 1000.0 / 256.0;
    if (f > 0.45 * m->fs)
        f = 0.45 * m->fs;
    rbj(m->type, f, m->fs, pow(2.0, 8.0 * lerp_adv(m->q0, m->q1, m->adv, m->ramp)),
        18.0 * lerp_adv(m->g0, m->g1, m->adv, m->ramp), c);
}

static void bqmodel_init(bqmodel_t *m, const pcl_biquad_config_t *cfg)
{
    m->fs = (double)cfg->sample_rate_hz;
    m->type = (int)cfg->type;
    m->depth = OCT_D(cfg->mod_depth_oct);
    m->u0 = m->u1 = log2(HZ_D(cfg->cutoff_hz) / 1000.0) / 8.0;
    m->q0 = m->q1 = log2(Q_D(cfg->q)) / 8.0;
    m->g0 = m->g1 = DB_D(cfg->gain_db) / 18.0;
    m->ramp = (long)pcl_smooth_ramp_samples(cfg->sample_rate_hz);
    m->adv = m->ramp;
    bqmodel_design(m, 0.0, m->c);
    memcpy(m->c_end, m->c, sizeof m->c);
    memset(m->step, 0, sizeof m->step);
    m->x1 = m->x2 = m->y1 = m->y2 = 0.0;
    m->pos = 0;
    m->dirty = 0;
}

static void bqmodel_set_cutoff(bqmodel_t *m, double hz)
{
    m->u0 = lerp_adv(m->u0, m->u1, m->adv, m->ramp);
    m->q0 = lerp_adv(m->q0, m->q1, m->adv, m->ramp);
    m->g0 = lerp_adv(m->g0, m->g1, m->adv, m->ramp);
    m->u1 = log2(hz / 1000.0) / 8.0;
    m->q1 = m->q0;
    m->g1 = m->g0;
    m->adv = 0;
}

static void bqmodel_tick(bqmodel_t *m, int have_mod, double mod)
{
    const int moving = m->adv < m->ramp;
    int       i;

    if (!have_mod && !moving && !m->dirty)
    {
        for (i = 0; i < 5; i++)
            m->step[i] = 0.0;
        return;
    }
    if (moving)
        m->adv += PCL_CONTROL_DIV;
    bqmodel_design(m, have_mod ? mod * m->depth : 0.0, m->c_end);
    for (i = 0; i < 5; i++)
        m->step[i] = (m->c_end[i] - m->c[i]) / PCL_CONTROL_DIV;
    m->dirty = have_mod;
}

static void bqmodel_process(
    bqmodel_t *m, const pcl_sample_t *in, const pcl_sample_t *mod, double *out, size_t n)
{
    size_t i;

    for (i = 0; i < n; i++)
    {
        double x, y;
        int    k;

        if (m->pos == 0)
            bqmodel_tick(m, mod != NULL, mod ? SMP_D(mod[i]) : 0.0);
        for (k = 0; k < 5; k++)
            m->c[k] += m->step[k];
        x = SMP_D(in[i]);
        y = m->c[0] * x + m->c[1] * m->x1 + m->c[2] * m->x2 - m->c[3] * m->y1 - m->c[4] * m->y2;
        m->x2 = m->x1, m->x1 = x, m->y2 = m->y1, m->y1 = y;
        out[i] = y;
        if (++m->pos == PCL_CONTROL_DIV)
        {
            m->pos = 0;
            memcpy(m->c, m->c_end, sizeof m->c);
        }
    }
}

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

static void model_case(
    pcl_biquad_type_t type, double fc, double q, double db, double depth, int use_mod)
{
    enum
    {
        BLK = 256,
        N = 188 * 256
    };
    const pcl_biquad_config_t cfg = make_cfg(48000u, type, fc, q, db, depth);
    pcl_biquad_t              f;
    bqmodel_t                 m;
    pcl_sample_t             *x = alloc_samples(N);
    pcl_sample_t             *mod = alloc_samples(N);
    pcl_sample_t             *y = alloc_samples(N);
    double                   *ref = (double *)calloc(N, sizeof(double));
    size_t                    n;
    int                       i;

    if (ref == NULL)
        exit(2);
    PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &cfg), PCL_OK);
    bqmodel_init(&m, &cfg);
    fill_noise(x, N, 0.3);
    for (i = 0; i < N; i++)
        mod[i] = SMP_OF(0.6 * sin(2.0 * pi * 3.0 * i / 48000.0) + 0.2 * frand());
    for (n = 0; n < N; n += BLK)
    {
        if (n == 12 * BLK)
        {
            const double hz = fc * 2.0 < 12000.0 ? fc * 2.0 : 12000.0;

            PCL_CHECK_EQ_INT(pcl_biquad_set_cutoff(&f, HZ_OF(hz)), PCL_OK);
            bqmodel_set_cutoff(&m, HZ_D(HZ_OF(hz)));
        }
        pcl_biquad_process(&f, x + n, y + n, use_mod ? mod + n : NULL, BLK);
        bqmodel_process(&m, x + n, use_mod ? mod + n : NULL, ref + n, BLK);
    }
    PCL_CHECK(rel_diff(y, ref, N) < (IS_FLOAT ? 1.0e-3 : 6.0e-4));
    free(x);
    free(mod);
    free(y);
    free(ref);
}

static void test_matches_time_domain_model(void)
{
    model_case(PCL_BIQUAD_LP, 1500.0, 0.7071, 0.0, 0.0, 0);
    model_case(PCL_BIQUAD_HP, 2500.0, 2.0, 0.0, 0.0, 0);
    model_case(PCL_BIQUAD_BP, 1200.0, 4.0, 0.0, 1.0, 1);
    model_case(PCL_BIQUAD_NOTCH, 2000.0, 3.0, 0.0, 1.0, 1);
    model_case(PCL_BIQUAD_ALLPASS, 1800.0, 1.5, 0.0, 1.0, 1);
    model_case(PCL_BIQUAD_PEAK, 1500.0, 2.0, 12.0, 1.0, 1);
    model_case(PCL_BIQUAD_LOWSHELF, 1000.0, 0.7071, -12.0, 0.5, 1);
    model_case(PCL_BIQUAD_HIGHSHELF, 3000.0, 1.0, 9.0, 0.5, 1);
}

/* ---- Stability ---- */

#if PCL_SAMPLE_TYPE == PCL_FLOAT
#define HIST_D(v) ((double)(v))
#else
#define HIST_D(v) ((double)(v) / 268435456.0)
#endif

/* Smooth modulation (an LFO and an envelope-like sweep) at high Q: the output stays bounded. */
static void test_smooth_modulation_is_stable(void)
{
    enum
    {
        BLK = 256
    };
    static const double qs[] = { 2.0, 8.0, 17.0 };
    size_t              k;

    for (k = 0; k < sizeof qs / sizeof qs[0]; k++)
    {
        const pcl_biquad_config_t cfg = make_cfg(48000u, PCL_BIQUAD_LP, 1500.0, qs[k], 0.0, 2.0);
        pcl_biquad_t              f;
        pcl_sample_t              x[BLK], m[BLK], y[BLK];
        double                    peak = 0.0;
        long                      b;

        PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &cfg), PCL_OK);
        for (b = 0; b < 60L * 48000L / BLK; b++)
        {
            int i;

            for (i = 0; i < BLK; i++)
            {
                const double t = (double)(b * BLK + i) / 48000.0;

                x[i] = SMP_OF(0.2 * frand());
                m[i] = SMP_OF(0.9 * sin(2.0 * pi * 0.7 * t) * sin(2.0 * pi * 3.1 * t));
            }
            pcl_biquad_process(&f, x, y, m, BLK);
            for (i = 0; i < BLK; i++)
                if (fabs(SMP_D(y[i])) > peak)
                    peak = fabs(SMP_D(y[i]));
        }
        /* fixed point saturates at full scale; float is not clipped, and 0.2 of noise into a Q of
           17 resonates to a few times full scale, not to the clamp */
        PCL_CHECK(peak < (IS_FLOAT ? 8.0 : 1.0001));
    }
}

/* Violent modulation of a high-Q filter: not a supported use, but it must not reach NaN or Inf. */
static void test_violent_modulation_stays_finite(void)
{
    enum
    {
        BLK = 256
    };
    const pcl_biquad_config_t cfg = make_cfg(48000u, PCL_BIQUAD_LP, 1500.0, 70.0, 0.0, 6.0);
    pcl_biquad_t              f;
    pcl_sample_t              x[BLK], m[BLK], y[BLK];
    long                      b;
    int                       bad = 0;
    double                    peak_hist = 0.0;

    PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &cfg), PCL_OK);
    for (b = 0; b < 60L * 48000L / BLK; b++)
    {
        int i;

        for (i = 0; i < BLK; i++)
        {
            x[i] = SMP_OF(0.5 * frand());
            m[i] = SMP_OF(frand());
        }
        pcl_biquad_process(&f, x, y, m, BLK);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
        for (i = 0; i < BLK; i++)
            if (!isfinite(y[i]))
                bad++;
#endif
        if (fabs(HIST_D(f.y1)) > peak_hist)
            peak_hist = fabs(HIST_D(f.y1));
    }
    PCL_CHECK_EQ_INT(bad, 0);
    PCL_CHECK(peak_hist <= 8.0);
}

#if PCL_SAMPLE_TYPE == PCL_FLOAT
static void test_float_denormals_do_not_appear(void)
{
    enum
    {
        BLK = 256
    };
    const pcl_biquad_config_t cfg = make_cfg(48000u, PCL_BIQUAD_LP, 3000.0, 2.0, 0.0, 0.0);
    pcl_biquad_t              f;
    pcl_sample_t              x[BLK], y[BLK];
    int                       b, i, subnormal = 0;

    PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &cfg), PCL_OK);
    for (i = 0; i < BLK; i++)
        x[i] = 0.5f * (float)frand();
    pcl_biquad_process(&f, x, y, NULL, BLK);
    for (i = 0; i < BLK; i++)
        x[i] = 0.0f;
    for (b = 0; b < 20 * 48000 / BLK; b++)
    {
        pcl_biquad_process(&f, x, y, NULL, BLK);
        for (i = 0; i < BLK; i++)
            if (fpclassify(y[i]) == FP_SUBNORMAL)
                subnormal++;
        if (fpclassify(f.y1) == FP_SUBNORMAL || fpclassify(f.y2) == FP_SUBNORMAL)
            subnormal++;
    }
    PCL_CHECK_EQ_INT(subnormal, 0);
}
#endif

/* A decaying filter reaches the quantisation floor, and in fixed point the error feedback does not
 * hold a limit cycle. */
static void test_decays_to_the_floor(void)
{
    enum
    {
        BLK = 256
    };
    const pcl_biquad_config_t cfg = make_cfg(48000u, PCL_BIQUAD_LP, 1000.0, 3.0, 0.0, 0.0);
    pcl_biquad_t              f;
    pcl_sample_t              x[BLK], y[BLK];
    int                       b, i;
    double                    tail = 0.0;

    PCL_CHECK_EQ_INT(pcl_biquad_init(&f, &cfg), PCL_OK);
    for (i = 0; i < BLK; i++)
        x[i] = SMP_OF(0.5 * frand());
    pcl_biquad_process(&f, x, y, NULL, BLK);
    for (i = 0; i < BLK; i++)
        x[i] = SMP_OF(0.0);
    for (b = 0; b < 8 * 48000 / BLK; b++)
        pcl_biquad_process(&f, x, y, NULL, BLK);
    for (i = 0; i < BLK; i++)
        tail = fmax(tail, fabs(SMP_D(y[i])));
#if PCL_SAMPLE_TYPE == PCL_Q15
    PCL_CHECK(tail <= 2.0 / 32768.0);
#else
    PCL_CHECK(tail < 1.0e-6);
#endif
}

int main(void)
{
    PCL_RUN_TEST(test_init_and_setters);
    PCL_RUN_TEST(test_coefficients_match_cookbook);
    PCL_RUN_TEST(test_coefficients_at_other_rates);
    PCL_RUN_TEST(test_gain_landmarks);
    PCL_RUN_TEST(test_sine_response_matches_cookbook);
    PCL_RUN_TEST(test_precision_at_low_cutoffs);
#if PCL_SAMPLE_TYPE != PCL_FLOAT
    PCL_RUN_TEST(test_history_precision);
#endif
    PCL_RUN_TEST(test_block_size_independence);
    PCL_RUN_TEST(test_reset_and_aliasing);
    PCL_RUN_TEST(test_setters_glide_in_octaves);
    PCL_RUN_TEST(test_q_and_gain_glide);
    PCL_RUN_TEST(test_type_change_keeps_state_and_ramps);
    PCL_RUN_TEST(test_modulation_timing);
    PCL_RUN_TEST(test_cutoff_returns_and_coefficients_land);
    PCL_RUN_TEST(test_matches_time_domain_model);
    PCL_RUN_TEST(test_smooth_modulation_is_stable);
    PCL_RUN_TEST(test_violent_modulation_stays_finite);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    PCL_RUN_TEST(test_float_denormals_do_not_appear);
#endif
    PCL_RUN_TEST(test_decays_to_the_floor);
    return pcl_test_summary("biquad");
}
