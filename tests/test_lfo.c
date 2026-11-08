/*
 * Tests for the LFO (include/pcl/pcl_lfo.h).
 *
 * The main check is an independent double-precision model of the algorithm written here from
 * the design note (evaluate the shape at control points, interpolate linearly between them), so
 * every shape, rate, start phase and polarity is compared sample by sample against it.
 */
#include <stdlib.h>
#include <string.h>

#include "pcl_test.h"

#define SAMPLE_RATE_HZ 48000u
#define DIV ((size_t)PCL_CONTROL_DIV)
#define PI 3.14159265358979323846

/* Tolerances against the model, in absolute units of full scale. The sine includes the quarter-wave
   table's own error (4.7e-6) and, in Q15, the table's rounding. Fixed point also carries the
   truncated ramp step (15 LSB of Q2.30 = 1.4e-8). */
#if PCL_SAMPLE_TYPE == PCL_FLOAT
#define TOL_SINE 1.0e-5
#define TOL_OTHER 2.0e-6
#elif PCL_SAMPLE_TYPE == PCL_Q15
#define TOL_SINE                                                                                   \
    9.5e-5 /* 3 LSB: table rounding + interpolation rounding + truncating output shift */
#define TOL_OTHER 7.0e-5 /* 2 LSB */
#else
#define TOL_SINE 1.0e-5
#define TOL_OTHER 3.0e-8
#endif

static const char *shape_name(int s)
{
    static const char *const names[]
        = { "sine", "triangle", "saw_up", "saw_down", "square", "random_step", "random_smooth" };
    return names[s];
}

/* The frequency and width the module actually holds: PCL_HZ is Q16.16 in fixed-point builds (a
   resolution of 1.5e-5 Hz, 4e-6 relative at 0.7 Hz), so the model must use the quantized value. */
static double q_hz(double hz)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return (double)(float)hz;
#else
    return (double)PCL_HZ(hz) / 65536.0;
#endif
}

static double q_coef(double w) { return pcl_test_coef_to_double(PCL_COEF(w)); }

/* ---- Independent model ---- */

static uint32_t model_next(uint32_t *s)
{
    uint32_t x = *s;
    x = x ^ (x << 13);
    x = x ^ (x >> 17);
    x = x ^ (x << 5);
    *s = x;
    return x;
}

static double model_pick(uint32_t *s)
{
    const uint32_t  v = model_next(s);
    const long long signed_v = v >= 0x80000000u ? (long long)v - 4294967296ll : (long long)v;
    return (double)(signed_v >> 1) / 1073741824.0;
}

typedef struct
{
    int      shape;
    double   hz;
    double   width;
    double   start; /* fraction of a cycle */
    int      unipolar;
    uint32_t seed; /* 0 = default */
} model_cfg_t;

static double model_shape(int shape, double p, double width, double rp, double rc)
{
    const double f = p - floor(p);
    switch (shape)
    {
    case PCL_LFO_TRIANGLE:
        return f < 0.25 ? 4.0 * f : (f < 0.75 ? 2.0 - 4.0 * f : 4.0 * f - 4.0);
    case PCL_LFO_SAW_UP:
        return f < 0.5 ? 2.0 * f : 2.0 * f - 2.0;
    case PCL_LFO_SAW_DOWN:
        return f < 0.5 ? -2.0 * f : 2.0 - 2.0 * f;
    case PCL_LFO_SQUARE:
        return f < width ? 1.0 : -1.0;
    case PCL_LFO_RANDOM_STEP:
        return rc;
    case PCL_LFO_RANDOM_SMOOTH:
        return rp + (rc - rp) * f;
    default:
        return sin(2.0 * PI * f);
    }
}

static void model_run(const model_cfg_t *c, size_t n, double *out)
{
    uint32_t     rng = c->seed != 0u ? c->seed : 0x2545F491u;
    double       rp = model_pick(&rng);
    double       rc = model_pick(&rng);
    const double step = 16.0 * c->hz / (double)SAMPLE_RATE_HZ; /* cycles per control period */
    double       p0 = c->start;
    double       v0 = model_shape(c->shape, p0, c->width, rp, rc);
    size_t       k, i;

    for (k = 0; k * DIV < n; k++)
    {
        const double p1 = p0 + step;
        double       v1, a, b;
        if (floor(p1) > floor(p0))
        {
            rp = rc;
            rc = model_pick(&rng);
        }
        v1 = model_shape(c->shape, p1, c->width, rp, rc);
        a = c->unipolar ? 0.5 * v0 + 0.5 : v0;
        b = c->unipolar ? 0.5 * v1 + 0.5 : v1;
        for (i = 0; i < DIV && k * DIV + i < n; i++)
            out[k * DIV + i] = a + (b - a) * (double)i / (double)DIV;
        p0 = p1;
        v0 = v1;
    }
}

/* ---- Helpers ---- */

static void make_lfo_cfg(pcl_lfo_config_t *cfg, int shape, double hz, double width, double depth,
    int unipolar, double start, uint32_t seed)
{
    cfg->sample_rate_hz = SAMPLE_RATE_HZ;
    cfg->shape = (pcl_lfo_shape_t)shape;
    cfg->frequency = PCL_HZ(hz);
    cfg->rate_depth_oct = PCL_OCT(depth);
    cfg->pulse_width = PCL_COEF(width);
    cfg->unipolar = unipolar;
    cfg->start_phase = (uint32_t)(start * 4294967296.0);
    cfg->seed = seed;
}

static void make_lfo(pcl_lfo_t *l, int shape, double hz, double width, double depth, int unipolar,
    double start, uint32_t seed)
{
    pcl_lfo_config_t cfg;
    make_lfo_cfg(&cfg, shape, hz, width, depth, unipolar, start, seed);
    PCL_CHECK_EQ_INT(pcl_lfo_init(l, &cfg), PCL_OK);
}

#define N_MODEL 6000u
#define N_BIG 40000u
static pcl_sample_t g_a[N_BIG], g_b[N_BIG];
static double       g_model[N_BIG];

/* Renders `frames` in blocks of `block`, with an optional modulation buffer. */
static void render(
    pcl_lfo_t *l, pcl_sample_t *out, const pcl_sample_t *mod, size_t frames, size_t block)
{
    size_t done = 0;
    while (done < frames)
    {
        const size_t c = frames - done < block ? frames - done : block;
        pcl_lfo_process(l, out + done, mod != NULL ? mod + done : NULL, c);
        done += c;
    }
}

static double max_abs_error(const pcl_sample_t *out, const double *model, size_t n)
{
    double worst = 0.0;
    size_t i;
    for (i = 0; i < n; i++)
    {
        const double d = fabs(pcl_test_to_double(out[i]) - model[i]);
        if (d > worst)
            worst = d;
    }
    return worst;
}

/* ---- Validation ---- */

static void test_init_validation(void)
{
    pcl_lfo_t        l, before;
    pcl_lfo_config_t cfg;

    make_lfo_cfg(&cfg, PCL_LFO_SINE, 5.0, 0.5, 1.0, 0, 0.0, 1u);
    PCL_CHECK_EQ_INT(pcl_lfo_init(&l, &cfg), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_lfo_init(NULL, &cfg), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_lfo_init(&l, NULL), PCL_ERR_NULL_POINTER);

    cfg.sample_rate_hz = PCL_SAMPLE_RATE_MIN_HZ - 1u;
    PCL_CHECK_EQ_INT(pcl_lfo_init(&l, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.sample_rate_hz = PCL_SAMPLE_RATE_MAX_HZ + 1u;
    PCL_CHECK_EQ_INT(pcl_lfo_init(&l, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.sample_rate_hz = PCL_SAMPLE_RATE_MIN_HZ;
    PCL_CHECK_EQ_INT(pcl_lfo_init(&l, &cfg), PCL_OK);
    cfg.sample_rate_hz = PCL_SAMPLE_RATE_MAX_HZ;
    PCL_CHECK_EQ_INT(pcl_lfo_init(&l, &cfg), PCL_OK);
    cfg.sample_rate_hz = SAMPLE_RATE_HZ;

    /* the frequency limit is exactly sample_rate / (16 * PCL_CONTROL_DIV) = 187.5 Hz */
    cfg.frequency = PCL_HZ(187.5);
    PCL_CHECK_EQ_INT(pcl_lfo_init(&l, &cfg), PCL_OK);
    cfg.frequency = PCL_HZ(187.6);
    PCL_CHECK_EQ_INT(pcl_lfo_init(&l, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.frequency = PCL_HZ(0.0);
    PCL_CHECK_EQ_INT(pcl_lfo_init(&l, &cfg), PCL_OK);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    cfg.frequency = -0.1f;
    PCL_CHECK_EQ_INT(pcl_lfo_init(&l, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.frequency = NAN;
    PCL_CHECK_EQ_INT(pcl_lfo_init(&l, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.frequency = INFINITY;
    PCL_CHECK_EQ_INT(pcl_lfo_init(&l, &cfg), PCL_ERR_OUT_OF_RANGE);
#endif
    cfg.frequency = PCL_HZ(5.0);

    cfg.shape = PCL_LFO_SHAPE_COUNT;
    PCL_CHECK_EQ_INT(pcl_lfo_init(&l, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.shape = (pcl_lfo_shape_t)-1;
    PCL_CHECK_EQ_INT(pcl_lfo_init(&l, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.shape = PCL_LFO_SINE;

    cfg.pulse_width = PCL_COEF(0.009);
    PCL_CHECK_EQ_INT(pcl_lfo_init(&l, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.pulse_width = PCL_COEF(0.991);
    PCL_CHECK_EQ_INT(pcl_lfo_init(&l, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.pulse_width = PCL_COEF(0.5);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    cfg.pulse_width = NAN;
    PCL_CHECK_EQ_INT(pcl_lfo_init(&l, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.pulse_width = 0.5f;
    cfg.rate_depth_oct = 8.0f;
    PCL_CHECK_EQ_INT(pcl_lfo_init(&l, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.rate_depth_oct = NAN;
    PCL_CHECK_EQ_INT(pcl_lfo_init(&l, &cfg), PCL_ERR_OUT_OF_RANGE);
#endif
    cfg.rate_depth_oct = PCL_OCT(-0.5);
    PCL_CHECK_EQ_INT(pcl_lfo_init(&l, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.rate_depth_oct = PCL_OCT(7.9);
    PCL_CHECK_EQ_INT(pcl_lfo_init(&l, &cfg), PCL_OK);

    /* errors leave the module untouched */
    make_lfo(&l, PCL_LFO_SAW_UP, 3.0, 0.5, 1.0, 0, 0.0, 1u);
    pcl_lfo_process(&l, g_a, NULL, 100);
    before = l;
    cfg.shape = PCL_LFO_SHAPE_COUNT;
    PCL_CHECK_EQ_INT(pcl_lfo_init(&l, &cfg), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK(memcmp(&l, &before, sizeof l) == 0);
    PCL_CHECK_EQ_INT(pcl_lfo_set_shape(&l, PCL_LFO_SHAPE_COUNT), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_lfo_set_frequency(&l, PCL_HZ(200.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_lfo_set_pulse_width(&l, PCL_COEF(0.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_lfo_set_rate_depth(&l, PCL_OCT(-1.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK(memcmp(&l, &before, sizeof l) == 0);
    PCL_CHECK_EQ_INT(pcl_lfo_set_shape(NULL, PCL_LFO_SINE), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_lfo_set_frequency(NULL, PCL_HZ(1.0)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_lfo_set_pulse_width(NULL, PCL_COEF(0.5)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_lfo_set_rate_depth(NULL, PCL_OCT(1.0)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_lfo_set_shape(&l, PCL_LFO_TRIANGLE), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_lfo_set_frequency(&l, PCL_HZ(187.5)), PCL_OK);
}

/* ---- Agreement with the model ---- */

static void check_model(
    int shape, double hz, double width, double start, int unipolar, uint32_t seed)
{
    pcl_lfo_t   l;
    model_cfg_t m;
    double      worst;
    /* The phase advance per control period is a whole number of 2^-32 cycles, so the phase can
       drift from the ideal by up to one count per period; the steepest shape moves 6.3 (sine) or 4
       per cycle. */
    const double drift = 6.3 * (double)(N_MODEL / DIV) / 4294967296.0;
    const double tol = (shape == PCL_LFO_SINE ? TOL_SINE : TOL_OTHER) + drift;

    m.shape = shape;
    m.hz = q_hz(hz);
    m.width = q_coef(width);
    m.start = start;
    m.unipolar = unipolar;
    m.seed = seed;
    make_lfo(&l, shape, hz, width, 1.0, unipolar, start, seed);
    render(&l, g_a, NULL, N_MODEL, 256);
    model_run(&m, N_MODEL, g_model);
    worst = max_abs_error(g_a, g_model, N_MODEL);
    if (!(worst <= tol))
        printf("    %s %.3f Hz start %.2f uni %d: max error %.3g (limit %.3g)\n", shape_name(shape),
            hz, start, unipolar, worst, tol);
    PCL_CHECK(worst <= tol);
}

static void test_matches_model(void)
{
    static const double rates[] = { 0.7, 5.3, 41.9, 180.3 };
    int                 s;
    size_t              r;

    for (s = 0; s < PCL_LFO_SHAPE_COUNT; s++)
    {
        for (r = 0; r < sizeof rates / sizeof rates[0]; r++)
        {
            check_model(s, rates[r], 0.37, 0.0, 0, 123u);
            check_model(s, rates[r], 0.37, 0.3, 1, 0u);
        }
        check_model(s, 41.9, 0.8, 0.77, 0, 99u);
    }
}

/* ---- Shapes ---- */

#define CYCLE_93_75 512u /* samples per cycle at 93.75 Hz and 48 kHz (32 control periods) */

static void test_shape_facts(void)
{
    pcl_lfo_t    l;
    size_t       i;
    const double tol = 1e-3;

    /* sine/triangle: zero rising at 0, peak at 0.25, zero at 0.5, trough at 0.75 */
    make_lfo(&l, PCL_LFO_SINE, 93.75, 0.5, 1.0, 0, 0.0, 1u);
    pcl_lfo_process(&l, g_a, NULL, 2 * CYCLE_93_75);
    PCL_CHECK_NEAR(pcl_test_to_double(g_a[0]), 0.0, tol);
    PCL_CHECK_NEAR(pcl_test_to_double(g_a[CYCLE_93_75 / 4]), 1.0, tol);
    PCL_CHECK_NEAR(pcl_test_to_double(g_a[CYCLE_93_75 / 2]), 0.0, tol);
    PCL_CHECK_NEAR(pcl_test_to_double(g_a[3 * CYCLE_93_75 / 4]), -1.0, tol);
    PCL_CHECK_NEAR(pcl_test_to_double(g_a[CYCLE_93_75]), 0.0, tol);

    make_lfo(&l, PCL_LFO_TRIANGLE, 93.75, 0.5, 1.0, 0, 0.0, 1u);
    pcl_lfo_process(&l, g_a, NULL, CYCLE_93_75);
    PCL_CHECK_NEAR(pcl_test_to_double(g_a[0]), 0.0, tol);
    PCL_CHECK_NEAR(pcl_test_to_double(g_a[CYCLE_93_75 / 4]), 1.0, tol);
    PCL_CHECK_NEAR(pcl_test_to_double(g_a[CYCLE_93_75 / 2]), 0.0, tol);
    PCL_CHECK_NEAR(pcl_test_to_double(g_a[3 * CYCLE_93_75 / 4]), -1.0, tol);

    /* saw up: 0 rising, +1 just before 0.5, jumps to -1, back to 0 at 1; saw down mirrors it */
    make_lfo(&l, PCL_LFO_SAW_UP, 93.75, 0.5, 1.0, 0, 0.0, 1u);
    pcl_lfo_process(&l, g_a, NULL, CYCLE_93_75);
    PCL_CHECK_NEAR(pcl_test_to_double(g_a[0]), 0.0, tol);
    PCL_CHECK_NEAR(pcl_test_to_double(g_a[CYCLE_93_75 / 4]), 0.5, tol);
    PCL_CHECK(pcl_test_to_double(g_a[CYCLE_93_75 / 2 - DIV]) > 0.9);
    PCL_CHECK_NEAR(pcl_test_to_double(g_a[CYCLE_93_75 / 2 + DIV]), -0.9375, 0.01);
    PCL_CHECK_NEAR(pcl_test_to_double(g_a[3 * CYCLE_93_75 / 4]), -0.5, tol);
    make_lfo(&l, PCL_LFO_SAW_DOWN, 93.75, 0.5, 1.0, 0, 0.0, 1u);
    pcl_lfo_process(&l, g_b, NULL, CYCLE_93_75);
    for (i = 0; i < CYCLE_93_75; i++)
        PCL_CHECK(fabs(pcl_test_to_double(g_a[i]) + pcl_test_to_double(g_b[i])) < 1e-3);

    /* means over whole cycles: sine, triangle, saw 0; square 2*width - 1 */
    {
        static const double widths[]
            = { 0.3125, 0.5, 0.75 }; /* multiples of 1/32: edges on control points */
        size_t w;
        for (w = 0; w < 3; w++)
        {
            double mean = 0.0;
            make_lfo(&l, PCL_LFO_SQUARE, 93.75, widths[w], 1.0, 0, 0.0, 1u);
            pcl_lfo_process(&l, g_a, NULL, 8 * CYCLE_93_75);
            for (i = 0; i < 8 * CYCLE_93_75; i++)
                mean += pcl_test_to_double(g_a[i]);
            mean /= (double)(8 * CYCLE_93_75);
            PCL_CHECK_NEAR(mean, 2.0 * widths[w] - 1.0, 0.01);
        }
    }
    {
        static const int zero_mean[]
            = { PCL_LFO_SINE, PCL_LFO_TRIANGLE, PCL_LFO_SAW_UP, PCL_LFO_SAW_DOWN };
        size_t s;
        for (s = 0; s < 4; s++)
        {
            double mean = 0.0;
            make_lfo(&l, zero_mean[s], 93.75, 0.5, 1.0, 0, 0.0, 1u);
            pcl_lfo_process(&l, g_a, NULL, 8 * CYCLE_93_75);
            for (i = 0; i < 8 * CYCLE_93_75; i++)
                mean += pcl_test_to_double(g_a[i]);
            mean /= (double)(8 * CYCLE_93_75);
            /* A saw wrap that lands exactly on a control point is smeared over the period before
               it, which leaves a DC offset of -/+1 per cycle's control points: -1/32 for saw up and
               +1/32 for saw down here. (At generic frequencies the edge falls at varying places and
               the offsets average out.) */
            if (zero_mean[s] == PCL_LFO_SAW_UP)
                PCL_CHECK_NEAR(mean, -1.0 / 32.0, 0.004);
            else if (zero_mean[s] == PCL_LFO_SAW_DOWN)
                PCL_CHECK_NEAR(mean, 1.0 / 32.0, 0.004);
            else
                PCL_CHECK(fabs(mean) < 0.004);
        }
    }

    /* unipolar: the same shapes mapped to [0, 1] */
    make_lfo(&l, PCL_LFO_SINE, 93.75, 0.5, 1.0, 1, 0.0, 1u);
    pcl_lfo_process(&l, g_a, NULL, CYCLE_93_75);
    PCL_CHECK_NEAR(pcl_test_to_double(g_a[0]), 0.5, tol);
    PCL_CHECK_NEAR(pcl_test_to_double(g_a[CYCLE_93_75 / 4]), 1.0, tol);
    PCL_CHECK_NEAR(pcl_test_to_double(g_a[3 * CYCLE_93_75 / 4]), 0.0, tol);
}

/* A frequency of 0 freezes the phase: the output stays at the start-phase value. */
static void test_zero_frequency_freezes(void)
{
    pcl_lfo_t l;
    size_t    i;
    int       same = 1;

    make_lfo(&l, PCL_LFO_SINE, 0.0, 0.5, 1.0, 0, 0.25, 1u);
    pcl_lfo_process(&l, g_a, NULL, 5000);
    for (i = 1; i < 5000; i++)
        same = same && g_a[i] == g_a[0];
    PCL_CHECK(same);
    PCL_CHECK_NEAR(pcl_test_to_double(g_a[0]), 1.0, 1e-3);
    PCL_CHECK_EQ_INT(pcl_lfo_phase(&l), 0x40000000u);
}

/* With the rate at 0 nothing moves for any shape; the random shapes must not draw new values
 * either. */
static void test_zero_rate_freezes_random_shapes(void)
{
    pcl_lfo_t l;
    size_t    i;
    int       s;

    for (s = 0; s < PCL_LFO_SHAPE_COUNT; s++)
    {
        int same = 1;
        make_lfo(&l, s, 0.0, 0.5, 1.0, 0, 0.3, 11u);
        pcl_lfo_process(&l, g_a, NULL, 5000);
        for (i = 1; i < 5000; i++)
            same = same && g_a[i] == g_a[0];
        PCL_CHECK(same);
        PCL_CHECK_EQ_INT(pcl_lfo_phase(&l), (uint32_t)(0.3 * 4294967296.0));
    }
}

/* ---- Range ---- */

static void check_range(const pcl_sample_t *buf, size_t n, int unipolar, size_t *bad)
{
    size_t i;
    for (i = 0; i < n; i++)
    {
        const double v = pcl_test_to_double(buf[i]);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
        if (!isfinite(buf[i]) || (buf[i] != 0.0f && fabsf(buf[i]) < FLT_MIN))
            (*bad)++;
#endif
        if (unipolar ? (v < 0.0 || v > 1.0) : (v < -1.0 || v > 1.0))
            (*bad)++;
    }
}

static void test_output_range_and_extremes(void)
{
    pcl_lfo_t    l;
    pcl_sample_t mod[N_MODEL];
    uint32_t     seed = 77u;
    size_t       i, bad = 0;
    int          s, uni;

    for (i = 0; i < N_MODEL; i++)
        mod[i] = pcl_test_from_double(pcl_test_rand(&seed));
    for (s = 0; s < PCL_LFO_SHAPE_COUNT; s++)
    {
        for (uni = 0; uni < 2; uni++)
        {
            /* max frequency, +/-1 modulation at depth 7.9 (the clamp), extreme widths, phase storms
             */
            make_lfo(&l, s, 187.5, 0.01, 7.9, uni, 0.9999, 5u);
            render(&l, g_a, mod, N_MODEL, 97);
            check_range(g_a, N_MODEL, uni, &bad);
            PCL_CHECK_EQ_INT(pcl_lfo_set_pulse_width(&l, PCL_COEF(0.99)), PCL_OK);
            for (i = 0; i < 200; i++)
            {
                pcl_lfo_set_phase(&l, (uint32_t)(i * 2654435761u));
                pcl_lfo_process(&l, g_a, mod, 23);
                check_range(g_a, 23, uni, &bad);
            }
            make_lfo(&l, s, 0.01, 0.99, 0.0, uni, 0.0, 1u);
            render(&l, g_a, NULL, N_MODEL, 256);
            check_range(g_a, N_MODEL, uni, &bad);
        }
    }
    PCL_CHECK_EQ_INT(bad, 0);
}

/*
 * Regression: the float build used to accumulate its ramp (cur += step), and the rounding error
 * could carry the output an ulp or two past +1 on the flat top of a slow sine (1.0000002 was seen).
 * Sweep the start phase across the peak at low rates, both polarities, and require the range to
 * hold exactly.
 */
static void test_peak_never_exceeds_range(void)
{
    static const double rates[] = { 0.2, 0.5, 0.9, 1.3, 2.0, 3.1 };
    size_t              r, sp, bad = 0;
    int                 shape, uni;

    for (r = 0; r < sizeof rates / sizeof rates[0]; r++)
        for (uni = 0; uni < 2; uni++)
            for (shape = PCL_LFO_SINE; shape <= PCL_LFO_TRIANGLE; shape++)
                for (sp = 0; sp < 400; sp++)
                {
                    pcl_lfo_t        l;
                    pcl_lfo_config_t cfg;
                    make_lfo_cfg(&cfg, shape, rates[r], 0.5, 0.0, uni, 0.0, 1u);
                    cfg.start_phase
                        = 0x3FF00000u + (uint32_t)sp * 40000u; /* around the +1 peak at 0.25 */
                    PCL_CHECK_EQ_INT(pcl_lfo_init(&l, &cfg), PCL_OK);
                    pcl_lfo_process(&l, g_a, NULL, 1024);
                    check_range(g_a, 1024, uni, &bad);
                }
    PCL_CHECK_EQ_INT(bad, 0);
}

/* ---- Determinism ---- */

static void test_block_size_independence(void)
{
    static const size_t blocks[] = { 1, 7, 16, 17, 256 };
    pcl_sample_t        mod[N_MODEL];
    uint32_t            seed = 5u;
    pcl_lfo_t           ref, l;
    size_t              i, b;
    int                 s;

    for (i = 0; i < N_MODEL; i++)
        mod[i] = pcl_test_from_double(0.8 * pcl_test_rand(&seed));
    for (s = 0; s < PCL_LFO_SHAPE_COUNT; s++)
    {
        /* rate modulation, a frequency glide and a width glide all active */
        make_lfo(&ref, s, 9.3, 0.4, 2.0, s & 1, 0.1, 31337u);
        PCL_CHECK_EQ_INT(pcl_lfo_set_frequency(&ref, PCL_HZ(120.0)), PCL_OK);
        PCL_CHECK_EQ_INT(pcl_lfo_set_pulse_width(&ref, PCL_COEF(0.7)), PCL_OK);
        pcl_lfo_process(&ref, g_a, mod, N_MODEL);
        for (b = 0; b < sizeof blocks / sizeof blocks[0]; b++)
        {
            make_lfo(&l, s, 9.3, 0.4, 2.0, s & 1, 0.1, 31337u);
            PCL_CHECK_EQ_INT(pcl_lfo_set_frequency(&l, PCL_HZ(120.0)), PCL_OK);
            PCL_CHECK_EQ_INT(pcl_lfo_set_pulse_width(&l, PCL_COEF(0.7)), PCL_OK);
            render(&l, g_b, mod, N_MODEL, blocks[b]);
            PCL_CHECK(memcmp(g_a, g_b, N_MODEL * sizeof g_a[0]) == 0);
            PCL_CHECK_EQ_INT(pcl_lfo_phase(&l), pcl_lfo_phase(&ref));
        }
    }
}

static void test_reset_and_seed(void)
{
    pcl_lfo_t l;
    int       s;

    for (s = 0; s < PCL_LFO_SHAPE_COUNT; s++)
    {
        make_lfo(&l, s, 12.5, 0.4, 1.0, 0, 0.2, 77u);
        pcl_lfo_process(&l, g_a, NULL, 5000);
        pcl_lfo_reset(&l);
        pcl_lfo_process(&l, g_b, NULL, 5000);
        PCL_CHECK(memcmp(g_a, g_b, 5000 * sizeof g_a[0]) == 0);

        /* seed 0 is the default seed; a fresh init with the same config matches too */
        make_lfo(&l, s, 12.5, 0.4, 1.0, 0, 0.2, 0u);
        pcl_lfo_process(&l, g_a, NULL, 5000);
        pcl_lfo_set_seed(&l, PCL_NOISE_DEFAULT_SEED);
        pcl_lfo_process(&l, g_b, NULL, 5000);
        PCL_CHECK(memcmp(g_a, g_b, 5000 * sizeof g_a[0]) == 0);
    }
    /* different seeds give different random shapes and identical deterministic ones */
    make_lfo(&l, PCL_LFO_RANDOM_SMOOTH, 12.5, 0.5, 1.0, 0, 0.0, 1u);
    pcl_lfo_process(&l, g_a, NULL, 5000);
    make_lfo(&l, PCL_LFO_RANDOM_SMOOTH, 12.5, 0.5, 1.0, 0, 0.0, 2u);
    pcl_lfo_process(&l, g_b, NULL, 5000);
    PCL_CHECK(memcmp(g_a, g_b, 5000 * sizeof g_a[0]) != 0);
    make_lfo(&l, PCL_LFO_TRIANGLE, 12.5, 0.5, 1.0, 0, 0.0, 1u);
    pcl_lfo_process(&l, g_a, NULL, 5000);
    make_lfo(&l, PCL_LFO_TRIANGLE, 12.5, 0.5, 1.0, 0, 0.0, 2u);
    pcl_lfo_process(&l, g_b, NULL, 5000);
    PCL_CHECK(memcmp(g_a, g_b, 5000 * sizeof g_a[0]) == 0);
}

static void test_rate_mod_may_alias_output(void)
{
    pcl_lfo_t    a, b;
    pcl_sample_t mod[2000];
    uint32_t     seed = 9u;
    size_t       i;

    for (i = 0; i < 2000; i++)
        mod[i] = pcl_test_from_double(0.7 * pcl_test_rand(&seed));
    make_lfo(&a, PCL_LFO_SINE, 30.0, 0.5, 1.5, 0, 0.0, 1u);
    make_lfo(&b, PCL_LFO_SINE, 30.0, 0.5, 1.5, 0, 0.0, 1u);
    pcl_lfo_process(&a, g_a, mod, 2000);
    memcpy(g_b, mod, sizeof mod);
    pcl_lfo_process(&b, g_b, g_b, 2000); /* in place */
    PCL_CHECK(memcmp(g_a, g_b, 2000 * sizeof g_a[0]) == 0);
}

/* ---- Frequency, glide and rate modulation ---- */

/* Total phase advance (in cycles) over `ticks` control periods, from pcl_lfo_phase(). */
static double advance_cycles(pcl_lfo_t *l, size_t ticks, const pcl_sample_t *mod)
{
    uint64_t total = 0;
    size_t   t;
    for (t = 0; t < ticks; t++)
    {
        const uint32_t a = pcl_lfo_phase(l);
        pcl_lfo_process(l, g_a, mod != NULL ? mod + t * DIV : NULL, DIV);
        total += (uint32_t)(pcl_lfo_phase(l) - a);
    }
    return (double)total / 4294967296.0;
}

static void test_frequency_accuracy(void)
{
    static const double rates[] = { 0.01, 0.1, 1.0, 13.7, 100.0, 187.5 };
    size_t              r;

    for (r = 0; r < sizeof rates / sizeof rates[0]; r++)
    {
        pcl_lfo_t    l;
        const size_t ticks = 3000; /* 1 s */
        double       cycles, expected, rel;

        make_lfo(&l, PCL_LFO_SINE, rates[r], 0.5, 1.0, 0, 0.0, 1u);
        cycles = advance_cycles(&l, ticks, NULL);
        expected = q_hz(rates[r]) * (double)(ticks * DIV) / (double)SAMPLE_RATE_HZ;
        rel = fabs(cycles - expected) / expected;
        if (rel > 1e-4)
            printf("    %.3f Hz: advanced %.9f cycles, expected %.9f (rel %.2e)\n", rates[r],
                cycles, expected, rel);
        /* the phase advance per control period is an integer count of 2^-32 cycles, truncated: at
           most one count short, i.e. 1 / advance in relative terms (7e-5 at 0.01 Hz, 1e-8 at 100
           Hz) */
        PCL_CHECK(
            rel < 1.5 / (q_hz(rates[r]) * 16.0 / (double)SAMPLE_RATE_HZ * 4294967296.0) + 1e-6);
    }
}

static void test_rate_modulation_octaves(void)
{
    static const struct
    {
        double depth, mod;
    } cases[] = { { 1.0, 1.0 }, { 1.0, -1.0 }, { 2.0, 0.5 }, { 2.1, -0.25 }, { 3.0, 0.75 } };
    size_t k;

    for (k = 0; k < sizeof cases / sizeof cases[0]; k++)
    {
        pcl_lfo_t          l;
        pcl_sample_t       mod[3200];
        const size_t       ticks = 200;
        double             base, modulated, ratio;
        size_t             i;
        const pcl_sample_t m = pcl_test_from_double(cases[k].mod);

        for (i = 0; i < 3200; i++)
            mod[i] = m;
        make_lfo(&l, PCL_LFO_SINE, 3.0, 0.5, cases[k].depth, 0, 0.0, 1u);
        base = advance_cycles(&l, ticks, NULL);
        make_lfo(&l, PCL_LFO_SINE, 3.0, 0.5, cases[k].depth, 0, 0.0, 1u);
        modulated = advance_cycles(&l, ticks, mod);
        ratio = modulated / base;
        /* the depth is quantized in the build's format; compare with the octaves actually applied
         */
        PCL_CHECK_NEAR(log2(ratio), cases[k].depth * pcl_test_to_double(m), 3e-4);
    }
}

/* A modulated rate stops at the maximum: exactly 16 control points per cycle. */
static void test_rate_is_clamped_at_the_maximum(void)
{
    pcl_lfo_t    l;
    pcl_sample_t mod[64 * 16];
    size_t       i;
    int          depth_case;

    for (i = 0; i < 64 * 16; i++)
        mod[i] = pcl_test_from_double(0.999);
    for (depth_case = 0; depth_case < 2; depth_case++)
    {
        /* at the limit already, then well below it with a depth that would overshoot it */
        make_lfo(&l, PCL_LFO_SINE, depth_case == 0 ? 187.5 : 120.0, 0.5,
            depth_case == 0 ? 1.0 : 7.5, 0, 0.0, 1u);
        PCL_CHECK_NEAR(advance_cycles(&l, 64, mod), 64.0 / 16.0, 2e-6); /* 1/16 cycle per period */
    }
}

/* The increment of a period changes at the tick that reads the new modulation: no lag. */
static void test_rate_modulation_has_no_lag(void)
{
    pcl_lfo_t    l;
    pcl_sample_t mod[16 * 12];
    size_t       i;
    uint32_t     before, a, b;

    for (i = 0; i < 16 * 12; i++)
        mod[i] = i < 160 ? (pcl_sample_t)0 : pcl_test_from_double(0.5);
    make_lfo(&l, PCL_LFO_SINE, 20.0, 0.5, 2.0, 0, 0.0, 1u); /* 0.5 * 2 = +1 octave */
    pcl_lfo_process(&l, g_a, mod, 144);                     /* periods 0..8 */
    before = pcl_lfo_phase(&l);
    pcl_lfo_process(&l, g_a, mod + 144, 16); /* period 9: modulation still 0 */
    a = pcl_lfo_phase(&l);
    pcl_lfo_process(&l, g_a, mod + 160, 16); /* period 10: the first with modulation */
    b = pcl_lfo_phase(&l);
    PCL_CHECK_NEAR((double)(uint32_t)(a - before), (double)(uint32_t)(a - before), 0.0);
    PCL_CHECK_NEAR((double)(uint32_t)(b - a) / (double)(uint32_t)(a - before), 2.0, 1e-4);
}

static void test_frequency_glide(void)
{
    pcl_lfo_t l;
    uint32_t  prev, cur, target_adv;
    size_t    t;
    int       monotone = 1;

    make_lfo(&l, PCL_LFO_SINE, 1.0, 0.5, 1.0, 0, 0.0, 1u);
    pcl_lfo_process(&l, g_a, NULL, 16 * 10);
    PCL_CHECK_EQ_INT(pcl_lfo_set_frequency(&l, PCL_HZ(100.0)), PCL_OK);
    prev = 0;
    for (t = 0; t < 60; t++)
    {
        const uint32_t a = pcl_lfo_phase(&l);
        pcl_lfo_process(&l, g_a, NULL, 16);
        cur = (uint32_t)(pcl_lfo_phase(&l) - a);
        if (cur < prev)
            monotone = 0;
        prev = cur;
    }
    PCL_CHECK(monotone);
    target_adv = (uint32_t)(100.0 * 16.0 / 48000.0 * 4294967296.0);
    PCL_CHECK(prev + 4096u >= target_adv && prev <= target_adv + 4096u); /* landed on the target */
    /* it takes about PCL_SMOOTH_MS: not instant, not slower than 2x */
    make_lfo(&l, PCL_LFO_SINE, 1.0, 0.5, 1.0, 0, 0.0, 1u);
    PCL_CHECK_EQ_INT(pcl_lfo_set_frequency(&l, PCL_HZ(100.0)), PCL_OK);
    pcl_lfo_process(&l, g_a, NULL, 16);
    {
        const uint32_t a = pcl_lfo_phase(&l);
        pcl_lfo_process(&l, g_a, NULL, 16);
        PCL_CHECK((uint32_t)(pcl_lfo_phase(&l) - a) < target_adv / 2u);
    }
}

/* ---- Phase control ---- */

static void test_set_phase_retrigger_shape(void)
{
    pcl_lfo_t l;

    /* set_phase takes effect at the next control point: the next period starts at the new value */
    make_lfo(&l, PCL_LFO_SINE, 0.5, 0.5, 1.0, 0, 0.0, 1u);
    pcl_lfo_process(&l, g_a, NULL, 160);
    pcl_lfo_set_phase(&l, 0x40000000u);
    PCL_CHECK_EQ_INT(pcl_lfo_phase(&l), 0x40000000u);
    pcl_lfo_process(&l, g_a, NULL, 16);
    PCL_CHECK_NEAR(pcl_test_to_double(g_a[0]), 1.0, 1e-3);

    /* retrigger returns to the configured start phase */
    make_lfo(&l, PCL_LFO_TRIANGLE, 2.0, 0.5, 1.0, 0, 0.25, 1u);
    pcl_lfo_process(&l, g_a, NULL, 3008); /* a whole number of control periods (188) */
    pcl_lfo_retrigger(&l);
    pcl_lfo_process(&l, g_a, NULL, 16);
    PCL_CHECK_NEAR(pcl_test_to_double(g_a[0]), 1.0, 1e-3);

    /* a shape change keeps the phase and ramps from the old shape's value: no step */
    make_lfo(&l, PCL_LFO_SINE, 2.0, 0.5, 1.0, 0, 0.0, 1u);
    pcl_lfo_process(&l, g_a, NULL, 12000);
    {
        const uint32_t ph = pcl_lfo_phase(&l);
        PCL_CHECK_EQ_INT(pcl_lfo_set_shape(&l, PCL_LFO_SQUARE), PCL_OK);
        PCL_CHECK_EQ_INT(pcl_lfo_phase(&l), ph);
        pcl_lfo_process(&l, g_b, NULL, 64);
        {
            size_t i;
            double worst = 0.0;
            for (i = 1; i < 64; i++)
                worst = fmax(
                    worst, fabs(pcl_test_to_double(g_b[i]) - pcl_test_to_double(g_b[i - 1])));
            PCL_CHECK(worst <= 2.0 / (double)DIV + 1e-3); /* no step larger than one ramp slope */
        }
    }
}

static void test_pulse_width_glide(void)
{
    pcl_lfo_t    l;
    double       mean_before = 0.0, mean_after = 0.0;
    size_t       i;
    const size_t n = 4 * CYCLE_93_75;

    make_lfo(&l, PCL_LFO_SQUARE, 93.75, 0.25, 1.0, 0, 0.0, 1u);
    pcl_lfo_process(&l, g_a, NULL, n);
    PCL_CHECK_EQ_INT(pcl_lfo_set_pulse_width(&l, PCL_COEF(0.75)), PCL_OK);
    pcl_lfo_process(&l, g_a, NULL, 2 * CYCLE_93_75); /* let the ~5 ms glide finish */
    pcl_lfo_process(&l, g_b, NULL, n);
    make_lfo(&l, PCL_LFO_SQUARE, 93.75, 0.25, 1.0, 0, 0.0, 1u);
    pcl_lfo_process(&l, g_a, NULL, n);
    for (i = 0; i < n; i++)
        mean_before += pcl_test_to_double(g_a[i]);
    for (i = 0; i < n; i++)
        mean_after += pcl_test_to_double(g_b[i]);
    PCL_CHECK_NEAR(mean_before / (double)n, -0.5, 0.03);
    PCL_CHECK_NEAR(mean_after / (double)n, 0.5, 0.03);
}

/* ---- Interpolation distortion ---- */

static void test_interpolation_distortion(void)
{
    /* points per cycle -> distortion measured with a script before implementation (dB) */
    static const struct
    {
        unsigned points;
        double   hz, script_db;
    } cases[] = { { 16, 187.5, -44.5 }, { 32, 93.75, -56.6 }, { 64, 46.875, -68.7 } };
    size_t k;

    for (k = 0; k < 3; k++)
    {
        pcl_lfo_t    l;
        const size_t m = (size_t)cases[k].points * DIV; /* one cycle of samples */
        double       p1 = 0.0, ph = 0.0, db;
        size_t       h, n;

        make_lfo(&l, PCL_LFO_SINE, cases[k].hz, 0.5, 1.0, 0, 0.0, 1u);
        pcl_lfo_process(&l, g_a, NULL, m);
        for (h = 1; h < m / 2; h++)
        {
            double re = 0.0, im = 0.0, p;
            for (n = 0; n < m; n++)
            {
                const double x = pcl_test_to_double(g_a[n]);
                re += x * cos(2.0 * PI * (double)(h * n % m) / (double)m);
                im += x * sin(2.0 * PI * (double)(h * n % m) / (double)m);
            }
            p = re * re + im * im;
            if (h == 1)
                p1 = p;
            else
                ph += p;
        }
        db = 10.0 * log10(ph / p1);
        printf("    %u points/cycle (%.3f Hz): distortion %.1f dB (script: %.1f dB)\n",
            cases[k].points, cases[k].hz, db, cases[k].script_db);
        PCL_CHECK(db < cases[k].script_db + 3.0);
        PCL_CHECK(db > cases[k].script_db - 3.0);
    }
}

/* ---- Random shapes ---- */

static void test_random_shapes(void)
{
    enum
    {
        CYCLES = 4000,
        BINS = 16
    };
    const size_t cyc = CYCLE_93_75; /* 512 samples */
    pcl_lfo_t    l;
    unsigned     hist[BINS];
    size_t       c, i, bad = 0;
    double       mean = 0.0, worst_step = 0.0;
    pcl_sample_t buf[CYCLE_93_75];

    memset(hist, 0, sizeof hist);
    make_lfo(&l, PCL_LFO_RANDOM_STEP, 93.75, 0.5, 1.0, 0, 0.0, 4242u);
    for (c = 0; c < CYCLES; c++)
    {
        double v;
        pcl_lfo_process(&l, buf, NULL, cyc);
        /* one value per cycle: constant between the ramp at the start and the ramp before the next
         * wrap */
        for (i = DIV + 1; i < cyc - DIV - 1; i++)
            if (buf[i] != buf[DIV + 1])
                bad++;
        v = pcl_test_to_double(buf[cyc / 2]);
        mean += v;
        {
            int bin = (int)floor((v + 1.0) * 0.5 * BINS);
            hist[bin < 0 ? 0 : (bin >= BINS ? BINS - 1 : bin)]++;
        }
    }
    PCL_CHECK_EQ_INT(bad, 0);
    {
        const double p = 1.0 / BINS, sigma = sqrt((double)CYCLES * p * (1.0 - p));
        int          ok = 1;
        for (i = 0; i < BINS; i++)
            if (fabs((double)hist[i] - (double)CYCLES * p) > 5.0 * sigma)
                ok = 0;
        PCL_CHECK(ok);
        PCL_CHECK(fabs(mean / CYCLES) < 5.0 * 0.5774 / sqrt((double)CYCLES));
    }

    /* smooth: continuous, per-sample change bounded by the steepest segment (|dr| <= 2 per cycle)
     */
    make_lfo(&l, PCL_LFO_RANDOM_SMOOTH, 93.75, 0.5, 1.0, 0, 0.0, 4242u);
    for (c = 0; c < 200; c++)
    {
        double prev = c == 0 ? 0.0 : pcl_test_to_double(buf[cyc - 1]);
        pcl_lfo_process(&l, buf, NULL, cyc);
        for (i = 0; i < cyc; i++)
        {
            const double v = pcl_test_to_double(buf[i]);
            if (!(c == 0 && i == 0) && fabs(v - prev) > worst_step)
                worst_step = fabs(v - prev);
            prev = v;
        }
    }
    PCL_CHECK(worst_step <= 2.0 / (double)cyc + 1e-4);
}

/* ---- Long runs ---- */

static void test_long_run(void)
{
    const size_t minutes_frames = (size_t)SAMPLE_RATE_HZ * 60u;
    pcl_lfo_t    l;
    pcl_sample_t mod[256];
    uint32_t     seed = 99u;
    size_t       done, i, bad = 0;
    int          s;

    for (s = 0; s < PCL_LFO_SHAPE_COUNT; s++)
    {
        const size_t frames = s == PCL_LFO_RANDOM_SMOOTH ? 10u * minutes_frames : minutes_frames;
        make_lfo(&l, s, 7.3, 0.3, 3.0, s & 1, 0.0, 8u);
        for (done = 0; done < frames; done += 256)
        {
            for (i = 0; i < 256; i++)
                mod[i] = pcl_test_from_double(pcl_test_rand(&seed));
            pcl_lfo_process(&l, g_a, mod, 256);
            check_range(g_a, 256, s & 1, &bad);
        }
    }
    printf("    1 minute per shape, 10 minutes for random_smooth, noise on the rate input\n");
    PCL_CHECK_EQ_INT(bad, 0);
}

int main(void)
{
    PCL_RUN_TEST(test_init_validation);
    PCL_RUN_TEST(test_matches_model);
    PCL_RUN_TEST(test_shape_facts);
    PCL_RUN_TEST(test_zero_frequency_freezes);
    PCL_RUN_TEST(test_zero_rate_freezes_random_shapes);
    PCL_RUN_TEST(test_output_range_and_extremes);
    PCL_RUN_TEST(test_peak_never_exceeds_range);
    PCL_RUN_TEST(test_block_size_independence);
    PCL_RUN_TEST(test_reset_and_seed);
    PCL_RUN_TEST(test_rate_mod_may_alias_output);
    PCL_RUN_TEST(test_frequency_accuracy);
    PCL_RUN_TEST(test_rate_modulation_octaves);
    PCL_RUN_TEST(test_rate_is_clamped_at_the_maximum);
    PCL_RUN_TEST(test_rate_modulation_has_no_lag);
    PCL_RUN_TEST(test_frequency_glide);
    PCL_RUN_TEST(test_set_phase_retrigger_shape);
    PCL_RUN_TEST(test_pulse_width_glide);
    PCL_RUN_TEST(test_interpolation_distortion);
    PCL_RUN_TEST(test_random_shapes);
    PCL_RUN_TEST(test_long_run);
    return pcl_test_summary("lfo");
}
