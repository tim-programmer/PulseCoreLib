/* Tests for the band-limited oscillator. */
#include <string.h>

#include "pcl_test.h"

#define SAMPLE_RATE_HZ 48000u
#define FFT_SIZE 16384u
#define MAX_FRAMES 96000u
#define PI 3.14159265358979323846

static pcl_sample_t g_out[MAX_FRAMES];
static pcl_sample_t g_sync[MAX_FRAMES];
static pcl_sample_t g_mod_a[MAX_FRAMES];
static pcl_sample_t g_mod_b[MAX_FRAMES];

static const pcl_osc_waveform_t g_waves[]
    = { PCL_OSC_SINE, PCL_OSC_TRIANGLE, PCL_OSC_SAW, PCL_OSC_PULSE };
static const char *const g_wave_names[] = { "sine", "triangle", "saw", "pulse" };

static void make_osc(pcl_osc_t *o, pcl_osc_waveform_t wf, double hz, double depth_oct, double width,
    double pwm_depth)
{
    pcl_osc_config_t cfg;
    cfg.sample_rate_hz = SAMPLE_RATE_HZ;
    cfg.waveform = wf;
    cfg.frequency = PCL_HZ(hz);
    cfg.pitch_depth_oct = PCL_OCT(depth_oct);
    cfg.pulse_width = PCL_COEF(width);
    cfg.pwm_depth = PCL_COEF(pwm_depth);
    PCL_CHECK_EQ_INT(pcl_osc_init(o, &cfg), PCL_OK);
}

static void render(pcl_osc_t *o, pcl_sample_t *out, size_t n)
{
    pcl_osc_process(o, out, NULL, NULL, NULL, NULL, n);
}

static void fill_noise(pcl_sample_t *buf, size_t n, uint32_t seed, double amplitude)
{
    size_t i;
    for (i = 0; i < n; i++)
        buf[i] = pcl_test_from_double(pcl_test_rand(&seed) * amplitude);
}

/* ---- validation ---- */

static void test_init_validation(void)
{
    pcl_osc_t        o;
    pcl_osc_config_t cfg;

    cfg.sample_rate_hz = SAMPLE_RATE_HZ;
    cfg.waveform = PCL_OSC_SAW;
    cfg.frequency = PCL_HZ(440.0);
    cfg.pitch_depth_oct = PCL_OCT(1.0);
    cfg.pulse_width = PCL_COEF(0.5);
    cfg.pwm_depth = PCL_COEF(0.25);

    PCL_CHECK_EQ_INT(pcl_osc_init(&o, &cfg), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_osc_init(NULL, &cfg), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_osc_init(&o, NULL), PCL_ERR_NULL_POINTER);

    cfg.sample_rate_hz = PCL_SAMPLE_RATE_MIN_HZ - 1u;
    PCL_CHECK_EQ_INT(pcl_osc_init(&o, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.sample_rate_hz = PCL_SAMPLE_RATE_MAX_HZ + 1u;
    PCL_CHECK_EQ_INT(pcl_osc_init(&o, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.sample_rate_hz = PCL_SAMPLE_RATE_MIN_HZ;
    PCL_CHECK_EQ_INT(pcl_osc_init(&o, &cfg), PCL_OK);
    cfg.sample_rate_hz = PCL_SAMPLE_RATE_MAX_HZ;
    cfg.frequency = PCL_HZ(1000.0);
    PCL_CHECK_EQ_INT(pcl_osc_init(&o, &cfg), PCL_OK);
    cfg.sample_rate_hz = SAMPLE_RATE_HZ;
    cfg.frequency = PCL_HZ(440.0);

    cfg.waveform = PCL_OSC_WAVEFORM_COUNT;
    PCL_CHECK_EQ_INT(pcl_osc_init(&o, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.waveform = (pcl_osc_waveform_t)-1;
    PCL_CHECK_EQ_INT(pcl_osc_init(&o, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.waveform = PCL_OSC_PULSE;

    cfg.frequency = PCL_HZ(0.0);
    PCL_CHECK_EQ_INT(pcl_osc_init(&o, &cfg), PCL_OK);
    cfg.frequency = PCL_HZ(21600.0); /* exactly 0.45 * 48000 */
    PCL_CHECK_EQ_INT(pcl_osc_init(&o, &cfg), PCL_OK);
    cfg.frequency = PCL_HZ(21601.0);
    PCL_CHECK_EQ_INT(pcl_osc_init(&o, &cfg), PCL_ERR_OUT_OF_RANGE);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    cfg.frequency = -1.0f;
    PCL_CHECK_EQ_INT(pcl_osc_init(&o, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.frequency = NAN;
    PCL_CHECK_EQ_INT(pcl_osc_init(&o, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.frequency = INFINITY;
    PCL_CHECK_EQ_INT(pcl_osc_init(&o, &cfg), PCL_ERR_OUT_OF_RANGE);
#endif
    cfg.frequency = PCL_HZ(440.0);

    cfg.pitch_depth_oct = PCL_OCT(-0.5);
    PCL_CHECK_EQ_INT(pcl_osc_init(&o, &cfg), PCL_ERR_OUT_OF_RANGE);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    cfg.pitch_depth_oct = 8.0f;
    PCL_CHECK_EQ_INT(pcl_osc_init(&o, &cfg), PCL_ERR_OUT_OF_RANGE);
#endif
    cfg.pitch_depth_oct = PCL_OCT(7.9);
    PCL_CHECK_EQ_INT(pcl_osc_init(&o, &cfg), PCL_OK);
    cfg.pitch_depth_oct = PCL_OCT(1.0);

    cfg.pulse_width = PCL_COEF(0.005);
    PCL_CHECK_EQ_INT(pcl_osc_init(&o, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.pulse_width = PCL_COEF(0.995);
    PCL_CHECK_EQ_INT(pcl_osc_init(&o, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.pulse_width = PCL_COEF(0.011);
    PCL_CHECK_EQ_INT(pcl_osc_init(&o, &cfg), PCL_OK);
    cfg.pulse_width = PCL_COEF(0.989);
    PCL_CHECK_EQ_INT(pcl_osc_init(&o, &cfg), PCL_OK);
    cfg.pulse_width = PCL_COEF(0.5);

    cfg.pwm_depth = PCL_COEF(-0.1);
    PCL_CHECK_EQ_INT(pcl_osc_init(&o, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.pwm_depth = PCL_COEF(0.5);
    PCL_CHECK_EQ_INT(pcl_osc_init(&o, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.pwm_depth = PCL_COEF(0.49);
    PCL_CHECK_EQ_INT(pcl_osc_init(&o, &cfg), PCL_OK);
}

static void test_setter_validation(void)
{
    pcl_osc_t o, before;
    size_t    i;

    make_osc(&o, PCL_OSC_SAW, 440.0, 1.0, 0.5, 0.0);
    before = o;

    PCL_CHECK_EQ_INT(pcl_osc_set_frequency(NULL, PCL_HZ(100.0)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_osc_set_frequency(&o, PCL_HZ(30000.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_osc_set_waveform(NULL, PCL_OSC_SAW), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_osc_set_waveform(&o, PCL_OSC_WAVEFORM_COUNT), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_osc_set_pitch_depth(NULL, PCL_OCT(1.0)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_osc_set_pitch_depth(&o, PCL_OCT(-1.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_osc_set_pulse_width(NULL, PCL_COEF(0.5)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_osc_set_pulse_width(&o, PCL_COEF(0.001)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_osc_set_pwm_depth(NULL, PCL_COEF(0.1)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_osc_set_pwm_depth(&o, PCL_COEF(0.9)), PCL_ERR_OUT_OF_RANGE);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    PCL_CHECK_EQ_INT(pcl_osc_set_frequency(&o, NAN), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_osc_set_pulse_width(&o, NAN), PCL_ERR_OUT_OF_RANGE);
#endif

    /* Rejected values changed nothing: both copies still produce the same audio. */
    {
        static pcl_sample_t a[256], b[256];
        render(&o, a, 256);
        render(&before, b, 256);
        PCL_CHECK(memcmp(a, b, sizeof a) == 0);
    }

    PCL_CHECK_EQ_INT(pcl_osc_set_frequency(&o, PCL_HZ(1000.0)), PCL_OK);
    for (i = 0; i < PCL_OSC_WAVEFORM_COUNT; i++)
        PCL_CHECK_EQ_INT(pcl_osc_set_waveform(&o, (pcl_osc_waveform_t)i), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_osc_set_pitch_depth(&o, PCL_OCT(2.0)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_osc_set_pulse_width(&o, PCL_COEF(0.25)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_osc_set_pwm_depth(&o, PCL_COEF(0.1)), PCL_OK);
}

/* ---- latency, phase ---- */

static void test_latency_and_set_phase(void)
{
    pcl_osc_t    o;
    pcl_sample_t out[8];

    make_osc(&o, PCL_OSC_SAW, 100.0, 1.0, 0.5, 0.0);
    render(&o, out, 8);
    PCL_CHECK(pcl_test_to_double(out[0]) == 0.0); /* the one-sample delay line starts empty */

    make_osc(&o, PCL_OSC_SAW, 100.0, 1.0, 0.5, 0.0);
    pcl_osc_set_phase(&o, 0x40000000u); /* quarter cycle: a rising saw is at -0.5 */
    render(&o, out, 8);
    PCL_CHECK_NEAR(pcl_test_to_double(out[1]), -0.5, 1e-3);
    PCL_CHECK(pcl_test_to_double(out[2]) > pcl_test_to_double(out[1])); /* and rising */

    make_osc(&o, PCL_OSC_SINE, 100.0, 1.0, 0.5, 0.0);
    pcl_osc_set_phase(&o, 0x40000000u); /* sine peak */
    render(&o, out, 8);
    PCL_CHECK_NEAR(pcl_test_to_double(out[1]), 1.0, 2e-4);
}

/* ---- reference model: the same algorithm in double precision ---- */

static double wrap01(double t) { return t - floor(t); }

static double ref_blep(double t, double dt)
{
    if (t < dt)
    {
        const double x = t / dt;
        return 2.0 * x - x * x - 1.0;
    }
    if (t > 1.0 - dt)
    {
        const double x = (t - 1.0) / dt;
        return x * x + 2.0 * x + 1.0;
    }
    return 0.0;
}

static double ref_blamp(double t, double dt)
{
    double x;
    if (t < dt)
        x = 1.0 - t / dt;
    else if (t > 1.0 - dt)
        x = 1.0 - (1.0 - t) / dt;
    else
        return 0.0;
    return x * x * x;
}

static double ref_wave(pcl_osc_waveform_t wf, double t, double dt, double width)
{
    switch (wf)
    {
    case PCL_OSC_SINE:
        return sin(2.0 * PI * t);
    case PCL_OSC_TRIANGLE:
    {
        const double tri = t < 0.5 ? -1.0 + 4.0 * t : 3.0 - 4.0 * t;
        return tri + (4.0 / 3.0) * dt * (ref_blamp(t, dt) - ref_blamp(wrap01(t - 0.5), dt));
    }
    case PCL_OSC_SAW:
        return 2.0 * t - 1.0 - ref_blep(t, dt);
    default:
        return (t < width ? 1.0 : -1.0) + ref_blep(t, dt) - ref_blep(wrap01(t - width), dt);
    }
}

static void check_against_model(pcl_osc_waveform_t wf, double hz, double width, double tol)
{
    pcl_osc_t    o;
    const size_t n = 2048;
    const double dt = hz / (double)SAMPLE_RATE_HZ;
    double       worst = 0.0;
    size_t       i;

    make_osc(&o, wf, hz, 1.0, width, 0.0);
    render(&o, g_out, n);
    for (i = 0; i + 1 < n; i++)
    {
        const double expected = ref_wave(wf, wrap01((double)i * dt), dt, width);
        const double err = fabs(pcl_test_to_double(g_out[i + 1]) - expected);
        if (err > worst)
            worst = err;
        PCL_CHECK_NEAR(pcl_test_to_double(g_out[i + 1]), expected, tol);
    }
    printf("    model %-8s %8.1f Hz: worst error %.2e\n", g_wave_names[wf], hz, worst);
}

static void test_matches_double_precision_model(void)
{
    size_t w;

    for (w = 0; w < 4; w++)
    {
        const double tol = (g_waves[w] == PCL_OSC_SINE) ? 1.5e-4 : 5e-4;
        check_against_model(g_waves[w], 1000.0, 0.3, tol);
        check_against_model(g_waves[w], 7321.5, 0.5, tol);
    }
}

/* ---- timing and frequency ---- */

/* Event times (in samples) of the phase returning to zero, from a sync_out buffer. */
static size_t collect_events(const pcl_sample_t *sync, size_t n, double *times, size_t cap)
{
    size_t i, count = 0;
    for (i = 0; i < n && count < cap; i++)
    {
        if (sync[i] != 0)
            times[count++] = (double)i - pcl_test_to_double(sync[i]);
    }
    return count;
}

static double g_events[MAX_FRAMES];

static void test_frequency_accuracy(void)
{
    static const double freqs[] = { 20.0, 55.5, 440.0, 1000.0, 9999.9, 20000.0 };
    size_t              f;

    for (f = 0; f < sizeof freqs / sizeof freqs[0]; f++)
    {
        pcl_osc_t o;
        size_t    count, k;
        double    period, expected, residual = 0.0;

        make_osc(&o, PCL_OSC_SAW, freqs[f], 1.0, 0.5, 0.0);
        pcl_osc_process(&o, g_out, NULL, NULL, NULL, g_sync, MAX_FRAMES);
        count = collect_events(g_sync, MAX_FRAMES, g_events, MAX_FRAMES);
        expected = (double)SAMPLE_RATE_HZ / freqs[f];
        PCL_CHECK(count >= 30);
        period = (g_events[count - 1] - g_events[0]) / (double)(count - 1);
        PCL_CHECK_NEAR(period / expected, 1.0, 5e-6);

        /* Event times are evenly spaced to sub-sample precision (no jitter). */
        for (k = 0; k < count; k++)
        {
            const double r = fabs(g_events[k] - (g_events[0] + (double)k * period));
            if (r > residual)
                residual = r;
        }
        PCL_CHECK(residual < 2e-3);
    }
}

static void test_pitch_modulation_in_octaves(void)
{
    /* A constant modulation signal bends the pitch by mod * depth octaves. */
    static const double depths[] = { 1.0, 3.0, 6.5 };
    static const double mods[] = { 0.3, -0.7, 0.5 };
    size_t              c;

    for (c = 0; c < 3; c++)
    {
        pcl_osc_t          o;
        const pcl_sample_t mod = pcl_test_from_double(mods[c]);
        /* Compare against the values as actually stored. */
        const double depth_q = depths[c];
        const double oct = depth_q * pcl_test_to_double(mod);
        const double base = 300.0;
        const double expected_hz = base * pow(2.0, oct);
        size_t       count, i;
        double       period, from, to;

        make_osc(&o, PCL_OSC_SAW, base, depths[c], 0.5, 0.0);
        for (i = 0; i < MAX_FRAMES; i++)
            g_mod_a[i] = mod;
        pcl_osc_process(&o, g_out, g_mod_a, NULL, NULL, g_sync, MAX_FRAMES);
        count = collect_events(g_sync, MAX_FRAMES, g_events, MAX_FRAMES);

        /* Skip the first control period, which ramps from the unmodulated pitch. */
        from = 2.0 * PCL_CONTROL_DIV;
        to = g_events[count - 1];
        {
            size_t first = 0;
            while (first < count && g_events[first] < from)
                first++;
            PCL_CHECK(count - first >= 20 || expected_hz < 100.0);
            period = (to - g_events[first]) / (double)(count - 1 - first);
        }
        PCL_CHECK_NEAR(((double)SAMPLE_RATE_HZ / period) / expected_hz, 1.0, 3e-5);
    }
}

static void test_null_modulation_equals_zero_modulation(void)
{
    pcl_osc_t           a, b;
    size_t              i;
    static pcl_sample_t out_a[1024], out_b[1024];

    for (i = 0; i < 1024; i++)
        g_mod_a[i] = 0;
    make_osc(&a, PCL_OSC_PULSE, 1234.5, 2.0, 0.4, 0.3);
    make_osc(&b, PCL_OSC_PULSE, 1234.5, 2.0, 0.4, 0.3);
    pcl_osc_process(&a, out_a, NULL, NULL, NULL, NULL, 1024);
    pcl_osc_process(&b, out_b, g_mod_a, g_mod_a, NULL, NULL, 1024);
    PCL_CHECK(memcmp(out_a, out_b, sizeof out_a) == 0);
}

static void test_frequency_glide_lands_on_target(void)
{
    pcl_osc_t o;
    size_t    count;
    double    period;

    make_osc(&o, PCL_OSC_SAW, 200.0, 1.0, 0.5, 0.0);
    PCL_CHECK_EQ_INT(pcl_osc_set_frequency(&o, PCL_HZ(1500.0)), PCL_OK);
    pcl_osc_process(&o, g_out, NULL, NULL, NULL, g_sync, 48000);
    count = collect_events(g_sync, 48000, g_events, MAX_FRAMES);
    /* Look at the last 100 events, long after the 5 ms glide. */
    period = (g_events[count - 1] - g_events[count - 101]) / 100.0;
    PCL_CHECK_NEAR(period / ((double)SAMPLE_RATE_HZ / 1500.0), 1.0, 5e-6);
}

/* ---- waveform shape ---- */

static void test_waveform_shapes(void)
{
    pcl_osc_t    o;
    const size_t n = 4800; /* 100 cycles of 1 kHz at 48 kHz */
    double       sum = 0.0;
    size_t       i;

    make_osc(&o, PCL_OSC_SAW, 1000.0, 1.0, 0.5, 0.0);
    render(&o, g_out, n);
    for (i = 1; i < n; i++)
        sum += pcl_test_to_double(g_out[i]);
    PCL_CHECK_NEAR(sum / (double)(n - 1), 0.0, 2e-3);

    make_osc(&o, PCL_OSC_TRIANGLE, 1000.0, 1.0, 0.5, 0.0);
    render(&o, g_out, n);
    sum = 0.0;
    for (i = 1; i < n; i++)
        sum += pcl_test_to_double(g_out[i]);
    PCL_CHECK_NEAR(sum / (double)(n - 1), 0.0, 2e-3);

    make_osc(&o, PCL_OSC_SINE, 1000.0, 1.0, 0.5, 0.0);
    render(&o, g_out, n);
    sum = 0.0;
    for (i = 1; i < n; i++)
        sum += pcl_test_to_double(g_out[i]);
    PCL_CHECK_NEAR(sum / (double)(n - 1), 0.0, 2e-3);

    /* Pulse: the DC component follows the width, 2 * width - 1. (The mean, not a count of
     * positive samples: at 1 kHz / 48 kHz a cycle is exactly 48 samples, so counting would
     * quantize the width to whole samples.) */
    make_osc(&o, PCL_OSC_PULSE, 1000.0, 1.0, 0.25, 0.0);
    render(&o, g_out, n);
    sum = 0.0;
    for (i = 1; i < n; i++)
        sum += pcl_test_to_double(g_out[i]);
    PCL_CHECK_NEAR(sum / (double)(n - 1), 2.0 * 0.25 - 1.0, 0.01);

    /* Peak levels stay at full scale (give or take the band-limiting ripple). */
    make_osc(&o, PCL_OSC_SAW, 220.0, 1.0, 0.5, 0.0);
    render(&o, g_out, 4800);
    {
        double peak = 0.0;
        for (i = 1; i < 4800; i++)
            if (fabs(pcl_test_to_double(g_out[i])) > peak)
                peak = fabs(pcl_test_to_double(g_out[i]));
        PCL_CHECK(peak > 0.97 && peak < 1.03);
    }
}

static double mean_from(const pcl_sample_t *x, size_t from, size_t to)
{
    double sum = 0.0;
    size_t i;
    for (i = from; i < to; i++)
        sum += pcl_test_to_double(x[i]);
    return sum / (double)(to - from);
}

static void test_pwm(void)
{
    pcl_osc_t           o;
    const size_t        n = 9600;
    size_t              i;
    static pcl_sample_t plain[1024], with_pwm[1024];

    /* width = 0.3 + 0.2 * 0.5 = 0.4, so the DC component is 2 * 0.4 - 1 = -0.2 */
    make_osc(&o, PCL_OSC_PULSE, 1000.0, 1.0, 0.3, 0.2);
    for (i = 0; i < n; i++)
        g_mod_a[i] = pcl_test_from_double(0.5);
    pcl_osc_process(&o, g_out, NULL, g_mod_a, NULL, NULL, n);
    PCL_CHECK_NEAR(mean_from(g_out, 1000, n), -0.2, 0.01);

    /* A negative modulation moves it to 0.3 - 0.1 = 0.2, DC -0.6. */
    for (i = 0; i < n; i++)
        g_mod_a[i] = pcl_test_from_double(-0.5);
    pcl_osc_process(&o, g_out, NULL, g_mod_a, NULL, NULL, n);
    PCL_CHECK_NEAR(mean_from(g_out, 1000, n), -0.6, 0.01);

    /* No modulation input: the base width. */
    pcl_osc_process(&o, g_out, NULL, NULL, NULL, NULL, n);
    PCL_CHECK_NEAR(mean_from(g_out, 1000, n), 2.0 * 0.3 - 1.0, 0.01);

    /* PWM only affects the pulse waveform. */
    for (i = 0; i < 1024; i++)
        g_mod_a[i] = pcl_test_from_double(0.9);
    make_osc(&o, PCL_OSC_SAW, 1000.0, 1.0, 0.3, 0.2);
    pcl_osc_process(&o, plain, NULL, NULL, NULL, NULL, 1024);
    make_osc(&o, PCL_OSC_SAW, 1000.0, 1.0, 0.3, 0.2);
    pcl_osc_process(&o, with_pwm, NULL, g_mod_a, NULL, NULL, 1024);
    PCL_CHECK(memcmp(plain, with_pwm, sizeof plain) == 0);
}

static void test_pulse_width_clamps_at_high_pitch(void)
{
    /* Above sample_rate/4 the two edges cannot be kept apart: the width collapses to 50 %,
     * i.e. no DC offset, instead of the requested 0.1 (which would give DC -0.8). */
    pcl_osc_t o;

    make_osc(&o, PCL_OSC_PULSE, 15000.0, 1.0, 0.1, 0.0);
    render(&o, g_out, 9600);
    PCL_CHECK_NEAR(mean_from(g_out, 1000, 9600), 0.0, 0.05);
}

/* ---- spectral measurements ---- */

static void fft(double *re, double *im, size_t n)
{
    size_t i, j, len;

    for (i = 1, j = 0; i < n; i++)
    {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j)
        {
            double t = re[i];
            re[i] = re[j];
            re[j] = t;
            t = im[i];
            im[i] = im[j];
            im[j] = t;
        }
    }
    for (len = 2; len <= n; len <<= 1)
    {
        const double ang = -2.0 * PI / (double)len;
        for (i = 0; i < n; i += len)
        {
            for (j = 0; j < len / 2; j++)
            {
                const double wr = cos(ang * (double)j), wi = sin(ang * (double)j);
                const double xr = re[i + j + len / 2] * wr - im[i + j + len / 2] * wi;
                const double xi = re[i + j + len / 2] * wi + im[i + j + len / 2] * wr;
                re[i + j + len / 2] = re[i + j] - xr;
                im[i + j + len / 2] = im[i + j] - xi;
                re[i + j] += xr;
                im[i + j] += xi;
            }
        }
    }
}

/*
 * Power outside the harmonics of f0 (aliased partials and noise), relative to the power of
 * the fundamental, in dB. Uses a 4-term Blackman-Harris window (sidelobes below -92 dB) and
 * skips +/-6 bins around each harmonic and around DC.
 */
static double nonharmonic_db(const double *x, double f0)
{
    static double        re[FFT_SIZE], im[FFT_SIZE];
    static unsigned char masked[FFT_SIZE / 2];
    const double         bin_hz = (double)SAMPLE_RATE_HZ / (double)FFT_SIZE;
    double               fund = 0.0, other = 0.0;
    size_t               k;
    int                  h;

    for (k = 0; k < FFT_SIZE; k++)
    {
        const double a = 2.0 * PI * (double)k / (double)FFT_SIZE;
        const double w
            = 0.35875 - 0.48829 * cos(a) + 0.14128 * cos(2.0 * a) - 0.01168 * cos(3.0 * a);
        re[k] = x[k] * w;
        im[k] = 0.0;
    }
    fft(re, im, FFT_SIZE);

    memset(masked, 0, sizeof masked);
    for (k = 0; k <= 6; k++)
        masked[k] = 1;
    for (h = 1; (double)h * f0 < (double)SAMPLE_RATE_HZ / 2.0; h++)
    {
        const long centre = (long)floor((double)h * f0 / bin_hz + 0.5);
        long       d;
        for (d = -6; d <= 6; d++)
        {
            const long b = centre + d;
            if (b >= 0 && b < (long)(FFT_SIZE / 2))
            {
                if (h == 1)
                    fund += re[b] * re[b] + im[b] * im[b];
                masked[b] = 1;
            }
        }
    }
    for (k = 1; k < FFT_SIZE / 2; k++)
        if (!masked[k])
            other += re[k] * re[k] + im[k] * im[k];
    return 10.0 * log10(other / fund);
}

static double naive_wave(pcl_osc_waveform_t wf, double t, double width)
{
    switch (wf)
    {
    case PCL_OSC_TRIANGLE:
        return t < 0.5 ? -1.0 + 4.0 * t : 3.0 - 4.0 * t;
    case PCL_OSC_SAW:
        return 2.0 * t - 1.0;
    default:
        return t < width ? 1.0 : -1.0;
    }
}

static void aliasing_case(
    pcl_osc_waveform_t wf, double hz, double min_improvement_db, double max_db)
{
    static double signal[FFT_SIZE], naive[FFT_SIZE];
    const double  width = 0.37;
    pcl_osc_t     o;
    double        lib_db, naive_db;
    size_t        i;

    make_osc(&o, wf, hz, 1.0, width, 0.0);
    render(&o, g_out, FFT_SIZE + 64);
    for (i = 0; i < FFT_SIZE; i++)
    {
        signal[i] = pcl_test_to_double(g_out[i + 64]);
        naive[i] = naive_wave(wf, wrap01((double)(i + 63) * hz / (double)SAMPLE_RATE_HZ), width);
    }
    lib_db = nonharmonic_db(signal, hz);
    naive_db = nonharmonic_db(naive, hz);
    printf("    alias %-8s %8.1f Hz: naive %6.1f dB, band-limited %6.1f dB (%.1f dB better)\n",
        g_wave_names[wf], hz, naive_db, lib_db, naive_db - lib_db);
    PCL_CHECK(naive_db - lib_db >= min_improvement_db);
    PCL_CHECK(lib_db <= max_db);
}

static void test_aliasing_is_reduced(void)
{
    /*
     * Limits come from the measured float build (docs/modules/oscillator.md), with about
     * 3 dB of margin; the improvement over the naive waveform must stay at least 10 dB.
     * A 2-point PolyBLEP is a modest filter (about 13-18 dB here), not a transparent one.
     */
    aliasing_case(PCL_OSC_SAW, 1234.5, 10.0, -26.0);
    aliasing_case(PCL_OSC_SAW, 3217.3, 10.0, -22.0);
    aliasing_case(PCL_OSC_SAW, 6011.1, 10.0, -15.0);
    aliasing_case(PCL_OSC_PULSE, 1234.5, 10.0, -27.0);
    aliasing_case(PCL_OSC_PULSE, 3217.3, 10.0, -27.0);
    aliasing_case(PCL_OSC_TRIANGLE, 1234.5, 10.0, -56.0);
    aliasing_case(PCL_OSC_TRIANGLE, 3217.3, 10.0, -46.0);
    aliasing_case(PCL_OSC_TRIANGLE, 6011.1, 10.0, -37.0);
}

static void test_sine_is_spectrally_pure(void)
{
    static double signal[FFT_SIZE];
    pcl_osc_t     o;
    size_t        i;
    double        db;

    make_osc(&o, PCL_OSC_SINE, 1234.5, 1.0, 0.5, 0.0);
    render(&o, g_out, FFT_SIZE + 64);
    for (i = 0; i < FFT_SIZE; i++)
        signal[i] = pcl_test_to_double(g_out[i + 64]);
    db = nonharmonic_db(signal, 1234.5);
    printf("    sine 1234.5 Hz: non-harmonic %6.1f dB\n", db);
    PCL_CHECK(db < -80.0);
}

/* ---- hard sync ---- */

static void test_sync_out_reports_phase_returns(void)
{
    pcl_osc_t    o;
    pcl_sample_t out_a[2048], sync_a[2048], out_b[2048];
    size_t       i, events = 0;

    /* Requesting sync_out must not change the audio. */
    make_osc(&o, PCL_OSC_SAW, 1000.0, 1.0, 0.5, 0.0);
    pcl_osc_process(&o, out_a, NULL, NULL, NULL, sync_a, 2048);
    make_osc(&o, PCL_OSC_SAW, 1000.0, 1.0, 0.5, 0.0);
    pcl_osc_process(&o, out_b, NULL, NULL, NULL, NULL, 2048);
    PCL_CHECK(memcmp(out_a, out_b, sizeof out_a) == 0);

    for (i = 0; i < 2048; i++)
    {
        if (sync_a[i] != 0)
        {
            const double d = pcl_test_to_double(sync_a[i]);
            PCL_CHECK(d > 0.0 && d < 1.0);
            events++;
        }
    }
    PCL_CHECK_EQ_INT(events, 42); /* 2048 samples / 48 samples per cycle */
}

static void test_sync_slave_follows_master(void)
{
    static double signal[FFT_SIZE], reference[FFT_SIZE];
    static double master_t[MAX_FRAMES];
    const double  master_hz = 1000.7, slave_hz = 2701.9;
    const size_t  total = FFT_SIZE + 64;
    pcl_osc_t     master, slave;
    size_t        i, events, k;
    double        lib_db, ref_db;

    make_osc(&master, PCL_OSC_SAW, master_hz, 1.0, 0.5, 0.0);
    pcl_osc_process(&master, g_out, NULL, NULL, NULL, g_sync, total);
    events = collect_events(g_sync, total, master_t, MAX_FRAMES);

    make_osc(&slave, PCL_OSC_SAW, slave_hz, 1.0, 0.5, 0.0);
    pcl_osc_process(&slave, g_out, NULL, NULL, g_sync, NULL, total);
    for (i = 0; i < FFT_SIZE; i++)
        signal[i] = pcl_test_to_double(g_out[i + 64]);

    /* Naive hard-sync reference: restart the slave at each exact master wrap. */
    k = 0;
    for (i = 0; i < FFT_SIZE; i++)
    {
        const double n = (double)(i + 63);
        double       since;
        while (k + 1 < events && master_t[k + 1] <= n)
            k++;
        since = n - master_t[k];
        reference[i] = 2.0 * wrap01(since * slave_hz / (double)SAMPLE_RATE_HZ) - 1.0;
    }

    lib_db = nonharmonic_db(signal, master_hz);
    ref_db = nonharmonic_db(reference, master_hz);
    printf("    sync saw: naive %6.1f dB, band-limited %6.1f dB (%.1f dB better)\n", ref_db, lib_db,
        ref_db - lib_db);
    PCL_CHECK(ref_db - lib_db >= 10.0); /* measured: 15 dB */
    PCL_CHECK(lib_db <= -10.0);         /* measured: -13.5 dB */
}

static void test_sync_every_sample_stays_sane(void)
{
    /* Worst case: a restart on every sample, at every sub-sample position. */
    size_t w;

    for (w = 0; w < 4; w++)
    {
        pcl_osc_t o;
        size_t    i;
        uint32_t  seed = 77u;
        double    peak = 0.0;

        make_osc(&o, g_waves[w], 3000.0, 1.0, 0.3, 0.0);
        for (i = 0; i < 4096; i++)
            g_sync[i] = pcl_test_from_double(pcl_test_rand(&seed) * 0.49 + 0.5);
        pcl_osc_process(&o, g_out, NULL, NULL, g_sync, NULL, 4096);
        for (i = 0; i < 4096; i++)
        {
            const double v = fabs(pcl_test_to_double(g_out[i]));
            if (v > peak)
                peak = v;
#if PCL_SAMPLE_TYPE == PCL_FLOAT
            PCL_CHECK(isfinite(g_out[i]));
#endif
        }
        PCL_CHECK(peak <= 1.5);
    }
}

/* A sync_in sample outside the contract (the fraction of a wrap, 0 or in (0, 1]) must not be
   undefined behaviour: a negative one (or NaN) is "no event", and one above 1 (float only) counts
   as 1. In the Q31 build a negative sample used to overflow a signed integer in osc_comp (found by
   UBSan with noise patched into a slave's sync input). */
static pcl_sample_t out_of_contract_sync(size_t i, pcl_sample_t *expected)
{
    pcl_sample_t bad;
    const double fractions[] = { 0.37, 0.0025, 0.9, 0.5 };

    switch (i % 8u)
    {
    case 0:
        bad = *expected
            = pcl_test_from_double(fractions[(i / 8u) % 4u]); /* a valid fraction passes through */
        return bad;
    case 1:
        bad = pcl_test_from_double(-0.0025); /* -5.4e6 in Q31: the value UBSan reported */
        break;
    case 2:
        bad = pcl_test_from_double(-0.9);
        break;
    case 3:
#if PCL_SAMPLE_TYPE == PCL_FLOAT
        bad = -1.0e30f;
#elif PCL_SAMPLE_TYPE == PCL_Q15
        bad = INT16_MIN;
#else
        bad = INT32_MIN;
#endif
        break;
    case 4:
#if PCL_SAMPLE_TYPE == PCL_FLOAT
        bad = NAN;
#else
        bad = -1; /* the smallest negative step */
#endif
        break;
    case 5:
#if PCL_SAMPLE_TYPE == PCL_FLOAT
        *expected = 1.0f; /* above 1: counts as 1 */
        return 2.5f;
#else
        *expected = 0;
        return 0;
#endif
    case 6:
#if PCL_SAMPLE_TYPE == PCL_FLOAT
        *expected = 1.0f;
        return INFINITY;
#else
        *expected = 0;
        return 0;
#endif
    default:
        *expected = 0; /* no event */
        return 0;
    }
    *expected = 0; /* a negative fraction or NaN: no event */
    return bad;
}

static void test_sync_in_outside_the_contract(void)
{
    static pcl_sample_t bad[4096], good[4096], out_bad[4096], out_good[4096], rep_bad[4096],
        rep_good[4096];
    size_t w, i;

    for (w = 0; w < 4; w++)
    {
        pcl_osc_t a, b;

        for (i = 0; i < 4096; i++)
            bad[i] = out_of_contract_sync(i, &good[i]);
        make_osc(&a, g_waves[w], 3000.0, 1.0, 0.3, 0.0);
        make_osc(&b, g_waves[w], 3000.0, 1.0, 0.3, 0.0);
        pcl_osc_process(&a, out_bad, NULL, NULL, bad, rep_bad, 4096);
        pcl_osc_process(&b, out_good, NULL, NULL, good, rep_good, 4096);
        /* the bad input behaves exactly like the sanitised one: same audio, same reported events */
        PCL_CHECK(memcmp(out_bad, out_good, sizeof out_bad) == 0);
        PCL_CHECK(memcmp(rep_bad, rep_good, sizeof rep_bad) == 0);
    }
}

/* ---- the contract every module's tests must cover (CLAUDE.md) ---- */

static void test_reset_is_deterministic(void)
{
    static pcl_sample_t out1[2048], out2[2048];
    size_t              w;

    for (w = 0; w < 4; w++)
    {
        pcl_osc_t o;
        make_osc(&o, g_waves[w], 1500.0, 2.0, 0.3, 0.2);
        fill_noise(g_mod_a, 2048, 11u, 0.8);
        pcl_osc_set_phase(&o, 0x12345678u);
        pcl_osc_process(&o, out1, g_mod_a, g_mod_a, NULL, NULL, 700); /* mid-ramp state */
        PCL_CHECK_EQ_INT(pcl_osc_set_frequency(&o, PCL_HZ(4000.0)), PCL_OK);

        pcl_osc_reset(&o);
        pcl_osc_process(&o, out1, g_mod_a, g_mod_a, NULL, NULL, 2048);
        pcl_osc_reset(&o);
        pcl_osc_process(&o, out2, g_mod_a, g_mod_a, NULL, NULL, 2048);
        PCL_CHECK(memcmp(out1, out2, sizeof out1) == 0);
    }
}

static void test_block_size_independence(void)
{
    static const size_t blocks[] = { 1, 7, 256 };
    static pcl_sample_t expected[2048], expected_sync[2048], out[2048], out_sync[2048];
    const size_t        total = 2048;
    size_t              w, b;

    /* Another oscillator provides realistic sync events. */
    {
        pcl_osc_t master;
        make_osc(&master, PCL_OSC_SAW, 1100.0, 1.0, 0.5, 0.0);
        pcl_osc_process(&master, g_out, NULL, NULL, NULL, g_sync, total);
    }
    fill_noise(g_mod_a, total, 21u, 0.6);
    fill_noise(g_mod_b, total, 22u, 1.0);

    for (w = 0; w < 4; w++)
    {
        pcl_osc_t base, ref;

        make_osc(&base, g_waves[w], 2300.0, 1.5, 0.4, 0.3);
        PCL_CHECK_EQ_INT(
            pcl_osc_set_frequency(&base, PCL_HZ(3100.0)), PCL_OK); /* glide spans blocks */
        ref = base;
        pcl_osc_process(&ref, expected, g_mod_a, g_mod_b, g_sync, expected_sync, total);

        for (b = 0; b < sizeof blocks / sizeof blocks[0]; b++)
        {
            pcl_osc_t o = base;
            size_t    pos;
            for (pos = 0; pos < total; pos += blocks[b])
            {
                const size_t n = (total - pos < blocks[b]) ? (total - pos) : blocks[b];
                pcl_osc_process(
                    &o, out + pos, g_mod_a + pos, g_mod_b + pos, g_sync + pos, out_sync + pos, n);
            }
            PCL_CHECK(memcmp(out, expected, sizeof out) == 0);
            PCL_CHECK(memcmp(out_sync, expected_sync, sizeof out_sync) == 0);
        }
    }
}

static void test_extreme_parameters_stay_finite(void)
{
    size_t w;

    for (w = 0; w < 4; w++)
    {
        pcl_osc_t o;
        size_t    i, block;
        double    peak = 0.0;

        /* Top of the range, full-scale square modulation at maximum depth, widths at
         * their limits, and a restart on every other sample. */
        make_osc(&o, g_waves[w], 21600.0, 7.9, 0.01, 0.49);
        for (i = 0; i < 4096; i++)
        {
            g_mod_a[i] = pcl_test_from_double((i & 32u) ? 1.0 : -1.0);
            g_mod_b[i] = pcl_test_from_double((i & 64u) ? 1.0 : -1.0);
            g_sync[i] = (i & 1u) ? pcl_test_from_double(0.999) : (pcl_sample_t)0;
        }
        for (block = 0; block < 100; block++)
        {
            if (block == 50)
            {
                PCL_CHECK_EQ_INT(pcl_osc_set_frequency(&o, PCL_HZ(0.0)), PCL_OK);
                PCL_CHECK_EQ_INT(pcl_osc_set_pulse_width(&o, PCL_COEF(0.99)), PCL_OK);
            }
            pcl_osc_process(&o, g_out, g_mod_a, g_mod_b, block % 2 ? g_sync : NULL, NULL, 4096);
            for (i = 0; i < 4096; i++)
            {
                const double v = fabs(pcl_test_to_double(g_out[i]));
                if (v > peak)
                    peak = v;
#if PCL_SAMPLE_TYPE == PCL_FLOAT
                PCL_CHECK(isfinite(g_out[i]));
                PCL_CHECK(fpclassify(g_out[i]) != FP_SUBNORMAL);
#endif
            }
        }
        PCL_CHECK(peak <= 1.5);
    }
}

int main(void)
{
    PCL_RUN_TEST(test_init_validation);
    PCL_RUN_TEST(test_setter_validation);
    PCL_RUN_TEST(test_latency_and_set_phase);
    PCL_RUN_TEST(test_matches_double_precision_model);
    PCL_RUN_TEST(test_frequency_accuracy);
    PCL_RUN_TEST(test_pitch_modulation_in_octaves);
    PCL_RUN_TEST(test_null_modulation_equals_zero_modulation);
    PCL_RUN_TEST(test_frequency_glide_lands_on_target);
    PCL_RUN_TEST(test_waveform_shapes);
    PCL_RUN_TEST(test_pwm);
    PCL_RUN_TEST(test_pulse_width_clamps_at_high_pitch);
    PCL_RUN_TEST(test_aliasing_is_reduced);
    PCL_RUN_TEST(test_sine_is_spectrally_pure);
    PCL_RUN_TEST(test_sync_out_reports_phase_returns);
    PCL_RUN_TEST(test_sync_slave_follows_master);
    PCL_RUN_TEST(test_sync_every_sample_stays_sane);
    PCL_RUN_TEST(test_sync_in_outside_the_contract);
    PCL_RUN_TEST(test_reset_is_deterministic);
    PCL_RUN_TEST(test_block_size_independence);
    PCL_RUN_TEST(test_extreme_parameters_stay_finite);
    return pcl_test_summary("osc");
}
