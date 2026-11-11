/* Tests for the unison oscillator. */
#include <string.h>

#include "pcl_test.h"

#define SAMPLE_RATE_HZ 48000u
#define MAX_FRAMES 48000u
#define SEED 12345u

static pcl_sample_t g_l[MAX_FRAMES];
static pcl_sample_t g_r[MAX_FRAMES];
static pcl_sample_t g_l2[MAX_FRAMES];
static pcl_sample_t g_r2[MAX_FRAMES];
static pcl_sample_t g_v[PCL_UNISON_MAX_VOICES][MAX_FRAMES];
static pcl_sample_t g_pitch[MAX_FRAMES];
static pcl_sample_t g_pwm[MAX_FRAMES];
static pcl_sample_t g_sync[MAX_FRAMES];

/* 1 / sqrt(n), as the module's table stores it. */
static const pcl_coef_t g_mix[8] = { PCL_COEF(1.0), PCL_COEF(0.70710678118654752),
    PCL_COEF(0.57735026918962576), PCL_COEF(0.5), PCL_COEF(0.44721359549995794),
    PCL_COEF(0.40824829046386302), PCL_COEF(0.37796447300922723), PCL_COEF(0.35355339059327376) };

static pcl_unison_config_t make_cfg(
    pcl_osc_waveform_t wf, double hz, size_t voices, double detune, double width)
{
    pcl_unison_config_t cfg;
    cfg.osc.sample_rate_hz = SAMPLE_RATE_HZ;
    cfg.osc.waveform = wf;
    cfg.osc.frequency = PCL_HZ(hz);
    cfg.osc.pitch_depth_oct = PCL_OCT(1.0);
    cfg.osc.pulse_width = PCL_COEF(0.4);
    cfg.osc.pwm_depth = PCL_COEF(0.3);
    cfg.voices = voices;
    cfg.detune_oct = PCL_OCT(detune);
    cfg.stereo_width = PCL_COEF(width);
    cfg.seed = SEED;
    return cfg;
}

static void make(
    pcl_unison_t *u, pcl_osc_waveform_t wf, double hz, size_t voices, double detune, double width)
{
    const pcl_unison_config_t cfg = make_cfg(wf, hz, voices, detune, width);
    PCL_CHECK_EQ_INT(pcl_unison_init(u, &cfg), PCL_OK);
}

static void fill_noise(pcl_sample_t *buf, size_t n, uint32_t seed, double amplitude)
{
    size_t i;
    for (i = 0; i < n; i++)
        buf[i] = pcl_test_from_double(pcl_test_rand(&seed) * amplitude);
}

/* Sparse sync events at random sub-sample positions, like a master's sync_out. */
static void fill_sync(pcl_sample_t *buf, size_t n, uint32_t seed)
{
    size_t i;
    for (i = 0; i < n; i++)
    {
        const double r = pcl_test_rand(&seed);
        buf[i] = (i % 97u) == 13u ? pcl_test_from_double(0.05 + 0.9 * (r * 0.5 + 0.5)) : 0;
    }
}

/* Sample * gain, rounded and saturated exactly as the module mixes. */
static double scaled(pcl_sample_t x, pcl_coef_t g)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return (double)(x * g);
#elif PCL_SAMPLE_TYPE == PCL_Q15
    return (double)pcl_mul_q15_q30(x, g);
#else
    return (double)pcl_mul_q31_q30(x, g);
#endif
}

#if PCL_SAMPLE_TYPE != PCL_FLOAT
static pcl_sample_t saturate(double acc)
{
#if PCL_SAMPLE_TYPE == PCL_Q15
    return acc > 32767.0 ? INT16_MAX : (acc < -32768.0 ? INT16_MIN : (int16_t)acc);
#else
    return acc > 2147483647.0 ? INT32_MAX : (acc < -2147483648.0 ? INT32_MIN : (int32_t)acc);
#endif
}
#endif

static int all_finite(const pcl_sample_t *x, size_t n)
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

/* ---- validation ---- */

static void test_init_validation(void)
{
    pcl_unison_t        u, before;
    pcl_unison_config_t cfg = make_cfg(PCL_OSC_SAW, 440.0, 4, 0.2, 0.5);

    PCL_CHECK_EQ_INT(pcl_unison_init(&u, &cfg), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_unison_init(NULL, &cfg), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_unison_init(&u, NULL), PCL_ERR_NULL_POINTER);
    before = u;

    cfg.voices = 0;
    PCL_CHECK_EQ_INT(pcl_unison_init(&u, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.voices = PCL_UNISON_MAX_VOICES + 1u;
    PCL_CHECK_EQ_INT(pcl_unison_init(&u, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.voices = 4;

    cfg.detune_oct = PCL_OCT(-0.01);
    PCL_CHECK_EQ_INT(pcl_unison_init(&u, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.detune_oct = PCL_OCT(1.01);
    PCL_CHECK_EQ_INT(pcl_unison_init(&u, &cfg), PCL_ERR_OUT_OF_RANGE);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    cfg.detune_oct = NAN;
    PCL_CHECK_EQ_INT(pcl_unison_init(&u, &cfg), PCL_ERR_OUT_OF_RANGE);
#endif
    cfg.detune_oct = PCL_OCT(0.2);

    cfg.stereo_width = PCL_COEF(-0.01);
    PCL_CHECK_EQ_INT(pcl_unison_init(&u, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.stereo_width = PCL_COEF(1.01);
    PCL_CHECK_EQ_INT(pcl_unison_init(&u, &cfg), PCL_ERR_OUT_OF_RANGE);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    cfg.stereo_width = NAN;
    PCL_CHECK_EQ_INT(pcl_unison_init(&u, &cfg), PCL_ERR_OUT_OF_RANGE);
#endif
    cfg.stereo_width = PCL_COEF(0.5);

    /* The oscillator fields are validated as pcl_osc_init does. */
    cfg.osc.frequency = PCL_HZ(21601.0);
    PCL_CHECK_EQ_INT(pcl_unison_init(&u, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.osc.frequency = PCL_HZ(440.0);
    cfg.osc.waveform = PCL_OSC_WAVEFORM_COUNT;
    PCL_CHECK_EQ_INT(pcl_unison_init(&u, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.osc.waveform = PCL_OSC_SAW;
    cfg.osc.sample_rate_hz = 1000u;
    PCL_CHECK_EQ_INT(pcl_unison_init(&u, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.osc.sample_rate_hz = SAMPLE_RATE_HZ;

    PCL_CHECK(memcmp(&u, &before, sizeof u) == 0);

    /* Range edges are accepted. */
    cfg.voices = 1;
    cfg.detune_oct = PCL_OCT(0.0);
    cfg.stereo_width = PCL_COEF(0.0);
    PCL_CHECK_EQ_INT(pcl_unison_init(&u, &cfg), PCL_OK);
    cfg.voices = PCL_UNISON_MAX_VOICES;
    cfg.detune_oct = PCL_OCT(1.0);
    cfg.stereo_width = PCL_COEF(1.0);
    cfg.osc.frequency = PCL_HZ(21600.0);
    PCL_CHECK_EQ_INT(pcl_unison_init(&u, &cfg), PCL_OK);
}

static void test_setter_validation(void)
{
    pcl_unison_t u, before;

    make(&u, PCL_OSC_SAW, 440.0, 4, 0.2, 0.5);
    before = u;

    PCL_CHECK_EQ_INT(pcl_unison_set_frequency(NULL, PCL_HZ(100.0)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_unison_set_frequency(&u, PCL_HZ(21601.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_unison_set_detune(NULL, PCL_OCT(0.1)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_unison_set_detune(&u, PCL_OCT(1.5)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_unison_set_voices(NULL, 2), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_unison_set_voices(&u, 0), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_unison_set_voices(&u, PCL_UNISON_MAX_VOICES + 1u), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_unison_set_stereo_width(NULL, PCL_COEF(0.1)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_unison_set_stereo_width(&u, PCL_COEF(-0.5)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_unison_set_waveform(NULL, PCL_OSC_SINE), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_unison_set_waveform(&u, PCL_OSC_WAVEFORM_COUNT), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_unison_set_pitch_depth(NULL, PCL_OCT(1.0)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_unison_set_pitch_depth(&u, PCL_OCT(-1.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_unison_set_pulse_width(NULL, PCL_COEF(0.5)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_unison_set_pulse_width(&u, PCL_COEF(0.001)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_unison_set_pwm_depth(NULL, PCL_COEF(0.1)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_unison_set_pwm_depth(&u, PCL_COEF(0.6)), PCL_ERR_OUT_OF_RANGE);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    PCL_CHECK_EQ_INT(pcl_unison_set_frequency(&u, NAN), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_unison_set_detune(&u, NAN), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_unison_set_stereo_width(&u, NAN), PCL_ERR_OUT_OF_RANGE);
#endif
    PCL_CHECK(memcmp(&u, &before, sizeof u) == 0);

    PCL_CHECK_EQ_INT(pcl_unison_set_frequency(&u, PCL_HZ(220.0)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_unison_set_detune(&u, PCL_OCT(1.0)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_unison_set_voices(&u, PCL_UNISON_MAX_VOICES), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_unison_set_stereo_width(&u, PCL_COEF(1.0)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_unison_set_waveform(&u, PCL_OSC_PULSE), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_unison_set_pitch_depth(&u, PCL_OCT(2.0)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_unison_set_pulse_width(&u, PCL_COEF(0.3)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_unison_set_pwm_depth(&u, PCL_COEF(0.2)), PCL_OK);
}

/* ---- equivalence with pcl_osc ---- */

static void test_single_voice_equals_oscillator(void)
{
    size_t w;

    fill_noise(g_pitch, MAX_FRAMES, 1u, 0.5);
    fill_noise(g_pwm, MAX_FRAMES, 2u, 1.0);
    fill_sync(g_sync, MAX_FRAMES, 3u);

    for (w = 0; w < (size_t)PCL_OSC_WAVEFORM_COUNT; w++)
    {
        const pcl_unison_config_t cfg = make_cfg((pcl_osc_waveform_t)w, 523.25, 1, 0.7, 1.0);
        pcl_unison_t              u;
        pcl_osc_t                 o;
        uint32_t                  state = SEED;
        size_t                    i;
        int                       same = 1;

        PCL_CHECK_EQ_INT(pcl_unison_init(&u, &cfg), PCL_OK);
        PCL_CHECK_EQ_INT(pcl_osc_init(&o, &cfg.osc), PCL_OK);
        /* Voice 0 starts at the first xorshift32 draw from the seed. */
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        pcl_osc_set_phase(&o, state);

        pcl_unison_process(&u, g_l, NULL, g_pitch, g_pwm, g_sync, MAX_FRAMES);
        pcl_osc_process(&o, g_l2, g_pitch, g_pwm, g_sync, NULL, MAX_FRAMES);
        for (i = 0; i < MAX_FRAMES; i++)
            same &= g_l[i] == g_l2[i];
        PCL_CHECK(same);
    }
}

/*
 * The mono mix equals 1/sqrt(N) times the sum of the voices run on their own (bit-exact in
 * fixed point), and stays within 1.5 N + 1 LSB of the exact mix: per voice 0.5 LSB of rounding
 * plus up to 1 LSB from the Q2.30 gain (its half-step is 2^-31), and 1 LSB of slack.
 */
static void test_mono_mix_equals_sum_of_voices(void)
{
    const size_t n = 5;
    pcl_unison_t u;
    pcl_osc_t    v[PCL_UNISON_MAX_VOICES];
    size_t       i, k;
    int          same = 1;
    double       worst = 0.0;

    fill_noise(g_pitch, MAX_FRAMES, 4u, 0.3);
    fill_noise(g_pwm, MAX_FRAMES, 5u, 1.0);
    fill_sync(g_sync, MAX_FRAMES, 6u);

    make(&u, PCL_OSC_PULSE, 330.0, n, 0.3, 0.8);
    for (k = 0; k < n; k++)
        v[k] = u.voice[k]; /* white-box: start from the voices' exact state */

    pcl_unison_process(&u, g_l, NULL, g_pitch, g_pwm, g_sync, MAX_FRAMES);
    for (k = 0; k < n; k++)
        pcl_osc_process(&v[k], g_v[k], g_pitch, g_pwm, g_sync, NULL, MAX_FRAMES);

    for (i = 0; i < MAX_FRAMES; i++)
    {
        double acc = 0.0, exact = 0.0;
        for (k = 0; k < n; k++)
        {
            acc += scaled(g_v[k][i], g_mix[n - 1u]);
            exact += pcl_test_to_double(g_v[k][i]) / sqrt((double)n);
        }
#if PCL_SAMPLE_TYPE == PCL_FLOAT
        same &= fabs((double)g_l[i] - acc) <= 1e-6;
#else
        same &= g_l[i] == saturate(acc);
#endif
        {
            const double err = fabs(pcl_test_to_double(g_l[i]) - pcl_test_clip(exact));
            if (err > worst)
                worst = err;
        }
    }
    PCL_CHECK(same);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    PCL_CHECK(worst <= 1e-6);
#elif PCL_SAMPLE_TYPE == PCL_Q15
    PCL_CHECK(worst <= (1.5 * (double)n + 1.0) / 32768.0);
#else
    PCL_CHECK(worst <= (1.5 * (double)n + 1.0) / 2147483648.0);
#endif
    printf("    mono mix vs exact: worst %.3g\n", worst);
}

/* ---- detune ---- */

/* Mean frequency from a voice's sync_out (wrap times with sub-sample fractions). */
static double measure_hz(pcl_osc_t *o, size_t frames)
{
    double first = -1.0, last = -1.0;
    size_t count = 0, i;

    pcl_osc_process(o, g_l2, NULL, NULL, NULL, g_r2, frames);
    for (i = 0; i < frames; i++)
    {
        if (g_r2[i] != 0)
        {
            const double t = (double)i - pcl_test_to_double(g_r2[i]);
            if (first < 0.0)
                first = t;
            last = t;
            count++;
        }
    }
    return count > 1u ? (double)(count - 1u) * SAMPLE_RATE_HZ / (last - first) : 0.0;
}

static void test_detune_spread(void)
{
    static const size_t counts[] = { 2, 3, 4, 7, 8 };
    const double        base = 440.0, detune = 0.25;
    size_t              c;

    for (c = 0; c < sizeof counts / sizeof counts[0]; c++)
    {
        const size_t n = counts[c];
        pcl_unison_t u;
        double       hz[PCL_UNISON_MAX_VOICES];
        size_t       k;

        if (n > (size_t)PCL_UNISON_MAX_VOICES)
            continue;
        make(&u, PCL_OSC_SAW, base, n, detune, 0.0);
        for (k = 0; k < n; k++)
        {
            pcl_osc_t    o = u.voice[k];
            const double off = detune * ((double)k / (double)(n - 1u) - 0.5);
            const double want = base * pow(2.0, off);
            hz[k] = measure_hz(&o, MAX_FRAMES);
            PCL_CHECK_NEAR(hz[k] / want, 1.0, 5e-5);
        }
        /* Symmetric around the base pitch (in octaves). */
        for (k = 0; k < n / 2u; k++)
            PCL_CHECK_NEAR(hz[k] * hz[n - 1u - k] / (base * base), 1.0, 1e-4);
    }
}

static void test_detune_glides_and_clamps(void)
{
    pcl_unison_t u;
    pcl_osc_t    o;

    /* A new detune glides: after the smoothing time the outer voice sits at the new offset. */
    make(&u, PCL_OSC_SAW, 1000.0, 3, 0.0, 0.0);
    PCL_CHECK_EQ_INT(pcl_unison_set_detune(&u, PCL_OCT(0.5)), PCL_OK);
    pcl_unison_process(&u, g_l, NULL, NULL, NULL, NULL, 2400); /* 50 ms */
    o = u.voice[2];
    PCL_CHECK_NEAR(measure_hz(&o, MAX_FRAMES) / (1000.0 * pow(2.0, 0.25)), 1.0, 5e-5);
    o = u.voice[1];
    PCL_CHECK_NEAR(measure_hz(&o, MAX_FRAMES) / 1000.0, 1.0, 5e-5);

    /* Voices pushed above 0.45 * sample rate are held there; the base stays valid. */
    make(&u, PCL_OSC_SAW, 20000.0, 2, 1.0, 0.0);
    o = u.voice[1];
    PCL_CHECK_NEAR(measure_hz(&o, MAX_FRAMES) / 21600.0, 1.0, 1e-3);
    o = u.voice[0];
    PCL_CHECK_NEAR(measure_hz(&o, MAX_FRAMES) / (20000.0 * pow(2.0, -0.5)), 1.0, 1e-4);
}

/* ---- stereo ---- */

static void test_stereo_width(void)
{
    pcl_unison_t u;
    pcl_osc_t    v0, v1;
    size_t       i;
    int          same = 1;
    double       worst_l = 0.0, worst_r = 0.0, worst_c = 0.0;

    /* Width 0: L == R == mono * cos(pi/4). */
    make(&u, PCL_OSC_SAW, 220.0, 6, 0.2, 0.0);
    pcl_unison_process(&u, g_l, g_r, NULL, NULL, NULL, MAX_FRAMES);
    make(&u, PCL_OSC_SAW, 220.0, 6, 0.2, 0.0);
    pcl_unison_process(&u, g_l2, NULL, NULL, NULL, NULL, MAX_FRAMES);
    for (i = 0; i < MAX_FRAMES; i++)
    {
        const double mono = pcl_test_to_double(g_l2[i]);
        const double err = fabs(pcl_test_to_double(g_l[i]) - mono * sqrt(0.5));
        same &= g_l[i] == g_r[i];
        /* Skip samples where the mono mix of 6 voices saturated in fixed point. */
        if (fabs(mono) < 0.999 && err > worst_c)
            worst_c = err;
    }
    PCL_CHECK(same);
    PCL_CHECK(worst_c <= 3e-4);

    /* Width 1, two voices: voice 0 hard left, voice 1 hard right, each at 1/sqrt(2). */
    make(&u, PCL_OSC_SAW, 220.0, 2, 0.2, 1.0);
    v0 = u.voice[0];
    v1 = u.voice[1];
    pcl_unison_process(&u, g_l, g_r, NULL, NULL, NULL, MAX_FRAMES);
    pcl_osc_process(&v0, g_v[0], NULL, NULL, NULL, NULL, MAX_FRAMES);
    pcl_osc_process(&v1, g_v[1], NULL, NULL, NULL, NULL, MAX_FRAMES);
    for (i = 0; i < MAX_FRAMES; i++)
    {
        const double el
            = fabs(pcl_test_to_double(g_l[i]) - pcl_test_to_double(g_v[0][i]) * sqrt(0.5));
        const double er
            = fabs(pcl_test_to_double(g_r[i]) - pcl_test_to_double(g_v[1][i]) * sqrt(0.5));
        if (el > worst_l)
            worst_l = el;
        if (er > worst_r)
            worst_r = er;
    }
    PCL_CHECK(worst_l <= 3e-4);
    PCL_CHECK(worst_r <= 3e-4);
    printf("    pan: centre %.3g, hard L %.3g, hard R %.3g\n", worst_c, worst_l, worst_r);
}

/* After a width ramp the output equals a unison initialized at the new width. */
static void test_width_ramp_lands_on_target(void)
{
    const size_t switch_at = 1000, settle = 1000;
    pcl_unison_t a, b;
    size_t       i;
    int          same = 1;
    double       max_step = 0.0;

    make(&a, PCL_OSC_SINE, 150.0, 4, 0.1, 0.0);
    make(&b, PCL_OSC_SINE, 150.0, 4, 0.1, 1.0);
    pcl_unison_process(&a, g_l, g_r, NULL, NULL, NULL, switch_at);
    pcl_unison_process(&b, g_l2, g_r2, NULL, NULL, NULL, switch_at);
    PCL_CHECK_EQ_INT(pcl_unison_set_stereo_width(&a, PCL_COEF(1.0)), PCL_OK);
    pcl_unison_process(
        &a, g_l + switch_at, g_r + switch_at, NULL, NULL, NULL, MAX_FRAMES - switch_at);
    pcl_unison_process(
        &b, g_l2 + switch_at, g_r2 + switch_at, NULL, NULL, NULL, MAX_FRAMES - switch_at);

    for (i = switch_at + settle; i < MAX_FRAMES; i++)
        same &= g_l[i] == g_l2[i] && g_r[i] == g_r2[i];
    PCL_CHECK(same);

    /* No jump at the switch: a 150 Hz sine moves far less than 0.05 per sample. */
    for (i = switch_at - 10u; i < switch_at + settle; i++)
    {
        const double d = fabs(pcl_test_to_double(g_l[i + 1u]) - pcl_test_to_double(g_l[i]));
        if (d > max_step)
            max_step = d;
    }
    PCL_CHECK(max_step < 0.05);
}

/* ---- voice count ---- */

static void test_voice_count_change(void)
{
    pcl_unison_t u;
    pcl_osc_t    v0;
    size_t       i;
    int          same = 1;

    make(&u, PCL_OSC_SAW, 300.0, PCL_UNISON_MAX_VOICES, 0.4, 0.7);
    pcl_unison_process(&u, g_l, g_r, NULL, NULL, NULL, 1000);
    PCL_CHECK(all_finite(g_l, 1000) && all_finite(g_r, 1000));

    /* One voice left: the mono output is exactly that voice. */
    PCL_CHECK_EQ_INT(pcl_unison_set_voices(&u, 1), PCL_OK);
    v0 = u.voice[0];
    pcl_unison_process(&u, g_l, NULL, NULL, NULL, NULL, 4000);
    pcl_osc_process(&v0, g_l2, NULL, NULL, NULL, NULL, 4000);
    for (i = 0; i < 4000u; i++)
        same &= g_l[i] == g_l2[i];
    PCL_CHECK(same);

    PCL_CHECK_EQ_INT(pcl_unison_set_voices(&u, 3), PCL_OK);
    pcl_unison_process(&u, g_l, g_r, NULL, NULL, NULL, 4000);
    PCL_CHECK(all_finite(g_l, 4000) && all_finite(g_r, 4000));
}

/* ---- determinism and blocking ---- */

static void test_reset_is_deterministic(void)
{
    pcl_unison_t u;
    size_t       i;
    int          same = 1;

    fill_noise(g_pitch, MAX_FRAMES, 7u, 0.2);
    make(&u, PCL_OSC_TRIANGLE, 440.0, 5, 0.3, 0.6);
    pcl_unison_process(&u, g_l, g_r, g_pitch, NULL, NULL, 10000);
    PCL_CHECK_EQ_INT(pcl_unison_set_stereo_width(&u, PCL_COEF(0.2)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_unison_set_frequency(&u, PCL_HZ(600.0)), PCL_OK);
    pcl_unison_process(&u, g_l, g_r, g_pitch, NULL, NULL, 100);
    pcl_unison_reset(&u);
    pcl_unison_process(&u, g_l, g_r, g_pitch, NULL, NULL, 10000);
    pcl_unison_reset(&u);
    pcl_unison_process(&u, g_l2, g_r2, g_pitch, NULL, NULL, 10000);
    for (i = 0; i < 10000u; i++)
        same &= g_l[i] == g_l2[i] && g_r[i] == g_r2[i];
    PCL_CHECK(same);
}

/*
 * Blocks of 1, 7, 33 and 256 give the same output as one big block, with pitch mod, PWM, sync,
 * a glide and a width ramp active, in stereo and mono.
 */
static void render_blocked(int stereo, size_t block, pcl_sample_t *l, pcl_sample_t *r)
{
    const size_t change_at = 3000;
    pcl_unison_t u;
    size_t       pos = 0;

    make(&u, PCL_OSC_PULSE, 250.0, PCL_UNISON_MAX_VOICES, 0.35, 0.3);
    while (pos < MAX_FRAMES)
    {
        size_t n = MAX_FRAMES - pos < block ? MAX_FRAMES - pos : block;
        if (pos < change_at && pos + n > change_at)
            n = change_at - pos;
        if (pos == change_at)
        {
            PCL_CHECK_EQ_INT(pcl_unison_set_stereo_width(&u, PCL_COEF(0.9)), PCL_OK);
            PCL_CHECK_EQ_INT(pcl_unison_set_frequency(&u, PCL_HZ(410.0)), PCL_OK);
            PCL_CHECK_EQ_INT(pcl_unison_set_detune(&u, PCL_OCT(0.1)), PCL_OK);
        }
        pcl_unison_process(
            &u, l + pos, stereo ? r + pos : NULL, g_pitch + pos, g_pwm + pos, g_sync + pos, n);
        pos += n;
    }
}

static void test_block_size_independence(void)
{
    static const size_t blocks[] = { 1, 7, 33, 256 };
    int                 stereo;

    fill_noise(g_pitch, MAX_FRAMES, 8u, 0.3);
    fill_noise(g_pwm, MAX_FRAMES, 9u, 1.0);
    fill_sync(g_sync, MAX_FRAMES, 10u);

    for (stereo = 0; stereo <= 1; stereo++)
    {
        size_t b;
        render_blocked(stereo, MAX_FRAMES, g_l, g_r);
        for (b = 0; b < sizeof blocks / sizeof blocks[0]; b++)
        {
            size_t i;
            int    same = 1;
            render_blocked(stereo, blocks[b], g_l2, g_r2);
            for (i = 0; i < MAX_FRAMES; i++)
                same &= g_l[i] == g_l2[i] && (!stereo || g_r[i] == g_r2[i]);
            PCL_CHECK(same);
        }
    }
}

/* ---- extremes ---- */

static void test_in_phase_voices_saturate(void)
{
    /* A sync event on every voice at once forces all voices into phase. */
    const size_t n = PCL_UNISON_MAX_VOICES;
    pcl_unison_t u;
    pcl_osc_t    v0;
    size_t       i, k;
    int          same = 1;

    for (i = 0; i < MAX_FRAMES; i++)
        g_sync[i] = (i % 200u) == 0u ? pcl_test_from_double(0.5) : 0;
    make(&u, PCL_OSC_PULSE, 50.0, n, 0.0, 0.0);
    v0 = u.voice[0];
    pcl_unison_process(&u, g_l, NULL, NULL, NULL, g_sync, MAX_FRAMES);
    pcl_osc_process(&v0, g_v[0], NULL, NULL, g_sync, NULL, MAX_FRAMES);
    /* From the first sync on, every voice equals voice 0. */
    for (i = 2; i < MAX_FRAMES; i++)
    {
        double acc = 0.0;
        for (k = 0; k < n; k++)
            acc += scaled(g_v[0][i], g_mix[n - 1u]);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
        same &= fabs((double)g_l[i] - acc) <= 1e-5;
#else
        same &= g_l[i] == saturate(acc); /* clips, never wraps */
#endif
    }
    PCL_CHECK(same);
}

static void test_extreme_parameters_stay_finite(void)
{
    pcl_unison_t u;
    size_t       i;
    double       peak = 0.0;

    for (i = 0; i < MAX_FRAMES; i++)
    {
        g_pitch[i] = pcl_test_from_double((i / 64u) % 2u ? 0.999 : -1.0);
        g_sync[i] = (i % 3u) == 0u ? pcl_test_from_double(0.37) : 0;
    }
    fill_noise(g_pwm, MAX_FRAMES, 11u, 1.0);
    make(&u, PCL_OSC_PULSE, 21600.0, PCL_UNISON_MAX_VOICES, 1.0, 1.0);
    PCL_CHECK_EQ_INT(pcl_unison_set_pitch_depth(&u, PCL_OCT(7.9)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_unison_set_pwm_depth(&u, PCL_COEF(0.49)), PCL_OK);
    pcl_unison_process(&u, g_l, g_r, g_pitch, g_pwm, g_sync, MAX_FRAMES);
    PCL_CHECK(all_finite(g_l, MAX_FRAMES) && all_finite(g_r, MAX_FRAMES));
    for (i = 0; i < MAX_FRAMES; i++)
    {
        const double a = fabs(pcl_test_to_double(g_l[i]));
        if (a > peak)
            peak = a;
    }
    /* sqrt(8) voices' worth of a slightly overshooting waveform at most. */
    PCL_CHECK(peak < 4.0);
}

int main(void)
{
    PCL_RUN_TEST(test_init_validation);
    PCL_RUN_TEST(test_setter_validation);
    PCL_RUN_TEST(test_single_voice_equals_oscillator);
    PCL_RUN_TEST(test_mono_mix_equals_sum_of_voices);
    PCL_RUN_TEST(test_detune_spread);
    PCL_RUN_TEST(test_detune_glides_and_clamps);
    PCL_RUN_TEST(test_stereo_width);
    PCL_RUN_TEST(test_width_ramp_lands_on_target);
    PCL_RUN_TEST(test_voice_count_change);
    PCL_RUN_TEST(test_reset_is_deterministic);
    PCL_RUN_TEST(test_block_size_independence);
    PCL_RUN_TEST(test_in_phase_voices_saturate);
    PCL_RUN_TEST(test_extreme_parameters_stay_finite);
    return pcl_test_summary("unison");
}
