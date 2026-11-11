/* Tests for the sampler. */
#include <string.h>

#include "pcl_test.h"

#define FS 48000u
#define DATA_FRAMES 48000u
#define MAX_FRAMES (3u * 48000u)
#define PI 3.14159265358979323846
#define SINE_HZ 375.0 /* 128 samples per cycle at 48 kHz: loops of whole cycles */

static int16_t      g_q15[DATA_FRAMES];
static pcl_sample_t g_nat[DATA_FRAMES];
static pcl_sample_t g_out[MAX_FRAMES];
static pcl_sample_t g_out2[MAX_FRAMES];
static pcl_sample_t g_mod[MAX_FRAMES];

static double data_value(size_t i, int kind)
{
    if (kind == 0) /* sine */
        return 0.6 * sin(2.0 * PI * SINE_HZ * (double)i / FS);
    if (kind == 1) /* ramp: frame i is i / 65536, exact in every format */
        return (double)i / 65536.0;
    {
        uint32_t seed = 1234u + (uint32_t)i * 2654435761u; /* white noise, deterministic */
        return 0.7 * pcl_test_rand(&seed);
    }
}

/* Fills both data formats with the same values, rounded to Q15 so the formats agree exactly. */
static void make_data(int kind)
{
    size_t i;
    for (i = 0; i < DATA_FRAMES; i++)
    {
        const double v = floor(data_value(i, kind) * 32768.0 + 0.5) / 32768.0;
        g_q15[i] = (int16_t)floor(v * 32768.0 + 0.5);
        g_nat[i] = pcl_test_from_double(v);
    }
}

static pcl_sampler_sample_t make_sample(pcl_sampler_format_t fmt, pcl_sampler_loop_t mode,
    uint32_t frames, uint32_t loop_start, uint32_t loop_end)
{
    pcl_sampler_sample_t x;
    x.data = fmt == PCL_SAMPLER_Q15 ? (const void *)g_q15 : (const void *)g_nat;
    x.frames = frames;
    x.format = fmt;
    x.sample_rate_hz = FS;
    x.root_note = 60;
    x.loop_mode = mode;
    x.loop_start = loop_start;
    x.loop_end = loop_end;
    return x;
}

static pcl_sampler_config_t make_cfg(const pcl_sampler_sample_t *x)
{
    pcl_sampler_config_t cfg;
    cfg.sample_rate_hz = FS;
    cfg.sample = x;
    cfg.interp = PCL_SAMPLER_LINEAR;
    cfg.pitch_depth_oct = PCL_OCT(1.0);
    cfg.gain = PCL_COEF(1.0);
    cfg.velocity_sens = PCL_COEF(0.0);
    cfg.start_offset = 0;
    cfg.reverse = 0;
    cfg.declick_ms = 0;
    return cfg;
}

static void init_ok(pcl_sampler_t *s, const pcl_sampler_config_t *cfg)
{
    PCL_CHECK_EQ_INT(pcl_sampler_init(s, cfg), PCL_OK);
}

static void fill_noise(pcl_sample_t *buf, size_t n, uint32_t seed, double amplitude)
{
    size_t i;
    for (i = 0; i < n; i++)
        buf[i] = pcl_test_from_double(pcl_test_rand(&seed) * amplitude);
}

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

/* ---- validation ---- */

static void test_init_validation(void)
{
    pcl_sampler_sample_t x = make_sample(PCL_SAMPLER_Q15, PCL_SAMPLER_FORWARD, 1000, 100, 900);
    pcl_sampler_config_t cfg = make_cfg(&x);
    pcl_sampler_t        s, before;

    make_data(2);
    init_ok(&s, &cfg);
    before = s;
    PCL_CHECK_EQ_INT(pcl_sampler_init(NULL, &cfg), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_sampler_init(&s, NULL), PCL_ERR_NULL_POINTER);
    cfg.sample = NULL;
    PCL_CHECK_EQ_INT(pcl_sampler_init(&s, &cfg), PCL_ERR_NULL_POINTER);
    cfg.sample = &x;

#define BAD_SAMPLE(field, value)                                                                   \
    do                                                                                             \
    {                                                                                              \
        pcl_sampler_sample_t b = x;                                                                \
        cfg.sample = &b;                                                                           \
        b.field = value;                                                                           \
        PCL_CHECK_EQ_INT(pcl_sampler_init(&s, &cfg), PCL_ERR_OUT_OF_RANGE);                        \
        PCL_CHECK_EQ_INT(pcl_sampler_set_sample(&s, &b), PCL_ERR_OUT_OF_RANGE);                    \
        cfg.sample = &x;                                                                           \
    } while (0)
    BAD_SAMPLE(data, NULL);
    BAD_SAMPLE(frames, 0u);
    BAD_SAMPLE(format, PCL_SAMPLER_FORMAT_COUNT);
    BAD_SAMPLE(sample_rate_hz, 7999u);
    BAD_SAMPLE(sample_rate_hz, 768001u);
    BAD_SAMPLE(root_note, 128u);
    BAD_SAMPLE(loop_mode, PCL_SAMPLER_LOOP_COUNT);
    BAD_SAMPLE(loop_end, 1001u);
    BAD_SAMPLE(loop_start, 900u);
    BAD_SAMPLE(loop_start, 899u); /* a loop of one frame */
#undef BAD_SAMPLE

#define BAD(field, value)                                                                          \
    do                                                                                             \
    {                                                                                              \
        pcl_sampler_config_t b = cfg;                                                              \
        b.field = value;                                                                           \
        PCL_CHECK_EQ_INT(pcl_sampler_init(&s, &b), PCL_ERR_OUT_OF_RANGE);                          \
    } while (0)
    BAD(sample_rate_hz, 7999u);
    BAD(interp, PCL_SAMPLER_INTERP_COUNT);
    BAD(pitch_depth_oct, PCL_OCT(-0.5));
    BAD(gain, PCL_COEF(-0.1));
    BAD(velocity_sens, PCL_COEF(1.01));
    BAD(start_offset, 1000u);
    BAD(declick_ms, PCL_SAMPLER_MAX_DECLICK_MS + 1u);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    BAD(gain, 2.0f);
    BAD(gain, NAN);
    BAD(pitch_depth_oct, 8.0f);
    BAD(velocity_sens, NAN);
#endif
#undef BAD
    PCL_CHECK(memcmp(&s, &before, sizeof s) == 0);

    /* A one-shot ignores its loop points; a two-frame loop is the smallest. */
    x.loop_mode = PCL_SAMPLER_ONE_SHOT;
    x.loop_start = 5000u;
    x.loop_end = 1u;
    init_ok(&s, &cfg);
    x = make_sample(PCL_SAMPLER_NATIVE, PCL_SAMPLER_PING_PONG, 1000, 998, 1000);
    cfg.start_offset = 999u;
    cfg.declick_ms = PCL_SAMPLER_MAX_DECLICK_MS;
    init_ok(&s, &cfg);
}

static void test_setter_validation(void)
{
    pcl_sampler_sample_t x = make_sample(PCL_SAMPLER_Q15, PCL_SAMPLER_SUSTAIN, 1000, 10, 20);
    pcl_sampler_config_t cfg = make_cfg(&x);
    pcl_sampler_t        s, before;

    init_ok(&s, &cfg);
    before = s;
    PCL_CHECK_EQ_INT(pcl_sampler_set_sample(NULL, &x), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_sampler_set_sample(&s, NULL), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_sampler_set_interp(&s, PCL_SAMPLER_INTERP_COUNT), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_sampler_set_pitch_depth(&s, PCL_OCT(-1.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_sampler_set_gain(&s, PCL_COEF(-1.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_sampler_set_velocity_sens(&s, PCL_COEF(-0.5)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_sampler_set_start_offset(&s, 1000u), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_sampler_set_declick(&s, 21u), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_sampler_set_interp(NULL, PCL_SAMPLER_LINEAR), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_sampler_set_gain(NULL, PCL_COEF(1.0)), PCL_ERR_NULL_POINTER);
    PCL_CHECK(memcmp(&s, &before, sizeof s) == 0);
    PCL_CHECK(!pcl_sampler_is_playing(&s));
}

/* ---- exact playback ---- */

/* At the root note and the recorded rate the output is the data, sample for sample. */
static void test_exact_playback(void)
{
    int fmt, interp, rev;
    make_data(2);
    for (fmt = 0; fmt < 2; fmt++)
        for (interp = 0; interp < 2; interp++)
            for (rev = 0; rev < 2; rev++)
            {
                pcl_sampler_sample_t x
                    = make_sample((pcl_sampler_format_t)fmt, PCL_SAMPLER_ONE_SHOT, 1000, 0, 0);
                pcl_sampler_config_t cfg = make_cfg(&x);
                pcl_sampler_t        s;
                size_t               i;
                int                  same = 1;

                cfg.interp = (pcl_sampler_interp_t)interp;
                cfg.reverse = rev;
                cfg.start_offset = 10;
                init_ok(&s, &cfg);
                pcl_sampler_note_on(&s, 60, 100);
                pcl_sampler_process(&s, g_out, NULL, NULL, 0, 1200);
                for (i = 0; i < 990u; i++)
                {
                    const size_t k = rev ? 989u - i : 10u + i;
                    same &= g_out[i] == g_nat[k];
                }
                for (i = 990u; i < 1200u; i++)
                    same &= g_out[i] == 0;
                PCL_CHECK(same);
                PCL_CHECK(!pcl_sampler_is_playing(&s));
            }
}

/* ---- pitch ---- */

static double play_hz(pcl_sampler_sample_t *x, uint8_t note, const pcl_sample_t *mod)
{
    pcl_sampler_config_t cfg = make_cfg(x);
    pcl_sampler_t        s;

    cfg.interp = PCL_SAMPLER_HERMITE;
    init_ok(&s, &cfg);
    pcl_sampler_note_on(&s, note, 127);
    pcl_sampler_process(&s, g_out, mod, NULL, 0, FS);
    return crossing_hz(g_out, 1000, FS);
}

static void test_pitch(void)
{
    pcl_sampler_sample_t x = make_sample(PCL_SAMPLER_NATIVE, PCL_SAMPLER_FORWARD, DATA_FRAMES, 0,
        DATA_FRAMES); /* 375 whole cycles */
    static const int     semis[] = { 0, 12, 7, -12, 1, -5, 24 };
    size_t               k, i;
    double               hz;

    make_data(0);
    for (k = 0; k < sizeof semis / sizeof semis[0]; k++)
    {
        const double want = SINE_HZ * pow(2.0, (double)semis[k] / 12.0);
        hz = play_hz(&x, (uint8_t)(60 + semis[k]), NULL);
        PCL_CHECK_NEAR(hz / want, 1.0, 2e-6);
    }
    /* A sample recorded at 24 kHz plays an octave down at its root. */
    x.sample_rate_hz = 24000u;
    hz = play_hz(&x, 60, NULL);
    PCL_CHECK_NEAR(hz / (SINE_HZ / 2.0), 1.0, 2e-6);
    x.sample_rate_hz = FS;
    /* pitch_mod 0.5 at depth 1 octave: half an octave up. */
    for (i = 0; i < FS; i++)
        g_mod[i] = pcl_test_from_double(0.5);
    hz = play_hz(&x, 60, g_mod);
    PCL_CHECK_NEAR(hz / (SINE_HZ * sqrt(2.0)), 1.0, 3e-5);
    printf("    pitch: +7 semitones %.4f Hz, mod +0.5 oct %.4f Hz\n", play_hz(&x, 67, NULL),
        play_hz(&x, 60, g_mod));
}

/* ---- loops ---- */

/* A one-cycle loop (128 frames) played 5 semitones up for 3 s: wraps keep the fraction, no drift.
 */
static void test_forward_loop_no_drift(void)
{
    pcl_sampler_sample_t x = make_sample(PCL_SAMPLER_NATIVE, PCL_SAMPLER_FORWARD, 1024, 512, 640);
    pcl_sampler_config_t cfg = make_cfg(&x);
    pcl_sampler_t        s;
    const double         want = SINE_HZ * pow(2.0, 5.0 / 12.0);
    double               hz;

    make_data(0);
    cfg.interp = PCL_SAMPLER_HERMITE;
    init_ok(&s, &cfg);
    pcl_sampler_note_on(&s, 65, 127);
    pcl_sampler_process(&s, g_out, NULL, NULL, 0, MAX_FRAMES);
    hz = crossing_hz(g_out, 2000, MAX_FRAMES);
    printf("    one-cycle loop, +5 semitones, 3 s (%.0f loops): %.5f Hz (want %.5f)\n", 3.0 * want,
        hz, want);
    PCL_CHECK_NEAR(hz / want, 1.0, 2e-6);
    PCL_CHECK(pcl_sampler_is_playing(&s));
}

/* Step 1 over a ramp: the exact frame sequence of each loop mode. */
static void test_loop_sequences(void)
{
    pcl_sampler_sample_t x;
    pcl_sampler_config_t cfg;
    pcl_sampler_t        s;
    size_t               i;
    int                  ok;

    make_data(1);

    /* Ping-pong over frames 10..19: 0..19, 18..10, 11..19, 18.. */
    x = make_sample(PCL_SAMPLER_NATIVE, PCL_SAMPLER_PING_PONG, 100, 10, 20);
    cfg = make_cfg(&x);
    init_ok(&s, &cfg);
    pcl_sampler_note_on(&s, 60, 127);
    pcl_sampler_process(&s, g_out, NULL, NULL, 0, 60);
    ok = 1;
    for (i = 0; i < 60u; i++)
    {
        size_t k;
        if (i <= 19u)
            k = i;
        else
        {
            const size_t t = (i - 19u) % 18u; /* steps since the first turn, period 18 */
            k = t <= 9u ? 19u - t : 10u + (t - 9u);
        }
        ok &= g_out[i] == g_nat[k];
    }
    PCL_CHECK(ok);

    /* Forward, reversed, from the end: 99..20, then 19..10 forever (wrapping down). */
    x = make_sample(PCL_SAMPLER_NATIVE, PCL_SAMPLER_FORWARD, 100, 10, 20);
    cfg = make_cfg(&x);
    cfg.reverse = 1;
    init_ok(&s, &cfg);
    pcl_sampler_note_on(&s, 60, 127);
    pcl_sampler_process(&s, g_out, NULL, NULL, 0, 130);
    ok = 1;
    for (i = 0; i < 130u; i++)
    {
        const size_t k = i < 90u ? 99u - i : 19u - (i - 90u) % 10u;
        ok &= g_out[i] == g_nat[k];
    }
    PCL_CHECK(ok);

    /* Sustain: loops 10..19 while held, then NOTE_OFF plays on through 20..99 and stops. */
    x = make_sample(PCL_SAMPLER_NATIVE, PCL_SAMPLER_SUSTAIN, 100, 10, 20);
    cfg = make_cfg(&x);
    init_ok(&s, &cfg);
    pcl_sampler_note_on(&s, 60, 127);
    pcl_sampler_process(&s, g_out, NULL, NULL, 0, 45); /* 0..19, 10..19, 10..19, 10..14 */
    pcl_sampler_note_off(&s);
    pcl_sampler_process(&s, g_out + 45, NULL, NULL, 0, 100);
    ok = 1;
    for (i = 0; i < 145u; i++)
    {
        size_t k;
        if (i < 20u)
            k = i;
        else if (i < 45u)
            k = 10u + (i - 20u) % 10u;
        else
            k = 15u + (i - 45u); /* 15 .. 99 */
        if (k > 99u)
            ok &= g_out[i] == 0;
        else
            ok &= g_out[i] == g_nat[k];
    }
    PCL_CHECK(ok);
    PCL_CHECK(!pcl_sampler_is_playing(&s));

    /* Forward: NOTE_OFF fades out over the declick and stops. */
    x = make_sample(PCL_SAMPLER_NATIVE, PCL_SAMPLER_FORWARD, 100, 10, 20);
    cfg = make_cfg(&x);
    cfg.declick_ms = 1; /* 48 samples */
    init_ok(&s, &cfg);
    pcl_sampler_note_on(&s, 60, 127);
    pcl_sampler_process(&s, g_out, NULL, NULL, 0, 200);
    pcl_sampler_note_off(&s);
    pcl_sampler_process(&s, g_out, NULL, NULL, 0, 47);
    PCL_CHECK(pcl_sampler_is_playing(&s));
    pcl_sampler_process(&s, g_out, NULL, NULL, 0, 1);
    PCL_CHECK(!pcl_sampler_is_playing(&s));
}

/* ---- events ---- */

static void test_events_are_sample_accurate(void)
{
    pcl_sampler_sample_t x = make_sample(PCL_SAMPLER_NATIVE, PCL_SAMPLER_FORWARD, 1000, 0, 1000);
    pcl_sampler_config_t cfg = make_cfg(&x);
    pcl_sampler_t        s;
    pcl_event_t          ev[2];
    size_t               i;
    int                  ok = 1;

    make_data(2);
    cfg.velocity_sens = PCL_COEF(1.0);
    init_ok(&s, &cfg);
    ev[0].offset = 37;
    ev[0].type = PCL_EVT_NOTE_ON;
    ev[0].note = 60;
    ev[0].velocity = 127;
    ev[1].offset = 200; /* beyond this block: ignored */
    ev[1].type = PCL_EVT_NOTE_OFF;
    ev[1].note = 60;
    ev[1].velocity = 0;
    pcl_sampler_process(&s, g_out, NULL, ev, 2, 64);
    for (i = 0; i < 64u; i++)
        ok &= g_out[i] == (i < 37u ? 0 : g_nat[i - 37u]);
    PCL_CHECK(ok);
    PCL_CHECK(pcl_sampler_is_playing(&s));

    /* NOTE_OFF at offset 5 (declick 0): silent from that sample on. */
    ev[0].offset = 5;
    ev[0].type = PCL_EVT_NOTE_OFF;
    pcl_sampler_process(&s, g_out, NULL, ev, 1, 16);
    ok = 1;
    for (i = 0; i < 16u; i++)
        ok &= i < 5u ? g_out[i] == g_nat[27u + i] : g_out[i] == 0;
    PCL_CHECK(ok);

    /* Velocity 64 at sensitivity 1: gain 64/127. A NOTE_ON with velocity 0 is a NOTE_OFF. */
    ev[0].offset = 0;
    ev[0].type = PCL_EVT_NOTE_ON;
    ev[0].velocity = 64;
    pcl_sampler_process(&s, g_out, NULL, ev, 1, 100);
    for (i = 0; i < 100u; i++)
        PCL_CHECK_NEAR(pcl_test_to_double(g_out[i]), pcl_test_to_double(g_nat[i]) * 64.0 / 127.0,
            2.0 * PCL_TEST_SAMPLE_TOL);
    ev[0].velocity = 0;
    pcl_sampler_process(&s, g_out, NULL, ev, 1, 10);
    PCL_CHECK(!pcl_sampler_is_playing(&s));
}

/* ---- declick ---- */

/* Retriggers mid-sound, stops and one-shot ends: no step larger than the signal's own. */
static void test_declick(void)
{
    pcl_sampler_sample_t x = make_sample(PCL_SAMPLER_NATIVE, PCL_SAMPLER_ONE_SHOT, 4000, 0, 0);
    pcl_sampler_config_t cfg = make_cfg(&x);
    pcl_sampler_t        s;
    pcl_event_t          ev[3];
    size_t               i;
    double               worst = 0.0, own = 0.0;

    make_data(0);
    for (i = 1; i < 4000u; i++)
    {
        const double d = fabs(pcl_test_to_double(g_nat[i]) - pcl_test_to_double(g_nat[i - 1u]));
        if (d > own)
            own = d;
    }
    cfg.declick_ms = 2;
    init_ok(&s, &cfg);
    memset(ev, 0, sizeof ev);
    ev[0].offset = 0;
    ev[0].type = PCL_EVT_NOTE_ON;
    ev[0].note = 60;
    ev[0].velocity = 127;
    ev[1] = ev[0];
    ev[1].offset = 1000; /* retrigger at a large value of the sine */
    ev[1].note = 64;
    ev[2] = ev[0];
    ev[2].offset = 2000;
    ev[2].note = 55;
    for (i = 0; i < 5000u; i++)
        g_out[i] = 0;
    pcl_sampler_process(&s, g_out, NULL, ev, 3, 5000);
    for (i = 1; i < 5000u; i++)
    {
        const double d = fabs(pcl_test_to_double(g_out[i]) - pcl_test_to_double(g_out[i - 1u]));
        if (d > worst)
            worst = d;
    }
    printf("    declick: largest step %.4f (the sine's own %.4f)\n", worst, own);
    PCL_CHECK(worst <= 1.6 * own); /* up to two overlapping notes, at different pitches */
    /* The last note (a fourth down, step 0.749) lasts 4000 / 0.749 = 5340 samples from 2000, plus
       the 96-sample declick: still playing at 7300, silent by 7500. */
    pcl_sampler_process(&s, g_out, NULL, NULL, 0, 2300);
    PCL_CHECK(pcl_sampler_is_playing(&s));
    pcl_sampler_process(&s, g_out, NULL, NULL, 0, 200);
    PCL_CHECK(!pcl_sampler_is_playing(&s));
}

/* ---- determinism, blocking, formats ---- */

static void render_blocked(size_t block, pcl_sample_t *out)
{
    static const size_t  note_at[] = { 10, 3000, 3001, 9000, 15000 };
    static const uint8_t notes[] = { 60, 67, 55, 72, 48 };
    pcl_sampler_sample_t x
        = make_sample(PCL_SAMPLER_Q15, PCL_SAMPLER_SUSTAIN, DATA_FRAMES, 1000, 5000);
    pcl_sampler_config_t cfg = make_cfg(&x);
    pcl_sampler_t        s;
    const size_t         total = 24000;
    size_t               pos = 0;

    cfg.interp = PCL_SAMPLER_HERMITE;
    cfg.declick_ms = 3;
    cfg.velocity_sens = PCL_COEF(0.5);
    cfg.pitch_depth_oct = PCL_OCT(0.5);
    init_ok(&s, &cfg);
    while (pos < total)
    {
        const size_t n = total - pos < block ? total - pos : block;
        pcl_event_t  ev[16]; /* up to 10 events in one block */
        size_t       k, cnt = 0;
        for (k = 0; k < 5u; k++)
        {
            if (note_at[k] >= pos && note_at[k] < pos + n)
            {
                ev[cnt].offset = (uint16_t)(note_at[k] - pos);
                ev[cnt].type = PCL_EVT_NOTE_ON;
                ev[cnt].note = notes[k];
                ev[cnt].velocity = (uint8_t)(40 + 20 * k);
                cnt++;
            }
            if (note_at[k] + 1500u >= pos && note_at[k] + 1500u < pos + n)
            {
                ev[cnt].offset = (uint16_t)(note_at[k] + 1500u - pos);
                ev[cnt].type = PCL_EVT_NOTE_OFF;
                ev[cnt].note = notes[k];
                ev[cnt].velocity = 0;
                cnt++;
            }
        }
        /* Sort by offset (the list is short). */
        for (k = 1; k < cnt; k++)
        {
            size_t j = k;
            while (j > 0u && ev[j - 1u].offset > ev[j].offset)
            {
                const pcl_event_t t = ev[j];
                ev[j] = ev[j - 1u];
                ev[j - 1u] = t;
                j--;
            }
        }
        pcl_sampler_process(&s, out + pos, g_mod + pos, ev, cnt, n);
        pos += n;
    }
}

static void test_block_size_independence(void)
{
    static const size_t blocks[] = { 1, 7, 16, 17, 256 };
    size_t              b, i;

    make_data(0);
    fill_noise(g_mod, 24000, 3u, 1.0);
    render_blocked(24000, g_out);
    for (b = 0; b < sizeof blocks / sizeof blocks[0]; b++)
    {
        int same = 1;
        render_blocked(blocks[b], g_out2);
        for (i = 0; i < 24000u; i++)
            same &= g_out[i] == g_out2[i];
        PCL_CHECK(same);
    }
}

static void test_reset_and_zero_mod(void)
{
    pcl_sampler_sample_t x
        = make_sample(PCL_SAMPLER_NATIVE, PCL_SAMPLER_PING_PONG, DATA_FRAMES, 100, 7000);
    pcl_sampler_config_t cfg = make_cfg(&x);
    pcl_sampler_t        s;
    size_t               i;
    int                  same = 1;
    const size_t         n = 20000;

    make_data(2);
    cfg.interp = PCL_SAMPLER_HERMITE;
    init_ok(&s, &cfg);
    fill_noise(g_mod, n, 4u, 1.0);
    pcl_sampler_note_on(&s, 63, 100);
    pcl_sampler_process(&s, g_out, g_mod, NULL, 0, n);
    pcl_sampler_reset(&s);
    PCL_CHECK(!pcl_sampler_is_playing(&s));
    pcl_sampler_note_on(&s, 63, 100);
    pcl_sampler_process(&s, g_out2, g_mod, NULL, 0, n);
    for (i = 0; i < n; i++)
        same &= g_out[i] == g_out2[i];
    PCL_CHECK(same);

    memset(g_mod, 0, n * sizeof g_mod[0]);
    pcl_sampler_reset(&s);
    pcl_sampler_note_on(&s, 63, 100);
    pcl_sampler_process(&s, g_out, g_mod, NULL, 0, n);
    pcl_sampler_reset(&s);
    pcl_sampler_note_on(&s, 63, 100);
    pcl_sampler_process(&s, g_out2, NULL, NULL, 0, n);
    same = 1;
    for (i = 0; i < n; i++)
        same &= g_out[i] == g_out2[i];
    PCL_CHECK(same);

    /* pitch_mod may alias out. */
    fill_noise(g_mod, n, 5u, 1.0);
    pcl_sampler_reset(&s);
    pcl_sampler_note_on(&s, 63, 100);
    pcl_sampler_process(&s, g_out, g_mod, NULL, 0, n);
    memcpy(g_out2, g_mod, n * sizeof g_mod[0]);
    pcl_sampler_reset(&s);
    pcl_sampler_note_on(&s, 63, 100);
    pcl_sampler_process(&s, g_out2, g_out2, NULL, 0, n);
    same = 1;
    for (i = 0; i < n; i++)
        same &= g_out[i] == g_out2[i];
    PCL_CHECK(same);
}

/* Q15 and native data with the same values play identically (the values are Q15-exact). */
static void test_formats_agree(void)
{
    int interp;
    make_data(2);
    for (interp = 0; interp < 2; interp++)
    {
        pcl_sampler_sample_t xq
            = make_sample(PCL_SAMPLER_Q15, PCL_SAMPLER_FORWARD, DATA_FRAMES, 200, 30000);
        pcl_sampler_sample_t xn
            = make_sample(PCL_SAMPLER_NATIVE, PCL_SAMPLER_FORWARD, DATA_FRAMES, 200, 30000);
        pcl_sampler_config_t cfg = make_cfg(&xq);
        pcl_sampler_t        a, b;
        size_t               i;
        int                  same = 1;

        cfg.interp = (pcl_sampler_interp_t)interp;
        init_ok(&a, &cfg);
        cfg.sample = &xn;
        init_ok(&b, &cfg);
        pcl_sampler_note_on(&a, 66, 90);
        pcl_sampler_note_on(&b, 66, 90);
        pcl_sampler_process(&a, g_out, NULL, NULL, 0, FS);
        pcl_sampler_process(&b, g_out2, NULL, NULL, 0, FS);
        for (i = 0; i < FS; i++)
            same &= g_out[i] == g_out2[i];
        PCL_CHECK(same);
    }
}

/* Linear and Hermite against a double model at the sampler's own (integer) positions. */
static void test_matches_double_model(void)
{
    int interp;
    make_data(0);
    for (interp = 0; interp < 2; interp++)
    {
        pcl_sampler_sample_t x
            = make_sample(PCL_SAMPLER_NATIVE, PCL_SAMPLER_FORWARD, DATA_FRAMES, 0, DATA_FRAMES);
        pcl_sampler_config_t cfg = make_cfg(&x);
        pcl_sampler_t        s;
        size_t               i;
        int64_t              pos = 0;
        uint32_t             step;
        double               worst = 0.0;

        cfg.interp = (pcl_sampler_interp_t)interp;
        init_ok(&s, &cfg);
        pcl_sampler_note_on(&s, 63, 127); /* 3 semitones up: a non-integer step */
        step = s.head[0].step;            /* white-box: the exact step */
        pcl_sampler_process(&s, g_out, NULL, NULL, 0, 20000);
        for (i = 0; i < 20000u; i++)
        {
            const int64_t idx = pos >> 24;
            const double  t = (double)(pos & 0xFFFFFF) / 16777216.0;
            const double  p0 = pcl_test_to_double(g_nat[idx > 0 ? idx - 1 : 0]);
            const double  p1 = pcl_test_to_double(g_nat[idx]);
            const double  p2 = pcl_test_to_double(g_nat[idx + 1]);
            const double  p3 = pcl_test_to_double(g_nat[idx + 2]);
            double        want;
            if (interp == 0)
                want = p1 + t * (p2 - p1);
            else
                want = p1
                    + 0.5 * t
                        * ((p2 - p0)
                            + t
                                * ((2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3)
                                    + t * ((p3 - p0) + 3.0 * (p1 - p2))));
            if (i > 0u && fabs(pcl_test_to_double(g_out[i]) - want) > worst)
                worst = fabs(pcl_test_to_double(g_out[i]) - want);
            pos += step;
        }
        printf("    %s vs double model: worst %.3g\n", interp ? "hermite" : "linear ", worst);
        /* 16-bit fraction: up to 2^-17 of a frame of position error, times the slope (< 0.03). */
#if PCL_SAMPLE_TYPE == PCL_Q15
        PCL_CHECK(worst < 4.0 / 32768.0);
#else
        PCL_CHECK(worst < 1e-6);
#endif
    }
}

/* Extreme pitches and long sustains stay finite (no denormals in float). */
static void test_extremes(void)
{
    pcl_sampler_sample_t x
        = make_sample(PCL_SAMPLER_NATIVE, PCL_SAMPLER_SUSTAIN, DATA_FRAMES, 10, DATA_FRAMES - 10u);
    pcl_sampler_config_t cfg = make_cfg(&x);
    pcl_sampler_t        s;
    size_t               i;
    int                  ok = 1;

    make_data(2);
    x.root_note = 0;
    x.sample_rate_hz = 768000u;
    cfg.interp = PCL_SAMPLER_HERMITE;
    cfg.pitch_depth_oct = PCL_OCT(7.9);
    cfg.gain = PCL_COEF(1.99);
    init_ok(&s, &cfg);
    pcl_sampler_note_on(&s, 127, 127); /* far beyond 256 frames per sample: clamped */
    PCL_CHECK_EQ_INT(s.head[0].step, 0xFFFFFFFFu);
    fill_noise(g_mod, MAX_FRAMES, 6u, 1.0);
    pcl_sampler_process(&s, g_out, g_mod, NULL, 0, MAX_FRAMES);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    for (i = 0; i < MAX_FRAMES; i++)
        ok &= isfinite(g_out[i]) && (g_out[i] == 0.0f || fabsf(g_out[i]) >= FLT_MIN);
#endif
    PCL_CHECK(ok);
    x.root_note = 127;
    x.sample_rate_hz = 8000u;
    pcl_sampler_note_on(&s, 0, 127); /* the slowest step */
    PCL_CHECK(s.head[0].step > 0u);
    pcl_sampler_process(&s, g_out, g_mod, NULL, 0, MAX_FRAMES);
    PCL_CHECK(pcl_sampler_is_playing(&s));
    (void)i;
}

int main(void)
{
    PCL_RUN_TEST(test_init_validation);
    PCL_RUN_TEST(test_setter_validation);
    PCL_RUN_TEST(test_exact_playback);
    PCL_RUN_TEST(test_pitch);
    PCL_RUN_TEST(test_forward_loop_no_drift);
    PCL_RUN_TEST(test_loop_sequences);
    PCL_RUN_TEST(test_events_are_sample_accurate);
    PCL_RUN_TEST(test_declick);
    PCL_RUN_TEST(test_block_size_independence);
    PCL_RUN_TEST(test_reset_and_zero_mod);
    PCL_RUN_TEST(test_formats_agree);
    PCL_RUN_TEST(test_matches_double_model);
    PCL_RUN_TEST(test_extremes);
    return pcl_test_summary("sampler");
}
