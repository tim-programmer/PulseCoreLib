/*
 * pcl_clock tests: pulse positions against exact rational arithmetic, long-run drift, block-size
 * independence, the transport, tempo changes, the output buffer limits.
 */
#include <stdlib.h>
#include <string.h>

#include "pcl_test.h"

#if PCL_SAMPLE_TYPE == PCL_FLOAT
#define BPM_OF(x) ((float)(x))
#define BPM_RAW(x)                                                                                 \
    ((uint32_t)((float)(x) * 65536.0f + 0.5f)) /* as the library rounds a float bpm */
#else
#define BPM_OF(x) ((uint32_t)((x) * 65536.0 + 0.5))
#define BPM_RAW(x) ((uint32_t)((x) * 65536.0 + 0.5))
#endif

typedef struct
{
    uint64_t *t; /* sample index of each CLOCK pulse, from the start of the run */
    size_t    n;
    size_t    cap;
} pulses_t;

static pcl_clock_config_t make_cfg(uint32_t fs, double bpm, uint32_t ppqn, int running)
{
    pcl_clock_config_t c;

    c.sample_rate_hz = fs;
    c.bpm = BPM_OF(bpm);
    c.ppqn = ppqn;
    c.running = running;
    return c;
}

/* Runs `total` samples in blocks of `blk`, collecting the global sample index of every pulse. */
static void run(pcl_clock_t *clk, uint64_t total, size_t blk, pulses_t *p)
{
    pcl_event_t *ev
        = (pcl_event_t *)malloc(sizeof(pcl_event_t) * (pcl_clock_max_events(clk, blk) + 4));
    uint64_t base = 0;

    if (ev == NULL)
        exit(2);
    while (base < total)
    {
        const size_t frames = (total - base < blk) ? (size_t)(total - base) : blk;
        const size_t cap = pcl_clock_max_events(clk, frames);
        const size_t n = pcl_clock_process(clk, frames, ev, cap);
        size_t       i;

        for (i = 0; i < n; i++)
            if (ev[i].type == PCL_EVT_CLOCK)
            {
                PCL_CHECK(ev[i].offset < frames);
                if (p->n < p->cap)
                    p->t[p->n] = base + ev[i].offset;
                p->n++;
            }
        base += frames;
    }
    free(ev);
}

static void test_init_and_setters(void)
{
    pcl_clock_t        c;
    pcl_clock_config_t cfg = make_cfg(48000u, 120.0, 24, 0);

    PCL_CHECK_EQ_INT(pcl_clock_init(&c, &cfg), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_clock_init(NULL, &cfg), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_clock_init(&c, NULL), PCL_ERR_NULL_POINTER);
    {
        pcl_clock_t keep = c;

        cfg = make_cfg(7999u, 120.0, 24, 0);
        PCL_CHECK_EQ_INT(pcl_clock_init(&c, &cfg), PCL_ERR_OUT_OF_RANGE);
        cfg = make_cfg(768001u, 120.0, 24, 0);
        PCL_CHECK_EQ_INT(pcl_clock_init(&c, &cfg), PCL_ERR_OUT_OF_RANGE);
        cfg = make_cfg(48000u, 9.0, 24, 0);
        PCL_CHECK_EQ_INT(pcl_clock_init(&c, &cfg), PCL_ERR_OUT_OF_RANGE);
        cfg = make_cfg(48000u, 1000.0, 24, 0);
        PCL_CHECK_EQ_INT(pcl_clock_init(&c, &cfg), PCL_ERR_OUT_OF_RANGE);
        cfg = make_cfg(48000u, 120.0, 0, 0);
        PCL_CHECK_EQ_INT(pcl_clock_init(&c, &cfg), PCL_ERR_OUT_OF_RANGE);
        cfg = make_cfg(48000u, 120.0, 97, 0);
        PCL_CHECK_EQ_INT(pcl_clock_init(&c, &cfg), PCL_ERR_OUT_OF_RANGE);
        PCL_CHECK(memcmp(&c, &keep, sizeof c) == 0);
    }
    /* the limits are accepted */
    cfg = make_cfg(48000u, 10.0, 1, 0);
    PCL_CHECK_EQ_INT(pcl_clock_init(&c, &cfg), PCL_OK);
    cfg = make_cfg(8000u, 999.0, 96, 0);
    PCL_CHECK_EQ_INT(pcl_clock_init(&c, &cfg), PCL_OK);

    cfg = make_cfg(8000u, 120.0, 24, 0);
    PCL_CHECK_EQ_INT(pcl_clock_init(&c, &cfg), PCL_OK);
    cfg = make_cfg(768000u, 10.0, 1, 0);
    PCL_CHECK_EQ_INT(pcl_clock_init(&c, &cfg), PCL_OK);

    cfg = make_cfg(48000u, 120.0, 24, 0);
    PCL_CHECK_EQ_INT(pcl_clock_init(&c, &cfg), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_clock_set_bpm(&c, BPM_OF(10.0)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_clock_set_bpm(&c, BPM_OF(999.0)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_clock_set_ppqn(&c, 1), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_clock_set_ppqn(&c, 96), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_clock_init(&c, &cfg), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_clock_set_bpm(NULL, BPM_OF(100.0)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_clock_set_bpm(&c, BPM_OF(9.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_clock_set_bpm(&c, BPM_OF(1000.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_clock_set_bpm(&c, BPM_OF(200.0)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_clock_set_ppqn(NULL, 24), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_clock_set_ppqn(&c, 0), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_clock_set_ppqn(&c, 97), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_clock_set_ppqn(&c, 48), PCL_OK);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    cfg = make_cfg(48000u, 120.0, 24, 0);
    cfg.bpm = NAN;
    PCL_CHECK_EQ_INT(pcl_clock_init(&c, &cfg), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_clock_set_bpm(&c, NAN), PCL_ERR_OUT_OF_RANGE);
#endif
}

/* Pulse k is at floor(k * fs * 60 / (bpm * ppqn)), up to the 2^-32 sample the period is rounded to.
 */
static void pulse_positions_case(uint32_t fs, double bpm, uint32_t ppqn, uint64_t seconds)
{
    enum
    {
        MAXP = 200000
    };
    pcl_clock_t              c;
    const pcl_clock_config_t cfg = make_cfg(fs, bpm, ppqn, 1);
    pulses_t                 p;
    size_t                   k;
    const long double        ideal
        = (long double)fs * 60.0L * 65536.0L / ((long double)BPM_RAW(bpm) * (long double)ppqn);

    p.t = (uint64_t *)calloc(MAXP, sizeof(uint64_t));
    p.n = 0;
    p.cap = MAXP;
    if (p.t == NULL)
        exit(2);
    PCL_CHECK_EQ_INT(pcl_clock_init(&c, &cfg), PCL_OK);
    run(&c, seconds * fs, 256, &p);
    PCL_CHECK(p.n > 10);
    for (k = 0; k < p.n && k < p.cap; k++)
    {
        const long double want = (long double)k * ideal;
        const long double err = want - (long double)p.t[k];

        /* floor(want) up to a hair: the period is truncated to 2^-32, so k pulses lose k * 2^-32 */
        if (!(err >= -1e-9L && err < 1.0L + 1e-3L))
        {
            printf("    fs %u bpm %g ppqn %u: pulse %zu at %llu, ideal %.6Lf\n", fs, bpm, ppqn, k,
                (unsigned long long)p.t[k], want);
            PCL_CHECK(0);
            break;
        }
    }
    PCL_CHECK_EQ_INT(p.t[0], 0); /* the first pulse is at once */
    free(p.t);
}

static void test_pulse_positions(void)
{
    pulse_positions_case(48000u, 120.0, 24, 60);
    pulse_positions_case(44100u, 120.0, 24, 60);
    pulse_positions_case(44100u, 133.3, 24, 60);
    pulse_positions_case(48000u, 97.123, 96, 30);
    pulse_positions_case(96000u, 301.0, 12, 30);
    pulse_positions_case(8000u, 999.0, 96, 20);
    pulse_positions_case(192000u, 10.0, 1, 600);
    pulse_positions_case(44100u, 123.456, 7, 60);
}

/* A tempo that divides the sample rate gives exactly even spacing. */
static void test_exact_spacing(void)
{
    pcl_clock_t              c;
    const pcl_clock_config_t cfg = make_cfg(48000u, 120.0, 24, 1);
    uint64_t                 t[64];
    pulses_t                 p;
    size_t                   k;

    p.t = t;
    p.n = 0;
    p.cap = 64;
    PCL_CHECK_EQ_INT(pcl_clock_init(&c, &cfg), PCL_OK);
    run(&c, 64000, 256, &p);
    PCL_CHECK_EQ_INT(p.n, 64);
    for (k = 0; k < 64; k++)
        PCL_CHECK_EQ_INT(t[k], 1000 * k); /* 48000 * 60 / (120 * 24) = 1000 samples */
    /* 44.1 kHz: 918.75 samples is exact in Q32.32 */
    {
        const pcl_clock_config_t cfg2 = make_cfg(44100u, 120.0, 24, 1);

        p.n = 0;
        PCL_CHECK_EQ_INT(pcl_clock_init(&c, &cfg2), PCL_OK);
        run(&c, 60000, 256, &p);
        for (k = 0; k < 64 && k < p.n; k++)
            PCL_CHECK_EQ_INT(t[k], (k * 3675) / 4);
    }
}

/* An hour at 44.1 kHz: the pulse count is exact, no drift. */
static void test_long_run_does_not_drift(void)
{
    pcl_clock_t              c;
    const pcl_clock_config_t cfg = make_cfg(44100u, 120.0, 24, 1);
    pulses_t                 p;
    uint64_t                 last[1] = { 0 };
    const uint64_t           samples = 3600ull * 44100ull;

    p.t = last;
    p.n = 0;
    p.cap = 0; /* count only */
    PCL_CHECK_EQ_INT(pcl_clock_init(&c, &cfg), PCL_OK);
    run(&c, samples, 4096, &p);
    /* pulses at 0, 918.75, ...: floor((samples - 1) / 918.75) + 1 of them fit */
    PCL_CHECK_EQ_INT(p.n, ((samples - 1) * 4) / 3675 + 1);
}

static void test_block_size_independence(void)
{
    static const size_t blocks[] = { 1, 7, 64, 255, 1000, 65536 };
    enum
    {
        MAXP = 5000
    };
    uint64_t                *ref = (uint64_t *)calloc(MAXP, sizeof(uint64_t));
    uint64_t                *got = (uint64_t *)calloc(MAXP, sizeof(uint64_t));
    size_t                   bi;
    pulses_t                 pr, pg;
    pcl_clock_t              c;
    const pcl_clock_config_t cfg = make_cfg(44100u, 127.7, 24, 1);

    if (ref == NULL || got == NULL)
        exit(2);
    pr.t = ref, pr.n = 0, pr.cap = MAXP;
    PCL_CHECK_EQ_INT(pcl_clock_init(&c, &cfg), PCL_OK);
    run(&c, 131072, 131072 > 65536 ? 65536 : 131072, &pr);
    PCL_CHECK(pr.n > 50);
    for (bi = 0; bi < sizeof blocks / sizeof blocks[0]; bi++)
    {
        pg.t = got, pg.n = 0, pg.cap = MAXP;
        memset(got, 0, MAXP * sizeof(uint64_t));
        PCL_CHECK_EQ_INT(pcl_clock_init(&c, &cfg), PCL_OK);
        run(&c, 131072, blocks[bi], &pg);
        PCL_CHECK_EQ_INT(pg.n, pr.n);
        PCL_CHECK(memcmp(ref, got, pr.n * sizeof(uint64_t)) == 0);
    }
    free(ref);
    free(got);
}

static void test_transport(void)
{
    pcl_clock_t              c;
    const pcl_clock_config_t cfg = make_cfg(48000u, 120.0, 24, 0);
    pcl_event_t              ev[64];
    size_t                   n;

    PCL_CHECK_EQ_INT(pcl_clock_init(&c, &cfg), PCL_OK);
    PCL_CHECK(!pcl_clock_is_running(&c));
    /* stopped: nothing, however long */
    PCL_CHECK_EQ_INT(pcl_clock_process(&c, 10000, ev, 64), 0);

    /* start: START, then a pulse, both at offset 0 */
    pcl_clock_start(&c);
    n = pcl_clock_process(&c, 100, ev, 64);
    PCL_CHECK_EQ_INT(n, 2);
    PCL_CHECK(ev[0].type == PCL_EVT_START && ev[0].offset == 0);
    PCL_CHECK(ev[1].type == PCL_EVT_CLOCK && ev[1].offset == 0);
    PCL_CHECK(pcl_clock_is_running(&c));
    /* the next pulse is a period after the start: sample 1000 of the run = block 10 */
    n = pcl_clock_process(&c, 900, ev, 64);
    PCL_CHECK_EQ_INT(n, 0);
    n = pcl_clock_process(&c, 100, ev, 64);
    PCL_CHECK_EQ_INT(n, 1);
    PCL_CHECK(ev[0].type == PCL_EVT_CLOCK && ev[0].offset == 0);

    /* start while running restarts the phase: START and a pulse at once */
    n = pcl_clock_process(&c, 500, ev, 64);
    PCL_CHECK_EQ_INT(n, 0);
    pcl_clock_start(&c);
    n = pcl_clock_process(&c, 100, ev, 64);
    PCL_CHECK_EQ_INT(n, 2);
    PCL_CHECK(ev[0].type == PCL_EVT_START && ev[1].type == PCL_EVT_CLOCK);

    /* stop: STOP at offset 0, then silence */
    pcl_clock_stop(&c);
    n = pcl_clock_process(&c, 5000, ev, 64);
    PCL_CHECK_EQ_INT(n, 1);
    PCL_CHECK(ev[0].type == PCL_EVT_STOP && ev[0].offset == 0);
    PCL_CHECK(!pcl_clock_is_running(&c));
    PCL_CHECK_EQ_INT(pcl_clock_process(&c, 5000, ev, 64), 0);
    /* stop on a stopped clock says nothing */
    pcl_clock_stop(&c);
    PCL_CHECK_EQ_INT(pcl_clock_process(&c, 100, ev, 64), 0);
    /* start then stop before processing: nothing happens at all */
    pcl_clock_start(&c);
    pcl_clock_stop(&c);
    PCL_CHECK_EQ_INT(pcl_clock_process(&c, 100, ev, 64), 0);
    /* stop then start before processing: a restart */
    pcl_clock_start(&c);
    (void)pcl_clock_process(&c, 100, ev, 64);
    pcl_clock_stop(&c);
    pcl_clock_start(&c);
    n = pcl_clock_process(&c, 100, ev, 64);
    PCL_CHECK(n == 2 && ev[0].type == PCL_EVT_START && ev[1].type == PCL_EVT_CLOCK);

    /* reset stops and clears pending requests */
    pcl_clock_start(&c);
    pcl_clock_reset(&c);
    PCL_CHECK(!pcl_clock_is_running(&c));
    PCL_CHECK_EQ_INT(pcl_clock_process(&c, 100, ev, 64), 0);

    /* running at init: the first block starts */
    {
        const pcl_clock_config_t cfg2 = make_cfg(48000u, 120.0, 24, 1);

        PCL_CHECK_EQ_INT(pcl_clock_init(&c, &cfg2), PCL_OK);
        n = pcl_clock_process(&c, 100, ev, 64);
        PCL_CHECK(n == 2 && ev[0].type == PCL_EVT_START && ev[1].type == PCL_EVT_CLOCK);
    }
}

/* Events come out sorted, in range, START before the pulse at its offset. */
static void test_event_stream_is_well_formed(void)
{
    pcl_clock_t              c;
    const pcl_clock_config_t cfg = make_cfg(48000u, 301.0, 96, 1);
    pcl_event_t              ev[256];
    int                      b;
    unsigned                 seed = 12345u;

    PCL_CHECK_EQ_INT(pcl_clock_init(&c, &cfg), PCL_OK);
    for (b = 0; b < 400; b++)
    {
        size_t frames, n, i;

        seed = seed * 1664525u + 1013904223u;
        frames = 1u + (seed >> 8) % 600u;
        if ((seed >> 4) % 29u == 0u)
            pcl_clock_start(&c);
        else if ((seed >> 4) % 31u == 0u)
            pcl_clock_stop(&c);
        PCL_CHECK(pcl_clock_max_events(&c, frames) <= 256);
        n = pcl_clock_process(&c, frames, ev, pcl_clock_max_events(&c, frames));
        for (i = 0; i < n; i++)
        {
            PCL_CHECK(ev[i].offset < frames);
            if (i > 0)
                PCL_CHECK(ev[i].offset >= ev[i - 1].offset);
            if (i > 0 && ev[i].offset == ev[i - 1].offset)
            {
                /* STOP, START, CLOCK at one offset: never a CLOCK before a START or STOP */
                PCL_CHECK(!(ev[i - 1].type == PCL_EVT_CLOCK && ev[i].type != PCL_EVT_CLOCK));
            }
            PCL_CHECK(ev[i].type == PCL_EVT_CLOCK || ev[i].type == PCL_EVT_START
                || ev[i].type == PCL_EVT_STOP);
            PCL_CHECK(ev[i].note == 0 && ev[i].velocity == 0);
        }
    }
    PCL_CHECK_EQ_INT(pcl_clock_dropped(&c), 0);
}

static void test_tempo_change(void)
{
    enum
    {
        MAXP = 2000
    };
    pcl_clock_t              c;
    const pcl_clock_config_t cfg = make_cfg(48000u, 120.0, 24, 1);
    uint64_t                 t[MAXP];
    pulses_t                 p;
    size_t                   k, slow = 0, fast = 0;

    p.t = t;
    p.n = 0;
    p.cap = MAXP;
    PCL_CHECK_EQ_INT(pcl_clock_init(&c, &cfg), PCL_OK);
    run(&c, 20000, 256, &p);
    PCL_CHECK_EQ_INT(pcl_clock_set_bpm(&c, BPM_OF(240.0)), PCL_OK); /* spacing 500 */
    {
        const size_t first = p.n;

        run(&c, 20000, 256, &p);
        for (k = first; k < p.n; k++)
            t[k] += 20000; /* run() counts from 0 on every call */
    }
    /* consecutive pulses are 1000 apart before the change, 500 after, and at most one in between */
    for (k = 1; k < p.n; k++)
    {
        const uint64_t d = t[k] - t[k - 1];

        PCL_CHECK(d == 1000 || d == 500 || (d > 500 && d < 1000));
        if (d == 1000)
            slow++;
        if (d == 500)
            fast++;
    }
    PCL_CHECK(slow >= 19 && fast >= 38);
}

/* max_events is a bound that is reached: START and a pulse on every period boundary. */
static void test_max_events_is_tight(void)
{
    pcl_clock_t              c;
    const pcl_clock_config_t cfg = make_cfg(48000u, 120.0, 24, 1); /* a pulse every 1000 samples */
    pcl_event_t              ev[16];
    size_t                   n;

    PCL_CHECK_EQ_INT(pcl_clock_init(&c, &cfg), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_clock_max_events(&c, 3001), 5);
    n = pcl_clock_process(&c, 3001, ev, 5); /* START, pulses at 0, 1000, 2000, 3000 */
    PCL_CHECK_EQ_INT(n, 5);
    PCL_CHECK_EQ_INT(pcl_clock_dropped(&c), 0);
    PCL_CHECK_EQ_INT(pcl_clock_max_events(&c, 3000), 5);
    PCL_CHECK_EQ_INT(pcl_clock_max_events(&c, 1), 2);
    pcl_clock_stop(&c);
    n = pcl_clock_process(&c, 3001, ev, 5); /* STOP only */
    PCL_CHECK_EQ_INT(n, 1);
}

/* A buffer that is too small loses events, counts them, and is never written past its end. */
static void test_buffer_limits(void)
{
    pcl_clock_t              c;
    const pcl_clock_config_t cfg = make_cfg(8000u, 999.0, 96, 1); /* a pulse every 5 samples */
    pcl_event_t              buf[16];
    size_t                   n, bound;
    int                      i;

    for (i = 0; i < 16; i++)
    {
        buf[i].offset = 0xFFFF;
        buf[i].type = 0xEE;
        buf[i].note = 0xEE;
        buf[i].velocity = 0xEE;
    }
    PCL_CHECK_EQ_INT(pcl_clock_init(&c, &cfg), PCL_OK);
    bound = pcl_clock_max_events(&c, 256);
    PCL_CHECK(bound >= 50);
    n = pcl_clock_process(&c, 256, buf, 8); /* room for 8 of ~52 */
    PCL_CHECK_EQ_INT(n, 8);
    for (i = 8; i < 16; i++)
        PCL_CHECK(buf[i].type == 0xEE); /* untouched */
    PCL_CHECK(pcl_clock_dropped(&c) >= 40);
    /* the clock itself kept its time: the next block carries on from where it would have been */
    {
        pcl_clock_t  d;
        pcl_event_t  big[128];
        size_t       m;
        const size_t nbig = (pcl_clock_init(&d, &cfg), pcl_clock_process(&d, 256, big, 128));

        PCL_CHECK_EQ_INT(nbig, 8 + pcl_clock_dropped(&c)); /* what was lost is what did not fit */
        m = pcl_clock_process(&c, 256, big, 128);
        {
            pcl_event_t big2[128];
            size_t      m2 = pcl_clock_process(&d, 256, big2, 128);

            size_t j;

            PCL_CHECK_EQ_INT(m, m2);
            for (j = 0; j < m && j < m2; j++)
                PCL_CHECK(big[j].offset == big2[j].offset && big[j].type == big2[j].type
                    && big[j].note == big2[j].note && big[j].velocity == big2[j].velocity);
        }
    }
    /* zero capacity: everything is dropped, nothing written */
    {
        const uint32_t before = pcl_clock_dropped(&c);

        n = pcl_clock_process(&c, 256, NULL, 0);
        PCL_CHECK_EQ_INT(n, 0);
        PCL_CHECK(pcl_clock_dropped(&c) > before);
    }
    /* the bound holds for many block sizes and tempos */
    for (i = 1; i < 2000; i += 37)
    {
        pcl_clock_t              e;
        pcl_event_t              ev[400];
        const pcl_clock_config_t ce
            = make_cfg(48000u, 10.0 + (double)(i % 900), 1 + (uint32_t)(i % 96), 1);

        if (pcl_clock_init(&e, &ce) != PCL_OK)
            continue;
        PCL_CHECK(pcl_clock_process(&e, (size_t)i, ev, 400) <= pcl_clock_max_events(&e, (size_t)i));
        PCL_CHECK_EQ_INT(pcl_clock_dropped(&e), 0);
    }
}

int main(void)
{
    PCL_RUN_TEST(test_init_and_setters);
    PCL_RUN_TEST(test_pulse_positions);
    PCL_RUN_TEST(test_exact_spacing);
    PCL_RUN_TEST(test_long_run_does_not_drift);
    PCL_RUN_TEST(test_block_size_independence);
    PCL_RUN_TEST(test_transport);
    PCL_RUN_TEST(test_event_stream_is_well_formed);
    PCL_RUN_TEST(test_tempo_change);
    PCL_RUN_TEST(test_max_events_is_tight);
    PCL_RUN_TEST(test_buffer_limits);
    return pcl_test_summary("clock");
}
