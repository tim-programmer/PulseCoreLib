/* Tests for the reverb. */
#include <stdlib.h>
#include <string.h>

#include "pcl_test.h"

#define FS 48000u
#define PI 3.14159265358979323846
#define BLOCK 256u
#define MEM_BYTES (160u * 1024u) /* QUALITY, native float / Q31, size 1, 100 ms predelay */
#define MAX_FRAMES (10u * 48000u)

static unsigned char g_mem[MEM_BYTES];
static unsigned char g_mem2[MEM_BYTES];
static pcl_sample_t  g_in[MAX_FRAMES];
static pcl_sample_t  g_l[MAX_FRAMES];
static pcl_sample_t  g_r[MAX_FRAMES];
static pcl_sample_t  g_l2[MAX_FRAMES];
static pcl_sample_t  g_r2[MAX_FRAMES];
static double        g_e[MAX_FRAMES];

/* Runtime conversions (PCL_MS etc. need constants in fixed point). */
static pcl_ms_t ms_of(double ms)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return (float)ms;
#else
    return (pcl_ms_t)(ms * 65536.0 + 0.5);
#endif
}

static pcl_reverb_config_t make_cfg(pcl_reverb_variant_t variant, double rt60_ms)
{
    pcl_reverb_config_t cfg;
    cfg.sample_rate_hz = FS;
    cfg.variant = variant;
    cfg.size = PCL_COEF(1.0);
    cfg.max_predelay_ms = 0u;
    cfg.predelay_ms = PCL_MS(0.0);
    cfg.rt60_ms = ms_of(rt60_ms);
    cfg.hf_ratio = PCL_COEF(1.0);
    cfg.hf_hz = PCL_HZ(4000.0);
    cfg.width = PCL_COEF(1.0);
    cfg.mix = PCL_COEF(1.0);
    cfg.mod_depth = PCL_COEF(0.0);
    cfg.freeze = 0;
    cfg.storage = PCL_DELAY_STORE_NATIVE;
    return cfg;
}

static void init_on(pcl_reverb_t *rv, const pcl_reverb_config_t *cfg, unsigned char *mem)
{
    const size_t need = pcl_reverb_required_bytes(cfg);
    PCL_CHECK(need > 0u && need <= MEM_BYTES);
    PCL_CHECK_EQ_INT(pcl_reverb_init(rv, cfg, mem, need), PCL_OK);
}

static void run(pcl_reverb_t *rv, const pcl_sample_t *in, pcl_sample_t *l, pcl_sample_t *r,
    size_t n, size_t block)
{
    size_t done = 0;
    while (done < n)
    {
        const size_t m = n - done < block ? n - done : block;
        pcl_reverb_process(rv, in + done, l + done, r != NULL ? r + done : NULL, m);
        done += m;
    }
}

static void impulse(pcl_sample_t *x, size_t n, double amp)
{
    memset(x, 0, n * sizeof *x);
    x[0] = pcl_test_from_double(amp);
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

static int same(const pcl_sample_t *a, const pcl_sample_t *b, size_t n)
{
    return memcmp(a, b, n * sizeof *a) == 0;
}

static double energy(const pcl_sample_t *x, size_t from, size_t to)
{
    double e = 0.0;
    size_t i;
    for (i = from; i < to; i++)
        e += pcl_test_to_double(x[i]) * pcl_test_to_double(x[i]);
    return e;
}

/*
 * RT60 from the energy decay curve (Schroeder backward integration) of e[0 .. n): the time from -5
 * to -25 dB, times 3 (T20), in ms.
 */
static double rt60_t20(double *e, size_t n)
{
    double tot = 0.0;
    size_t i, t5 = 0, t25 = 0;
    for (i = n; i-- > 0;)
    {
        tot += e[i];
        e[i] = tot;
    }
    for (i = 0; i < n; i++)
    {
        const double db = 10.0 * log10(e[i] / e[0] + 1e-300);
        if (t5 == 0 && db < -5.0)
            t5 = i;
        if (t25 == 0 && db < -25.0)
        {
            t25 = i;
            break;
        }
    }
    return (double)(t25 - t5) * 3.0 * 1000.0 / FS;
}

/* ---- validation ---- */

static void test_sizing_and_init_validation(void)
{
    pcl_reverb_t        rv, before;
    pcl_reverb_config_t cfg = make_cfg(PCL_REVERB_SMALL, 1000.0);
    const size_t        native = pcl_reverb_required_bytes(&cfg);
    size_t              q15, need;

    PCL_CHECK(native > 0u);
    PCL_CHECK_EQ_INT(pcl_reverb_required_bytes(NULL), 0);
    cfg.storage = PCL_DELAY_STORE_Q15;
    q15 = pcl_reverb_required_bytes(&cfg);
    /* SMALL at 48 kHz, size 1: 5371 comb + 645 all-pass samples, plus primes and 1 predelay slot.
     */
    PCL_CHECK(q15 >= 2u * 6000u && q15 <= 2u * 6100u);
    PCL_CHECK_EQ_INT(native, (q15 - 1u) / 2u * sizeof(pcl_sample_t) + sizeof(pcl_sample_t) - 1u);
    cfg.max_predelay_ms = 100u;
    PCL_CHECK_EQ_INT(pcl_reverb_required_bytes(&cfg), q15 + 2u * 4800u);
    cfg.max_predelay_ms = 0u;
    cfg.size = PCL_COEF(0.5);
    need = pcl_reverb_required_bytes(&cfg);
    PCL_CHECK(need > q15 / 2u - 64u && need < q15 / 2u + 64u);
    cfg.size = PCL_COEF(1.0);
    cfg.variant = PCL_REVERB_QUALITY;
    need = pcl_reverb_required_bytes(&cfg);
    /* 20 554 line + 1 461 diffuser samples, plus primes, modulation slack and the predelay slot. */
    PCL_CHECK(need >= 2u * 22015u && need <= 2u * 22200u);
    cfg.storage = PCL_DELAY_STORE_NATIVE;

    need = pcl_reverb_required_bytes(&cfg);
    PCL_CHECK_EQ_INT(pcl_reverb_init(&rv, &cfg, g_mem, need), PCL_OK);
    before = rv;
    PCL_CHECK_EQ_INT(pcl_reverb_init(NULL, &cfg, g_mem, need), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_reverb_init(&rv, NULL, g_mem, need), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_reverb_init(&rv, &cfg, NULL, need), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_reverb_init(&rv, &cfg, g_mem, need - 1u), PCL_ERR_BUFFER_TOO_SMALL);
    /* An unaligned buffer works: required_bytes includes the alignment slack. */
    PCL_CHECK_EQ_INT(pcl_reverb_init(&rv, &cfg, g_mem + 1, need), PCL_OK);
    PCL_CHECK(((uintptr_t)rv.pre.buf % sizeof(pcl_sample_t)) == 0u);
    PCL_CHECK_EQ_INT(pcl_reverb_init(&rv, &cfg, g_mem, need), PCL_OK);

#define BAD(field, value)                                                                          \
    do                                                                                             \
    {                                                                                              \
        pcl_reverb_config_t b = cfg;                                                               \
        b.field = value;                                                                           \
        PCL_CHECK_EQ_INT(pcl_reverb_required_bytes(&b), 0);                                        \
        PCL_CHECK_EQ_INT(pcl_reverb_init(&rv, &b, g_mem, MEM_BYTES), PCL_ERR_OUT_OF_RANGE);        \
    } while (0)
    before = rv;
    BAD(sample_rate_hz, PCL_SAMPLE_RATE_MIN_HZ - 1u);
    BAD(sample_rate_hz, PCL_SAMPLE_RATE_MAX_HZ + 1u);
    BAD(variant, PCL_REVERB_VARIANT_COUNT);
    BAD(size, PCL_COEF(0.24));
    BAD(size, PCL_COEF(1.01));
    BAD(max_predelay_ms, PCL_REVERB_MAX_PREDELAY_MS + 1u);
    BAD(predelay_ms, PCL_MS(0.5)); /* max_predelay_ms is 0 */
    BAD(rt60_ms, PCL_MS(99.0));
    BAD(rt60_ms, PCL_MS(30001.0));
    BAD(hf_ratio, PCL_COEF(0.09));
    BAD(hf_ratio, PCL_COEF(1.01));
    BAD(hf_hz, PCL_HZ(999.0));
    BAD(hf_hz, PCL_HZ(21601.0));
    BAD(width, PCL_COEF(-0.01));
    BAD(width, PCL_COEF(1.01));
    BAD(mix, PCL_COEF(1.01));
    BAD(mod_depth, PCL_COEF(1.01));
    BAD(storage, PCL_DELAY_STORE_COUNT);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    BAD(size, NAN);
    BAD(rt60_ms, NAN);
    BAD(predelay_ms, NAN);
    BAD(hf_ratio, NAN);
    BAD(hf_hz, NAN);
    BAD(mix, NAN);
#endif
#undef BAD
    PCL_CHECK(memcmp(&rv, &before, sizeof rv) == 0); /* untouched on error */

    /* The extreme rates and sizes initialize (8 kHz at size 0.25 is the shortest layout). */
    cfg.storage = PCL_DELAY_STORE_Q15;
    cfg.sample_rate_hz = PCL_SAMPLE_RATE_MIN_HZ;
    cfg.size = PCL_COEF(0.25);
    cfg.hf_hz = PCL_HZ(1000.0);
    PCL_CHECK_EQ_INT(pcl_reverb_init(&rv, &cfg, g_mem, MEM_BYTES), PCL_OK);
    cfg.variant = PCL_REVERB_SMALL;
    PCL_CHECK_EQ_INT(pcl_reverb_init(&rv, &cfg, g_mem, MEM_BYTES), PCL_OK);
    cfg.sample_rate_hz = 192000u;
    cfg.size = PCL_COEF(1.0);
    need = pcl_reverb_required_bytes(&cfg);
    PCL_CHECK(need > 0u && need <= MEM_BYTES);
    PCL_CHECK_EQ_INT(pcl_reverb_init(&rv, &cfg, g_mem, need), PCL_OK);
    cfg.sample_rate_hz = PCL_SAMPLE_RATE_MAX_HZ; /* sizes, even if the test has no room for it */
    cfg.variant = PCL_REVERB_QUALITY;
    cfg.storage = PCL_DELAY_STORE_NATIVE;
    PCL_CHECK(pcl_reverb_required_bytes(&cfg) > 4u * need);
}

static void test_setter_validation(void)
{
    pcl_reverb_t        rv, before;
    pcl_reverb_config_t cfg = make_cfg(PCL_REVERB_QUALITY, 1000.0);

    cfg.max_predelay_ms = 50u;
    init_on(&rv, &cfg, g_mem);
    before = rv;
    PCL_CHECK_EQ_INT(pcl_reverb_set_predelay(&rv, PCL_MS(50.5)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_reverb_set_rt60(&rv, PCL_MS(99.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_reverb_set_rt60(&rv, PCL_MS(30001.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(
        pcl_reverb_set_damping(&rv, PCL_COEF(0.09), PCL_HZ(4000.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(
        pcl_reverb_set_damping(&rv, PCL_COEF(0.5), PCL_HZ(21601.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_reverb_set_width(&rv, PCL_COEF(1.01)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_reverb_set_mix(&rv, PCL_COEF(-0.01)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_reverb_set_mod_depth(&rv, PCL_COEF(1.01)), PCL_ERR_OUT_OF_RANGE);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    PCL_CHECK_EQ_INT(pcl_reverb_set_rt60(&rv, NAN), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_reverb_set_mix(&rv, NAN), PCL_ERR_OUT_OF_RANGE);
#endif
    PCL_CHECK(memcmp(&rv, &before, sizeof rv) == 0);
    PCL_CHECK_EQ_INT(pcl_reverb_set_predelay(NULL, PCL_MS(1.0)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_reverb_set_rt60(NULL, PCL_MS(1000.0)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(
        pcl_reverb_set_damping(NULL, PCL_COEF(1.0), PCL_HZ(4000.0)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_reverb_set_width(NULL, PCL_COEF(1.0)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_reverb_set_mix(NULL, PCL_COEF(1.0)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_reverb_set_mod_depth(NULL, PCL_COEF(1.0)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_reverb_set_predelay(&rv, PCL_MS(50.0)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_reverb_set_rt60(&rv, PCL_MS(30000.0)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_reverb_set_damping(&rv, PCL_COEF(0.1), PCL_HZ(21600.0)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_reverb_set_width(&rv, PCL_COEF(0.0)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_reverb_set_mix(&rv, PCL_COEF(0.0)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_reverb_set_mod_depth(&rv, PCL_COEF(1.0)), PCL_OK);
}

/* Every tank and all-pass length is a distinct prime, so all pairs are coprime. */
static int is_prime(uint32_t n)
{
    uint32_t d;
    if (n < 2u)
        return 0;
    for (d = 2u; d * d <= n; d++)
        if (n % d == 0u)
            return 0;
    return 1;
}

static void test_line_lengths_are_distinct_primes(void)
{
    static const uint32_t rates[] = { 8000u, 44100u, 48000u, 96000u };
    pcl_reverb_t          rv;
    unsigned              v, s, k, i, j;

    for (v = 0; v < (unsigned)PCL_REVERB_VARIANT_COUNT; v++)
        for (s = 0; s < 3u; s++)
            for (k = 0; k < sizeof rates / sizeof rates[0]; k++)
            {
                pcl_reverb_config_t cfg = make_cfg((pcl_reverb_variant_t)v, 1000.0);
                const unsigned      n_tank = v == PCL_REVERB_QUALITY ? 8u : 4u;
                uint32_t            len[12];
                unsigned            n = 0;
                cfg.sample_rate_hz = rates[k];
                cfg.size = s == 0u ? PCL_COEF(0.25) : (s == 1u ? PCL_COEF(0.6) : PCL_COEF(1.0));
                cfg.storage = PCL_DELAY_STORE_Q15;
                cfg.hf_hz = PCL_HZ(1000.0);
                PCL_CHECK_EQ_INT(pcl_reverb_init(&rv, &cfg, g_mem, MEM_BYTES), PCL_OK);
                for (i = 0; i < n_tank; i++)
                    len[n++] = rv.delay[i];
                for (i = 0; i < PCL_REVERB_ALLPASSES; i++)
                    len[n++] = rv.ap[i].length;
                for (i = 0; i < n; i++)
                {
                    PCL_CHECK(is_prime(len[i]));
                    for (j = i + 1u; j < n; j++)
                        PCL_CHECK(len[i] != len[j]);
                }
                /* Lengths follow the size: the longest tank line at size 1 is 30.7 / 83.9 ms. */
                if (s == 2u && rates[k] == FS)
                    PCL_CHECK_NEAR((double)rv.delay[n_tank - 1u],
                        v == PCL_REVERB_QUALITY ? 4027.0 : 1476.0, 8.0);
            }
}

/* ---- decay ---- */

static double measure_rt60(pcl_reverb_variant_t variant, double rt60, pcl_delay_storage_t storage)
{
    pcl_reverb_t        rv;
    pcl_reverb_config_t cfg = make_cfg(variant, rt60);
    const size_t        n = (size_t)((rt60 * 1.6 + 300.0) * FS / 1000.0);
    size_t              i;

    cfg.storage = storage;
    init_on(&rv, &cfg, g_mem);
    impulse(g_in, n, 0.9);
    run(&rv, g_in, g_l, g_r, n, BLOCK);
    PCL_CHECK(finite_no_denormals(g_l, n) && finite_no_denormals(g_r, n));
    for (i = 0; i < n; i++)
    {
        const double a = pcl_test_to_double(g_l[i]), b = pcl_test_to_double(g_r[i]);
        g_e[i] = a * a + b * b;
    }
    return rt60_t20(g_e, n);
}

static void test_rt60_accuracy(void)
{
    static const double rts[] = { 300.0, 1000.0, 3000.0 };
    unsigned            v, k, s;

    for (v = 0; v < (unsigned)PCL_REVERB_VARIANT_COUNT; v++)
        for (k = 0; k < 3u; k++)
            for (s = 0; s < 2u; s++)
            {
                const double got
                    = measure_rt60((pcl_reverb_variant_t)v, rts[k], (pcl_delay_storage_t)s);
                PCL_CHECK_NEAR(got / rts[k], 1.0, 0.08);
            }
}

/* A 2nd-order Butterworth (RBJ cookbook), in place on doubles: low-pass or high-pass. */
static void butter(double *x, size_t n, double fc, int highpass)
{
    const double w = 2.0 * PI * fc / FS, c = cos(w), al = sin(w) / (2.0 * sqrt(0.5));
    const double a0 = 1.0 + al;
    const double b1 = (highpass ? -(1.0 + c) : (1.0 - c)) / a0;
    const double b0 = (highpass ? (1.0 + c) / 2.0 : (1.0 - c) / 2.0) / a0, b2 = b0;
    const double a1 = -2.0 * c / a0, a2 = (1.0 - al) / a0;
    double       x1 = 0, x2 = 0, y1 = 0, y2 = 0;
    size_t       i;
    for (i = 0; i < n; i++)
    {
        const double y = b0 * x[i] + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        x2 = x1;
        x1 = x[i];
        y2 = y1;
        y1 = y;
        x[i] = y;
    }
}

static double band_rt60(const pcl_sample_t *x, size_t n, double fc, int highpass)
{
    double *b = (double *)malloc(n * sizeof *b);
    double  rt;
    size_t  i;
    for (i = 0; i < n; i++)
        b[i] = pcl_test_to_double(x[i]);
    butter(b, n, fc, highpass);
    butter(b, n, fc, highpass);
    for (i = 0; i < n; i++)
        g_e[i] = b[i] * b[i];
    rt = rt60_t20(g_e, n);
    free(b);
    return rt;
}

static void test_hf_damping(void)
{
    unsigned v;

    for (v = 0; v < (unsigned)PCL_REVERB_VARIANT_COUNT; v++)
    {
        pcl_reverb_t        rv;
        pcl_reverb_config_t cfg = make_cfg((pcl_reverb_variant_t)v, 2000.0);
        const size_t        n = 4u * FS;
        double              lo, hi;

        cfg.hf_ratio = PCL_COEF(0.4);
        cfg.hf_hz = PCL_HZ(2000.0);
        init_on(&rv, &cfg, g_mem);
        /* A 20 ms burst rather than one impulse: enough level in each band for 16-bit lines. */
        memset(g_in, 0, n * sizeof g_in[0]);
        fill_noise(g_in, FS / 50u, 17u, 0.9);
        run(&rv, g_in, g_l, NULL, n, BLOCK);
        lo = band_rt60(g_l, n, 200.0, 0);
        hi = band_rt60(g_l, n, 10000.0, 1);
        /* Low band: the set RT60. High band: about 0.4 of it (the one-pole shelf is not a brick
         * wall, so a little longer). */
        printf("    damping: %s low band %.0f ms, high band %.2f of it\n", v ? "QUALITY" : "SMALL",
            lo, hi / lo);
        PCL_CHECK_NEAR(lo / 2000.0, 1.0, 0.1);
        PCL_CHECK(hi / lo > 0.35 && hi / lo < 0.55);
    }
}

/* After the input stops: fixed point reaches exactly 0 (no limit cycles); float stays clean. */
static void test_tail_reaches_silence(void)
{
    unsigned v, s;

    for (v = 0; v < (unsigned)PCL_REVERB_VARIANT_COUNT; v++)
        for (s = 0; s < 2u; s++)
        {
            pcl_reverb_t        rv;
            pcl_reverb_config_t cfg = make_cfg((pcl_reverb_variant_t)v, 1000.0);
            const size_t        burst = FS / 4u, n = 8u * FS;

            cfg.storage = (pcl_delay_storage_t)s;
            cfg.mod_depth = PCL_COEF(1.0);
            init_on(&rv, &cfg, g_mem);
            memset(g_in, 0, n * sizeof g_in[0]);
            fill_noise(g_in, burst, 7u + v, 0.9);
            run(&rv, g_in, g_l, g_r, n, BLOCK);
            PCL_CHECK(finite_no_denormals(g_l, n) && finite_no_denormals(g_r, n));
#if PCL_SAMPLE_TYPE == PCL_FLOAT
            /* 7.75 s of silence after a 1 s RT60: far below -120 dB. */
            PCL_CHECK(energy(g_l, n - FS / 10u, n) < 1e-14);
#else
            {
                size_t i, last = 0;
                for (i = 0; i < n; i++)
                    if (g_l[i] != 0 || g_r[i] != 0)
                        last = i;
                PCL_CHECK(last < n - FS);
            }
#endif
        }
}

static void test_freeze_holds(void)
{
    unsigned v;

    for (v = 0; v < (unsigned)PCL_REVERB_VARIANT_COUNT; v++)
    {
        pcl_reverb_t        rv;
        pcl_reverb_config_t cfg = make_cfg((pcl_reverb_variant_t)v, 1000.0);
        const size_t        n = 6u * FS;
        double              e1, e5;

        cfg.mod_depth
            = PCL_COEF(1.0); /* frozen, the modulation stops (it would low-pass the tail) */
        init_on(&rv, &cfg, g_mem);
        fill_noise(g_in, n, 11u, 0.5);
        run(&rv, g_in, g_l, g_r, FS / 2u, BLOCK);
        pcl_reverb_set_freeze(&rv, 1);
        /* The input keeps coming but is muted after the fade. */
        run(&rv, g_in + FS / 2u, g_l, g_r, n - FS / 2u, BLOCK);
        e1 = energy(g_l, FS / 2u, FS) + energy(g_r, FS / 2u, FS);
        e5 = energy(g_l, n - 2u * FS, n - 3u * FS / 2u)
            + energy(g_r, n - 2u * FS, n - 3u * FS / 2u);
        PCL_CHECK(e1 > 0.0);
        /* Held within 0.5 dB over 4 s (a 1 s RT60 would have dropped 240 dB). */
        PCL_CHECK_NEAR(10.0 * log10(e5 / e1), 0.0, 0.5);
        PCL_CHECK(finite_no_denormals(g_l, n - FS / 2u));
        /* Unfreezing resumes the decay. */
        pcl_reverb_set_freeze(&rv, 0);
        memset(g_in, 0, 2u * FS * sizeof g_in[0]);
        run(&rv, g_in, g_l, g_r, 2u * FS, BLOCK);
        PCL_CHECK(energy(g_l, 3u * FS / 2u, 2u * FS) < 1e-4 * e1);
    }
}

/* ---- timing, stereo, mix ---- */

/* Non-zero, above float's anti-denormal guard (1e-24 per line write). */
static int audible(pcl_sample_t x) { return fabs(pcl_test_to_double(x)) > 1e-12; }

static void test_predelay_and_first_arrival(void)
{
    unsigned v;

    for (v = 0; v < (unsigned)PCL_REVERB_VARIANT_COUNT; v++)
    {
        pcl_reverb_t        rv;
        pcl_reverb_config_t cfg = make_cfg((pcl_reverb_variant_t)v, 1000.0);
        const size_t        n = FS / 4u;
        size_t              i, first = 0, shortest;

        cfg.max_predelay_ms = 100u;
        cfg.predelay_ms = PCL_MS(25.0);
        init_on(&rv, &cfg, g_mem);
        shortest = rv.delay[0];
        impulse(g_in, n, 0.9);
        run(&rv, g_in, g_l, g_r, n, 7u);
        for (i = 0; i < n; i++)
            if (audible(g_l[i]) || audible(g_r[i]))
            {
                first = i;
                break;
            }
        /* The all-passes pass their input straight through, so the first echo is the shortest
         * tank line after the predelay. */
        PCL_CHECK_EQ_INT(first, 1200u + shortest);

        PCL_CHECK_EQ_INT(pcl_reverb_set_predelay(&rv, PCL_MS(0.0)), PCL_OK);
        pcl_reverb_reset(&rv);
        run(&rv, g_in, g_l, g_r, n, 7u);
        for (i = 0; i < n; i++)
            if (audible(g_l[i]) || audible(g_r[i]))
                break;
        PCL_CHECK_EQ_INT(i, shortest);
    }
}

static double correlation(const pcl_sample_t *a, const pcl_sample_t *b, size_t from, size_t to)
{
    double ab = 0, aa = 0, bb = 0;
    size_t i;
    for (i = from; i < to; i++)
    {
        const double x = pcl_test_to_double(a[i]), y = pcl_test_to_double(b[i]);
        ab += x * y;
        aa += x * x;
        bb += y * y;
    }
    return ab / sqrt(aa * bb);
}

static void test_width_and_mono(void)
{
    unsigned v;

    for (v = 0; v < (unsigned)PCL_REVERB_VARIANT_COUNT; v++)
    {
        pcl_reverb_t        rv;
        pcl_reverb_config_t cfg = make_cfg((pcl_reverb_variant_t)v, 1500.0);
        const size_t        n = FS;
        size_t              i;
        double              worst = 0.0;

        fill_noise(g_in, n, 3u, 0.25); /* no clipping, so mono == the stereo average */
        init_on(&rv, &cfg, g_mem);
        run(&rv, g_in, g_l, g_r, n, BLOCK);
        /* Full width: the two channels are nearly uncorrelated. */
        PCL_CHECK(fabs(correlation(g_l, g_r, FS / 4u, n)) < 0.3);
        /* The mono output is the average of the two. */
        pcl_reverb_reset(&rv);
        run(&rv, g_in, g_l2, NULL, n, BLOCK);
        for (i = 0; i < n; i++)
        {
            const double want = 0.5 * (pcl_test_to_double(g_l[i]) + pcl_test_to_double(g_r[i]));
            const double d = fabs(pcl_test_to_double(g_l2[i]) - want);
            worst = d > worst ? d : worst;
        }
        PCL_CHECK(worst <= 2.0 * PCL_TEST_SAMPLE_TOL + 1e-6);

        cfg.width = PCL_COEF(0.0);
        init_on(&rv, &cfg, g_mem);
        run(&rv, g_in, g_l, g_r, n, BLOCK);
        PCL_CHECK(same(g_l, g_r, n)); /* width 0: mono wet */
        PCL_CHECK(energy(g_l, FS / 4u, n) > 0.0);
    }
}

static void test_mix(void)
{
    pcl_reverb_t        rv;
    pcl_reverb_config_t cfg = make_cfg(PCL_REVERB_QUALITY, 1000.0);
    const size_t        n = FS / 2u, ramp = FS * PCL_SMOOTH_MS / 1000u;

    fill_noise(g_in, n, 5u, 0.7);
    cfg.mix = PCL_COEF(0.0);
    init_on(&rv, &cfg, g_mem);
    run(&rv, g_in, g_l, g_r, n, BLOCK);
    PCL_CHECK(same(g_l, g_in, n) && same(g_r, g_in, n)); /* dry only: bit-exact */

    /* A mix change ramps over PCL_SMOOTH_MS, then the output is the wet signal alone. */
    PCL_CHECK_EQ_INT(pcl_reverb_set_mix(&rv, PCL_COEF(1.0)), PCL_OK);
    run(&rv, g_in, g_l, g_r, n, BLOCK);
    cfg.mix = PCL_COEF(1.0);
    init_on(&rv, &cfg, g_mem2);
    run(&rv, g_in, g_l2, g_r2, n, BLOCK);
    run(&rv, g_in, g_l2, g_r2, n, BLOCK);
    PCL_CHECK(same(g_l + ramp, g_l2 + ramp, n - ramp));
    PCL_CHECK(!same(g_l, g_l2, ramp));
}

/* ---- determinism, blocks, aliasing ---- */

static void render_blocked(pcl_reverb_variant_t v, pcl_delay_storage_t s, size_t block,
    pcl_sample_t *l, pcl_sample_t *r, size_t n)
{
    pcl_reverb_t        rv;
    pcl_reverb_config_t cfg = make_cfg(v, 2000.0);

    cfg.storage = s;
    cfg.mod_depth = PCL_COEF(1.0);
    cfg.max_predelay_ms = 20u;
    cfg.predelay_ms = PCL_MS(7.5);
    cfg.hf_ratio = PCL_COEF(0.5);
    init_on(&rv, &cfg, g_mem2);
    run(&rv, g_in, l, r, n, block);
}

static void test_block_size_independence(void)
{
    static const size_t blocks[] = { 1u, 7u, 16u, 17u, 256u };
    const size_t        n = FS / 2u;
    unsigned            v, s, k;

    fill_noise(g_in, n, 21u, 0.6);
    for (v = 0; v < (unsigned)PCL_REVERB_VARIANT_COUNT; v++)
        for (s = 0; s < 2u; s++)
        {
            render_blocked((pcl_reverb_variant_t)v, (pcl_delay_storage_t)s, n, g_l, g_r, n);
            for (k = 0; k < sizeof blocks / sizeof blocks[0]; k++)
            {
                render_blocked(
                    (pcl_reverb_variant_t)v, (pcl_delay_storage_t)s, blocks[k], g_l2, g_r2, n);
                PCL_CHECK(same(g_l, g_l2, n) && same(g_r, g_r2, n));
            }
        }
}

static void test_reset_aliasing_and_small_ignores_mod(void)
{
    pcl_reverb_t        rv;
    pcl_reverb_config_t cfg = make_cfg(PCL_REVERB_QUALITY, 1500.0);
    const size_t        n = FS / 2u;

    cfg.mod_depth = PCL_COEF(0.7);
    cfg.mix = PCL_COEF(0.5);
    fill_noise(g_in, n, 9u, 0.6);
    init_on(&rv, &cfg, g_mem);
    run(&rv, g_in, g_l, g_r, n, BLOCK);
    pcl_reverb_reset(&rv);
    run(&rv, g_in, g_l2, g_r2, n, BLOCK);
    PCL_CHECK(same(g_l, g_l2, n) && same(g_r, g_r2, n)); /* reset: bit-identical */

    /* In place: in aliases out_l. */
    pcl_reverb_reset(&rv);
    memcpy(g_l2, g_in, n * sizeof g_in[0]);
    run(&rv, g_l2, g_l2, g_r2, n, BLOCK);
    PCL_CHECK(same(g_l, g_l2, n) && same(g_r, g_r2, n));

    /* Modulation changes QUALITY's output and leaves SMALL's alone. */
    PCL_CHECK_EQ_INT(pcl_reverb_set_mod_depth(&rv, PCL_COEF(0.0)), PCL_OK);
    pcl_reverb_reset(&rv);
    run(&rv, g_in, g_l2, g_r2, n, BLOCK);
    PCL_CHECK(!same(g_l, g_l2, n));
    cfg.variant = PCL_REVERB_SMALL;
    init_on(&rv, &cfg, g_mem);
    run(&rv, g_in, g_l, g_r, n, BLOCK);
    PCL_CHECK_EQ_INT(pcl_reverb_set_mod_depth(&rv, PCL_COEF(0.0)), PCL_OK);
    pcl_reverb_reset(&rv);
    run(&rv, g_in, g_l2, g_r2, n, BLOCK);
    PCL_CHECK(same(g_l, g_l2, n) && same(g_r, g_r2, n));
}

/* ---- against a double-precision model ---- */

typedef struct
{
    double *buf;
    size_t  len, w;
} mline_t;

static void mline_init(mline_t *l, size_t len)
{
    l->buf = (double *)calloc(len, sizeof(double));
    l->len = len;
    l->w = 0;
}

/* Reads the value written `len` samples ago, then writes v. */
static double mline_step(mline_t *l, double v)
{
    const double o = l->buf[l->w];
    l->buf[l->w] = v;
    l->w = l->w + 1u == l->len ? 0u : l->w + 1u;
    return o;
}

static double mallpass(mline_t *l, double u, double g)
{
    const double wo = l->buf[l->w];
    const double wn = u + g * wo;
    mline_step(l, wn);
    return wo - g * wn;
}

/*
 * The algorithm of docs/modules/reverb.md in doubles, on the lengths the module chose, with gains
 * from the formulas (not pcl_db_to_gain). No modulation; width 1, mix 1; stereo out.
 */
static void model(const pcl_reverb_t *rv, int quality, double rt60, double ratio, double hf,
    const pcl_sample_t *in, double *l, double *r, size_t n)
{
    static const double diff_g[4] = { 0.75, 0.75, 0.625, 0.625 };
    static const int    neg[8] = { 0, 1, 1, 0, 1, 0, 0, 1 }; /* Hadamard row 7 */
    const unsigned      nt = quality ? 8u : 4u;
    const double        p = exp(-2.0 * PI * hf / FS);
    const double        scale = quality ? sqrt(0.125) : 1.0;
    mline_t             tank[8], ap[4];
    double              gh[8], gl[8], lp[8] = { 0 };
    size_t              i;
    unsigned            j;

    for (j = 0; j < nt; j++)
    {
        const double d = rv->delay[j];
        const double g = pow(10.0, -3.0 * d / (rt60 * FS / 1000.0));
        const double h = pow(10.0, -3.0 * d / (rt60 * ratio * FS / 1000.0));
        gh[j] = h * scale;
        gl[j] = (g - h) * scale;
        mline_init(&tank[j], rv->delay[j]);
    }
    for (j = 0; j < 4u; j++)
        mline_init(&ap[j], rv->ap[j].length);
    for (i = 0; i < n; i++)
    {
        const double x = pcl_test_to_double(in[i]);
        double       y[8], h[8], wl, wr;
        if (!quality)
        {
            const double u = 0.1 * x;
            for (j = 0; j < 4u; j++)
            {
                y[j] = tank[j].buf[tank[j].w];
                lp[j] = p * lp[j] + (1.0 - p) * y[j];
                mline_step(&tank[j], u + gh[j] * y[j] + gl[j] * lp[j]);
            }
            wl = 0.5 * ((y[0] + y[1]) - (y[2] + y[3]));
            wr = 0.5 * ((y[0] - y[1]) + (y[2] - y[3]));
            wl = mallpass(&ap[1], mallpass(&ap[0], wl, 0.7), 0.7);
            wr = mallpass(&ap[3], mallpass(&ap[2], wr, 0.7), 0.7);
        }
        else
        {
            double   u = 0.165 * x;
            unsigned s;
            for (j = 0; j < 4u; j++)
                u = mallpass(&ap[j], u, diff_g[j]);
            for (j = 0; j < 8u; j++)
            {
                y[j] = tank[j].buf[tank[j].w];
                lp[j] = p * lp[j] + (1.0 - p) * y[j];
                h[j] = gh[j] * y[j] + gl[j] * lp[j];
            }
            for (s = 1u; s < 8u; s <<= 1)
                for (j = 0; j < 8u; j++)
                    if ((j & s) == 0u)
                    {
                        const double a = h[j], b = h[j + s];
                        h[j] = a + b;
                        h[j + s] = a - b;
                    }
            for (j = 0; j < 8u; j++)
                mline_step(&tank[j], h[j] + (neg[j] ? -u : u));
            wl = h[1];
            wr = h[2];
        }
        l[i] = 2.0 * wl; /* the wet make-up gain */
        r[i] = 2.0 * wr;
    }
    for (j = 0; j < nt; j++)
        free(tank[j].buf);
    for (j = 0; j < 4u; j++)
        free(ap[j].buf);
}

static void test_matches_double_model(void)
{
    /* Over the first 150 ms (later the fixed-point tails decorrelate): float and Q31 differ from
     * the model only by pcl_db_to_gain's 8e-6 and rounding; 16-bit storage adds its quantization.
     */
#if PCL_SAMPLE_TYPE == PCL_Q15
    const double tol[2] = { 3e-4, 3e-4 };
#else
    const double tol[2] = { 5e-6, 3e-4 };
#endif
    const size_t n = 150u * FS / 1000u;
    double      *ml = (double *)malloc(n * sizeof(double));
    double      *mr = (double *)malloc(n * sizeof(double));
    unsigned     v, s;

    fill_noise(g_in, n, 31u, 0.7);
    for (v = 0; v < (unsigned)PCL_REVERB_VARIANT_COUNT; v++)
        for (s = 0; s < 2u; s++)
        {
            pcl_reverb_t        rv;
            pcl_reverb_config_t cfg = make_cfg((pcl_reverb_variant_t)v, 1200.0);
            double              worst = 0.0;
            size_t              i;

            cfg.storage = (pcl_delay_storage_t)s;
            cfg.hf_ratio = PCL_COEF(0.5);
            cfg.hf_hz = PCL_HZ(3000.0);
            init_on(&rv, &cfg, g_mem);
            run(&rv, g_in, g_l, g_r, n, BLOCK);
            model(&rv, v == PCL_REVERB_QUALITY, 1200.0, 0.5, 3000.0, g_in, ml, mr, n);
            for (i = 0; i < n; i++)
            {
                const double dl = fabs(pcl_test_to_double(g_l[i]) - pcl_test_clip(ml[i]));
                const double dr = fabs(pcl_test_to_double(g_r[i]) - pcl_test_clip(mr[i]));
                worst = dl > worst ? dl : worst;
                worst = dr > worst ? dr : worst;
            }
            printf("    model: %s %s worst %.2e\n", v ? "QUALITY" : "SMALL", s ? "Q15" : "native",
                worst);
            PCL_CHECK(worst <= tol[s]);
        }
    free(ml);
    free(mr);
}

/* ---- extremes ---- */

static void test_extremes_stay_bounded(void)
{
    unsigned v, s;

    for (v = 0; v < (unsigned)PCL_REVERB_VARIANT_COUNT; v++)
        for (s = 0; s < 2u; s++)
        {
            pcl_reverb_t        rv;
            pcl_reverb_config_t cfg = make_cfg((pcl_reverb_variant_t)v, 30000.0);
            const size_t        n = 4u * FS;
            size_t              i;

            cfg.storage = (pcl_delay_storage_t)s;
            cfg.size = PCL_COEF(0.25);
            cfg.mod_depth = PCL_COEF(1.0);
            init_on(&rv, &cfg, g_mem);
            /* Full-scale square-ish noise, with freeze toggled and parameters thrown around. */
            for (i = 0; i < n; i++)
                g_in[i] = pcl_test_from_double(((i / 37u) & 1u) ? 1.0 : -1.0);
            for (i = 0; i < n; i += 4096u)
            {
                const size_t m = n - i < 4096u ? n - i : 4096u;
                pcl_reverb_set_freeze(&rv, (int)((i / 4096u) % 3u == 1u));
                pcl_reverb_set_damping(&rv, (i / 4096u) & 1u ? PCL_COEF(0.1) : PCL_COEF(1.0),
                    (i / 4096u) & 2u ? PCL_HZ(1000.0) : PCL_HZ(21600.0));
                pcl_reverb_set_rt60(&rv, (i / 4096u) & 4u ? PCL_MS(100.0) : PCL_MS(30000.0));
                run(&rv, g_in + i, g_l + i, g_r + i, m, 61u);
            }
            PCL_CHECK(finite_no_denormals(g_l, n) && finite_no_denormals(g_r, n));
#if PCL_SAMPLE_TYPE == PCL_FLOAT
            {
                float m = 0.0f;
                for (i = 0; i < n; i++)
                {
                    m = fabsf(g_l[i]) > m ? fabsf(g_l[i]) : m;
                    m = fabsf(g_r[i]) > m ? fabsf(g_r[i]) : m;
                }
                PCL_CHECK(m < 16.0f);
            }
#endif
        }
}

int main(void)
{
    PCL_RUN_TEST(test_sizing_and_init_validation);
    PCL_RUN_TEST(test_setter_validation);
    PCL_RUN_TEST(test_line_lengths_are_distinct_primes);
    PCL_RUN_TEST(test_rt60_accuracy);
    PCL_RUN_TEST(test_hf_damping);
    PCL_RUN_TEST(test_tail_reaches_silence);
    PCL_RUN_TEST(test_freeze_holds);
    PCL_RUN_TEST(test_predelay_and_first_arrival);
    PCL_RUN_TEST(test_width_and_mono);
    PCL_RUN_TEST(test_mix);
    PCL_RUN_TEST(test_block_size_independence);
    PCL_RUN_TEST(test_reset_aliasing_and_small_ignores_mod);
    PCL_RUN_TEST(test_matches_double_model);
    PCL_RUN_TEST(test_extremes_stay_bounded);
    return pcl_test_summary("reverb");
}
