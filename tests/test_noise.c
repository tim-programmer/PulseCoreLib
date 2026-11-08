/*
 * Tests for the noise sources and the sample-and-hold (include/pcl/pcl_noise.h).
 *
 * The generators are integer-only and identical in every build, so correctness is checked
 * against independent models written here (plain integer/double code, a different structure
 * from the library: running sums become full re-sums, shifts become floor divisions). The
 * spectra are measured with an FFT (Welch average) as 1/3-octave power densities.
 */
#include <stdlib.h>
#include <string.h>

#include "pcl_test.h"

#define SAMPLE_RATE_HZ 48000u
#define BLOCK 256u

/* Conversion of a Q31 reference value into what a build must output for it. */
#if PCL_SAMPLE_TYPE == PCL_FLOAT
#define Q31_TOL 6.0e-8 /* (float)q rounds to 24 bits: relative error <= 2^-24 */
#else
#define Q31_TOL 0.0
#endif

static double expect_from_q31(int32_t q)
{
#if PCL_SAMPLE_TYPE == PCL_Q15
    return floor((double)q / 65536.0) / 32768.0;
#else
    return (double)q / 2147483648.0;
#endif
}

static double expect_from_q30(int32_t v) /* sample-and-hold values: Q2.30 */
{
#if PCL_SAMPLE_TYPE == PCL_Q15
    return floor((double)v / 32768.0) / 32768.0;
#else
    return (double)v / 1073741824.0;
#endif
}

/* ---- Independent models ---- */

static uint32_t model_next(uint32_t *s)
{
    uint32_t x = *s;
    x = x ^ (x << 13);
    x = x ^ (x >> 17);
    x = x ^ (x << 5);
    *s = x;
    return x;
}

static uint32_t model_seed(uint32_t seed) { return seed ? seed : 0x2545F491u; }

static int32_t model_signed(uint32_t v)
{
    return v >= 0x80000000u ? (int32_t)(v - 0x80000000u) - 2147483647 - 1 : (int32_t)v;
}

/* floor(x / 2^k) without relying on right-shifting negatives. */
static long long floor_div_pow2(long long x, int k)
{
    const long long d = 1ll << k;
    long long       q = x / d;
    if (x % d != 0 && x < 0)
        q--;
    return q;
}

static void model_white(uint32_t seed, size_t n, int32_t *out)
{
    uint32_t s = model_seed(seed);
    size_t   i;
    for (i = 0; i < n; i++)
        out[i] = model_signed(model_next(&s));
}

static void model_pink(uint32_t seed, size_t n, int32_t *out)
{
    uint32_t s = model_seed(seed);
    int32_t  rows[16];
    uint32_t counter = 0;
    size_t   i;
    int      r;

    for (r = 0; r < 16; r++)
        rows[r] = (int32_t)floor_div_pow2(model_signed(model_next(&s)), 4);
    for (i = 0; i < n; i++)
    {
        long long sum = 0;
        rows[0] = (int32_t)floor_div_pow2(model_signed(model_next(&s)), 4);
        counter++;
        for (r = 1; r <= 15; r++)
        {
            /* row r is replaced when the counter has exactly r-1 trailing zeros */
            if ((counter % (1u << r)) == (1u << (r - 1)))
                rows[r] = (int32_t)floor_div_pow2(model_signed(model_next(&s)), 4);
        }
        for (r = 0; r < 16; r++)
            sum += rows[r];
        out[i] = (int32_t)sum;
    }
}

static int model_brown_k(uint32_t fs)
{
    int k = (int)floor(log2((double)fs / (2.0 * 3.14159265358979323846 * 15.0)) + 0.5);
    return k < 6 ? 6 : (k > 13 ? 13 : k);
}

static long long clamp_ll(long long v, long long lo, long long hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static void model_brown(uint32_t seed, uint32_t fs, size_t n, int32_t *out)
{
    uint32_t        s = model_seed(seed);
    const int       k = model_brown_k(fs);
    const long long gain
        = (long long)floor(0.1 * sqrt(2.0) / (sqrt(1.0 / 3.0) * pow(2.0, k / 2.0)) * 65536.0 + 0.5);
    const long long fs29 = 1ll << 29;
    long long       acc = 0, prev_in = 0, prev_out = 0;
    size_t          i;

    for (i = 0; i < n; i++)
    {
        const long long x = floor_div_pow2(model_signed(model_next(&s)), 16);
        const long long u = floor_div_pow2(x * gain, 2);
        long long       y;

        acc = clamp_ll(acc + u - floor_div_pow2(acc, k), -fs29, fs29 - 1);
        y = clamp_ll(acc - prev_in + prev_out - floor_div_pow2(prev_out, k + 2), -fs29, fs29 - 1);
        prev_in = acc;
        prev_out = y;
        out[i] = (int32_t)(y * 4);
    }
}

/* ---- Helpers ---- */

static void make_noise(pcl_noise_t *n, pcl_noise_colour_t colour, uint32_t fs, uint32_t seed)
{
    pcl_noise_config_t cfg;
    cfg.sample_rate_hz = fs;
    cfg.colour = colour;
    cfg.seed = seed;
    PCL_CHECK_EQ_INT(pcl_noise_init(n, &cfg), PCL_OK);
}

/* Renders `frames` samples in blocks of `block` (the last one may be shorter). */
static void render(pcl_noise_t *n, pcl_sample_t *out, size_t frames, size_t block)
{
    size_t done = 0;
    while (done < frames)
    {
        const size_t c = frames - done < block ? frames - done : block;
        pcl_noise_process(n, out + done, c);
        done += c;
    }
}

static const char *colour_name(pcl_noise_colour_t c)
{
    return c == PCL_NOISE_WHITE ? "white" : (c == PCL_NOISE_PINK ? "pink" : "brown");
}

static void model_run(pcl_noise_colour_t c, uint32_t seed, uint32_t fs, size_t n, int32_t *out)
{
    if (c == PCL_NOISE_WHITE)
        model_white(seed, n, out);
    else if (c == PCL_NOISE_PINK)
        model_pink(seed, n, out);
    else
        model_brown(seed, fs, n, out);
}

#define MODEL_FRAMES 20000u
static pcl_sample_t g_out[MODEL_FRAMES];
static pcl_sample_t g_out2[MODEL_FRAMES];
static int32_t      g_model[MODEL_FRAMES];

/* ---- Validation ---- */

static void test_init_validation(void)
{
    pcl_noise_t        n, before;
    pcl_noise_config_t cfg;

    cfg.sample_rate_hz = SAMPLE_RATE_HZ;
    cfg.colour = PCL_NOISE_PINK;
    cfg.seed = 7u;
    PCL_CHECK_EQ_INT(pcl_noise_init(&n, &cfg), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_noise_init(NULL, &cfg), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_noise_init(&n, NULL), PCL_ERR_NULL_POINTER);

    cfg.sample_rate_hz = PCL_SAMPLE_RATE_MIN_HZ - 1u;
    PCL_CHECK_EQ_INT(pcl_noise_init(&n, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.sample_rate_hz = PCL_SAMPLE_RATE_MAX_HZ + 1u;
    PCL_CHECK_EQ_INT(pcl_noise_init(&n, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.sample_rate_hz = PCL_SAMPLE_RATE_MIN_HZ;
    PCL_CHECK_EQ_INT(pcl_noise_init(&n, &cfg), PCL_OK);
    cfg.sample_rate_hz = PCL_SAMPLE_RATE_MAX_HZ;
    PCL_CHECK_EQ_INT(pcl_noise_init(&n, &cfg), PCL_OK);

    cfg.sample_rate_hz = SAMPLE_RATE_HZ;
    cfg.colour = PCL_NOISE_COLOUR_COUNT;
    PCL_CHECK_EQ_INT(pcl_noise_init(&n, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.colour = (pcl_noise_colour_t)-1;
    PCL_CHECK_EQ_INT(pcl_noise_init(&n, &cfg), PCL_ERR_OUT_OF_RANGE);

    /* Errors leave the module untouched. */
    make_noise(&n, PCL_NOISE_BROWN, SAMPLE_RATE_HZ, 5u);
    pcl_noise_process(&n, g_out, 100);
    before = n;
    cfg.colour = PCL_NOISE_WHITE;
    cfg.sample_rate_hz = 1u;
    PCL_CHECK_EQ_INT(pcl_noise_init(&n, &cfg), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK(memcmp(&n, &before, sizeof n) == 0);
    PCL_CHECK_EQ_INT(pcl_noise_set_colour(&n, PCL_NOISE_COLOUR_COUNT), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK(memcmp(&n, &before, sizeof n) == 0);
    PCL_CHECK_EQ_INT(pcl_noise_set_colour(NULL, PCL_NOISE_WHITE), PCL_ERR_NULL_POINTER);
}

/* ---- Exact agreement with the models ---- */

static void check_against_model(pcl_noise_colour_t c, uint32_t fs, uint32_t seed, size_t block)
{
    pcl_noise_t n;
    size_t      i, bad = 0;

    make_noise(&n, c, fs, seed);
    render(&n, g_out, MODEL_FRAMES, block);
    model_run(c, seed, fs, MODEL_FRAMES, g_model);
    for (i = 0; i < MODEL_FRAMES; i++)
        if (fabs(pcl_test_to_double(g_out[i]) - expect_from_q31(g_model[i])) > Q31_TOL)
            bad++;
    if (bad != 0)
        printf("    %s fs=%u seed=%u block=%u: %u samples differ from the model\n", colour_name(c),
            (unsigned)fs, (unsigned)seed, (unsigned)block, (unsigned)bad);
    PCL_CHECK_EQ_INT(bad, 0);
}

static void test_matches_models(void)
{
    static const uint32_t rates[] = { 8000u, 44100u, 48000u, 96000u, 768000u };
    int                   c;
    size_t                r;

    for (c = 0; c < PCL_NOISE_COLOUR_COUNT; c++)
        for (r = 0; r < sizeof rates / sizeof rates[0]; r++)
            check_against_model((pcl_noise_colour_t)c, rates[r], 12345u, BLOCK);
}

/* xorshift32 is pinned by a published value: seed 1 -> 270369. */
static void test_white_known_value(void)
{
    pcl_noise_t n;
    make_noise(&n, PCL_NOISE_WHITE, SAMPLE_RATE_HZ, 1u);
    pcl_noise_process(&n, g_out, 2);
    PCL_CHECK_NEAR(pcl_test_to_double(g_out[0]), expect_from_q31(270369), Q31_TOL);
}

static void test_seed_semantics(void)
{
    pcl_noise_t a, b;
    int         c;

    for (c = 0; c < PCL_NOISE_COLOUR_COUNT; c++)
    {
        make_noise(&a, (pcl_noise_colour_t)c, SAMPLE_RATE_HZ, 0u);
        make_noise(&b, (pcl_noise_colour_t)c, SAMPLE_RATE_HZ, PCL_NOISE_DEFAULT_SEED);
        pcl_noise_process(&a, g_out, 2000);
        pcl_noise_process(&b, g_out2, 2000);
        PCL_CHECK(memcmp(g_out, g_out2, 2000 * sizeof g_out[0]) == 0); /* 0 == default */

        make_noise(&b, (pcl_noise_colour_t)c, SAMPLE_RATE_HZ, 99u);
        pcl_noise_process(&b, g_out2, 2000);
        PCL_CHECK(memcmp(g_out, g_out2, 2000 * sizeof g_out[0]) != 0); /* seeds differ */

        pcl_noise_set_seed(&b, 0u);
        pcl_noise_process(&b, g_out2, 2000);
        PCL_CHECK(memcmp(g_out, g_out2, 2000 * sizeof g_out[0]) == 0);
    }
}

static void test_reset_and_set_colour_determinism(void)
{
    pcl_noise_t a, b;
    int         c, d;

    for (c = 0; c < PCL_NOISE_COLOUR_COUNT; c++)
    {
        make_noise(&a, (pcl_noise_colour_t)c, SAMPLE_RATE_HZ, 77u);
        pcl_noise_process(&a, g_out, 5000);
        pcl_noise_reset(&a);
        pcl_noise_process(&a, g_out2, 5000);
        PCL_CHECK(memcmp(g_out, g_out2, 5000 * sizeof g_out[0]) == 0);

        /* set_colour from any other colour behaves like a fresh init of that colour */
        for (d = 0; d < PCL_NOISE_COLOUR_COUNT; d++)
        {
            make_noise(&b, (pcl_noise_colour_t)d, SAMPLE_RATE_HZ, 77u);
            pcl_noise_process(&b, g_out2, 3333);
            PCL_CHECK_EQ_INT(pcl_noise_set_colour(&b, (pcl_noise_colour_t)c), PCL_OK);
            pcl_noise_process(&b, g_out2, 5000);
            PCL_CHECK(memcmp(g_out, g_out2, 5000 * sizeof g_out[0]) == 0);
        }
    }
}

static void test_block_size_independence(void)
{
    static const size_t blocks[] = { 1u, 7u, 256u, 1000u };
    pcl_noise_t         n;
    int                 c;
    size_t              b;

    for (c = 0; c < PCL_NOISE_COLOUR_COUNT; c++)
    {
        make_noise(&n, (pcl_noise_colour_t)c, SAMPLE_RATE_HZ, 31337u);
        pcl_noise_process(&n, g_out, MODEL_FRAMES); /* one large block */
        for (b = 0; b < sizeof blocks / sizeof blocks[0]; b++)
        {
            make_noise(&n, (pcl_noise_colour_t)c, SAMPLE_RATE_HZ, 31337u);
            render(&n, g_out2, MODEL_FRAMES, blocks[b]);
            PCL_CHECK(memcmp(g_out, g_out2, sizeof g_out) == 0);
        }
    }
}

/* ---- Spectra ---- */

#define FFT_N 8192u
#define SPEC_FRAMES (1u << 21)

static double g_psd[FFT_N / 2 + 1];
static double g_twr[FFT_N / 2], g_twi[FFT_N / 2];
static double g_re[FFT_N], g_im[FFT_N], g_win[FFT_N];
static double g_spec[SPEC_FRAMES];

static void fft_init(void)
{
    size_t i;
    for (i = 0; i < FFT_N / 2; i++)
    {
        g_twr[i] = cos(-2.0 * 3.14159265358979323846 * (double)i / (double)FFT_N);
        g_twi[i] = sin(-2.0 * 3.14159265358979323846 * (double)i / (double)FFT_N);
    }
    for (i = 0; i < FFT_N; i++)
        g_win[i] = 0.5 - 0.5 * cos(2.0 * 3.14159265358979323846 * (double)i / (double)FFT_N);
}

static void fft(double *re, double *im)
{
    size_t i, j = 0, len;
    for (i = 1; i < FFT_N; i++)
    {
        size_t bit = FFT_N >> 1;
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
    for (len = 2; len <= FFT_N; len <<= 1)
    {
        const size_t step = FFT_N / len;
        for (i = 0; i < FFT_N; i += len)
        {
            size_t k;
            for (k = 0; k < len / 2; k++)
            {
                const double wr = g_twr[k * step], wi = g_twi[k * step];
                const size_t a = i + k, b = i + k + len / 2;
                const double xr = re[b] * wr - im[b] * wi;
                const double xi = re[b] * wi + im[b] * wr;
                re[b] = re[a] - xr;
                im[b] = im[a] - xi;
                re[a] += xr;
                im[a] += xi;
            }
        }
    }
}

/* Welch power spectrum of x[0..n) into g_psd. */
static void welch(const double *x, size_t n)
{
    size_t pos, segments = 0, i;
    memset(g_psd, 0, sizeof g_psd);
    for (pos = 0; pos + FFT_N <= n; pos += FFT_N / 2)
    {
        for (i = 0; i < FFT_N; i++)
        {
            g_re[i] = x[pos + i] * g_win[i];
            g_im[i] = 0.0;
        }
        fft(g_re, g_im);
        for (i = 0; i <= FFT_N / 2; i++)
            g_psd[i] += g_re[i] * g_re[i] + g_im[i] * g_im[i];
        segments++;
    }
    for (i = 0; i <= FFT_N / 2; i++)
        g_psd[i] /= (double)segments;
}

/*
 * Mean power density (dB) in successive 1/3-octave bands between f_lo and f_hi, regressed against
 * log2(frequency): returns the slope in dB per octave and the largest band deviation from that
 * line.
 */
static void spectrum_slope(
    const double *x, size_t n, double fs, double f_lo, double f_hi, double *slope, double *max_dev)
{
    double band_db[64], band_l2[64];
    double sx = 0, sy = 0, sxx = 0, sxy = 0, icpt, f0, third = pow(2.0, 1.0 / 3.0);
    int    nb = 0, b;

    welch(x, n);
    for (f0 = f_lo; f0 * third <= f_hi * 1.0001 && nb < 64; f0 *= third)
    {
        const double f1 = f0 * third;
        double       p = 0.0;
        int          count = 0;
        size_t       k;
        for (k = 1; k < FFT_N / 2; k++)
        {
            const double f = (double)k * fs / (double)FFT_N;
            if (f >= f0 && f < f1)
            {
                p += g_psd[k];
                count++;
            }
        }
        if (count == 0)
            continue;
        band_db[nb] = 10.0 * log10(p / (double)count);
        band_l2[nb] = log2(sqrt(f0 * f1));
        nb++;
    }
    for (b = 0; b < nb; b++)
    {
        sx += band_l2[b];
        sy += band_db[b];
        sxx += band_l2[b] * band_l2[b];
        sxy += band_l2[b] * band_db[b];
    }
    *slope = ((double)nb * sxy - sx * sy) / ((double)nb * sxx - sx * sx);
    icpt = (sy - *slope * sx) / (double)nb;
    *max_dev = 0.0;
    for (b = 0; b < nb; b++)
    {
        const double d = fabs(band_db[b] - (*slope * band_l2[b] + icpt));
        if (d > *max_dev)
            *max_dev = d;
    }
}

static void render_spectrum_input(pcl_noise_colour_t c, uint32_t fs, size_t frames)
{
    pcl_noise_t  n;
    pcl_sample_t chunk[BLOCK];
    size_t       done = 0, i;

    make_noise(&n, c, fs, 2024u);
    while (done < frames)
    {
        const size_t m = frames - done < BLOCK ? frames - done : BLOCK;
        pcl_noise_process(&n, chunk, m);
        for (i = 0; i < m; i++)
            g_spec[done + i] = pcl_test_to_double(chunk[i]);
        done += m;
    }
}

static void test_white_mean_and_flat_spectrum(void)
{
    double mean = 0.0, slope, dev;
    size_t i;

    fft_init();
    render_spectrum_input(PCL_NOISE_WHITE, SAMPLE_RATE_HZ, SPEC_FRAMES);
    for (i = 0; i < SPEC_FRAMES; i++)
        mean += g_spec[i];
    mean /= (double)SPEC_FRAMES;
    /* standard error of the mean of uniform noise with rms 0.577 */
    PCL_CHECK(fabs(mean) < 5.0 * 0.5774 / sqrt((double)SPEC_FRAMES));

    spectrum_slope(g_spec, SPEC_FRAMES, SAMPLE_RATE_HZ, 100.0, 20000.0, &slope, &dev);
    printf("    white: slope %+.3f dB/oct, worst band %.3f dB, mean %+.2e\n", slope, dev, mean);
    PCL_CHECK(fabs(slope) < 0.1);
    PCL_CHECK(dev < 0.5);
}

static void test_pink_spectrum(void)
{
    double slope, dev;

    fft_init();
    render_spectrum_input(PCL_NOISE_PINK, SAMPLE_RATE_HZ, SPEC_FRAMES);
    spectrum_slope(g_spec, SPEC_FRAMES, SAMPLE_RATE_HZ, 100.0, 20000.0, &slope, &dev);
    printf("    pink 100 Hz..20 kHz: slope %+.3f dB/oct, worst band %.3f dB\n", slope, dev);
    PCL_CHECK_NEAR(slope, -3.01, 0.3);
    PCL_CHECK(dev < 1.0);
}

static void check_brown_spectrum(uint32_t fs, double f_hi)
{
    double slope, dev;

    fft_init();
    render_spectrum_input(PCL_NOISE_BROWN, fs, SPEC_FRAMES / 2);
    spectrum_slope(g_spec, SPEC_FRAMES / 2, (double)fs, 100.0, f_hi, &slope, &dev);
    printf("    brown fs=%u, 100 Hz..%.0f Hz: slope %+.3f dB/oct, worst band %.3f dB\n",
        (unsigned)fs, f_hi, slope, dev);
    PCL_CHECK_NEAR(slope, -6.02, 0.3);
    PCL_CHECK(dev < 0.5);
}

static void test_brown_spectrum(void)
{
    /* The upper edge is 0.2 * sample rate: a discrete integrator departs from -6 dB/oct above it.
     */
    check_brown_spectrum(SAMPLE_RATE_HZ, 9600.0);
    check_brown_spectrum(8000u, 1600.0);   /* k = 6 */
    check_brown_spectrum(96000u, 19200.0); /* k = 10 */
}

/* ---- Levels, headroom, finite values ---- */

static void test_levels(void)
{
    static const double expected_rms[] = { 0.577, 0.143, 0.090 };
    pcl_noise_t         n;
    int                 c;
    size_t              i;

    for (c = 0; c < PCL_NOISE_COLOUR_COUNT; c++)
    {
        double sum2 = 0.0, peak = 0.0;
        make_noise(&n, (pcl_noise_colour_t)c, SAMPLE_RATE_HZ, 2024u);
        for (i = 0; i < 100u; i++)
        {
            size_t j;
            pcl_noise_process(&n, g_out, MODEL_FRAMES);
            for (j = 0; j < MODEL_FRAMES; j++)
            {
                const double v = pcl_test_to_double(g_out[j]);
                sum2 += v * v;
                if (fabs(v) > peak)
                    peak = fabs(v);
            }
        }
        {
            const double rms = sqrt(sum2 / (100.0 * MODEL_FRAMES));
            printf("    %s: rms %.4f, peak %.3f\n", colour_name((pcl_noise_colour_t)c), rms, peak);
            PCL_CHECK_NEAR(rms, expected_rms[c], 0.01);
        }
    }
}

/* 10 minutes at 48 kHz: nothing saturates, nothing non-finite, nothing denormal. */
static void check_long_run(pcl_noise_colour_t c, uint32_t seed, size_t frames, double peak_limit)
{
    pcl_noise_t  n;
    pcl_sample_t buf[BLOCK];
    double       peak = 0.0;
    size_t       done = 0, i, rails = 0, bad = 0;

    make_noise(&n, c, SAMPLE_RATE_HZ, seed);
    while (done < frames)
    {
        const size_t m = frames - done < BLOCK ? frames - done : BLOCK;
        pcl_noise_process(&n, buf, m);
        for (i = 0; i < m; i++)
        {
            const double v = pcl_test_to_double(buf[i]);
            if (fabs(v) > peak)
                peak = fabs(v);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
            if (!isfinite(buf[i]) || (buf[i] != 0.0f && fabsf(buf[i]) < FLT_MIN))
                bad++;
            if (fabsf(buf[i]) >= 1.0f)
                rails++;
#elif PCL_SAMPLE_TYPE == PCL_Q15
            if (buf[i] == INT16_MAX || buf[i] == INT16_MIN)
                rails++;
#else
            if (buf[i] == INT32_MAX || buf[i] == INT32_MIN)
                rails++;
#endif
        }
        done += m;
    }
    printf("    %s seed %u, %.1f min: peak %.3f, rails %u, clamps %u\n", colour_name(c),
        (unsigned)seed, (double)frames / SAMPLE_RATE_HZ / 60.0, peak, (unsigned)rails,
        (unsigned)pcl_noise_clamp_count(&n));
    if (peak_limit < 1.0) /* white is uniform over the full range, so it may touch the rails */
        PCL_CHECK_EQ_INT(rails, 0);
    PCL_CHECK_EQ_INT(bad, 0);
    PCL_CHECK_EQ_INT(pcl_noise_clamp_count(&n), 0);
    PCL_CHECK(peak < peak_limit);
}

static void test_ten_minutes_no_saturation(void)
{
    const size_t ten_minutes = (size_t)SAMPLE_RATE_HZ * 600u;

    check_long_run(
        PCL_NOISE_WHITE, 0u, SAMPLE_RATE_HZ * 10u, 1.01); /* white reaches 1.0 by design */
    check_long_run(PCL_NOISE_PINK, 0u, ten_minutes, 0.9);
    check_long_run(PCL_NOISE_BROWN, 0u, ten_minutes, 0.9);
    check_long_run(PCL_NOISE_BROWN, 1u, ten_minutes, 0.9);
    check_long_run(PCL_NOISE_BROWN, 0xDEADBEEFu, ten_minutes, 0.9);
    check_long_run(PCL_NOISE_PINK, 0xDEADBEEFu, ten_minutes, 0.9);
}

/* ---- Sample-and-hold ---- */

static void make_sah(pcl_sah_t *s, double rate_hz, uint32_t slew_ms, uint32_t seed)
{
    pcl_sah_config_t cfg;
    cfg.sample_rate_hz = SAMPLE_RATE_HZ;
    cfg.seed = seed;
    cfg.rate_hz = PCL_HZ(rate_hz);
    cfg.slew_ms = slew_ms;
    PCL_CHECK_EQ_INT(pcl_sah_init(s, &cfg), PCL_OK);
}

/* The sequence of picks the library must make for a seed: pick 0 at init, then one per trigger. */
static int32_t model_pick(uint32_t *s) { return model_signed(model_next(s)) >> 1; }

/* Expected S&H output (no slew) for a trigger flag per sample. */
static void sah_model(uint32_t seed, const unsigned char *trig, size_t n, double *expect)
{
    uint32_t s = model_seed(seed);
    int32_t  v = model_pick(&s);
    size_t   i;
    for (i = 0; i < n; i++)
    {
        if (trig[i])
            v = model_pick(&s);
        expect[i] = expect_from_q30(v);
    }
}

#define SAH_FRAMES 20000u
static unsigned char g_trig[SAH_FRAMES];
static double        g_expect[SAH_FRAMES];

/* Counts samples whose output differs from the expectation. */
static size_t count_mismatches(const pcl_sample_t *out, const double *expect, size_t n)
{
    size_t i, bad = 0;
    for (i = 0; i < n; i++)
        if (fabs(pcl_test_to_double(out[i]) - expect[i]) > Q31_TOL + 1e-12)
            bad++;
    return bad;
}

static void test_sah_validation(void)
{
    pcl_sah_t        s, before;
    pcl_sah_config_t cfg;

    cfg.sample_rate_hz = SAMPLE_RATE_HZ;
    cfg.seed = 3u;
    cfg.rate_hz = PCL_HZ(10.0);
    cfg.slew_ms = 5u;
    PCL_CHECK_EQ_INT(pcl_sah_init(&s, &cfg), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_sah_init(NULL, &cfg), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_sah_init(&s, NULL), PCL_ERR_NULL_POINTER);

    cfg.sample_rate_hz = PCL_SAMPLE_RATE_MIN_HZ - 1u;
    PCL_CHECK_EQ_INT(pcl_sah_init(&s, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.sample_rate_hz = PCL_SAMPLE_RATE_MAX_HZ + 1u;
    PCL_CHECK_EQ_INT(pcl_sah_init(&s, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.sample_rate_hz = SAMPLE_RATE_HZ;

    cfg.rate_hz = PCL_HZ(0.0);
    PCL_CHECK_EQ_INT(pcl_sah_init(&s, &cfg), PCL_OK);
    cfg.rate_hz = PCL_HZ(24000.0); /* exactly 0.5 * sample rate */
    PCL_CHECK_EQ_INT(pcl_sah_init(&s, &cfg), PCL_OK);
    cfg.rate_hz = PCL_HZ(24001.0);
    PCL_CHECK_EQ_INT(pcl_sah_init(&s, &cfg), PCL_ERR_OUT_OF_RANGE);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    cfg.rate_hz = -1.0f;
    PCL_CHECK_EQ_INT(pcl_sah_init(&s, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.rate_hz = NAN;
    PCL_CHECK_EQ_INT(pcl_sah_init(&s, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.rate_hz = INFINITY;
    PCL_CHECK_EQ_INT(pcl_sah_init(&s, &cfg), PCL_ERR_OUT_OF_RANGE);
#endif
    cfg.rate_hz = PCL_HZ(10.0);

    cfg.slew_ms = PCL_SAH_SLEW_MAX_MS;
    PCL_CHECK_EQ_INT(pcl_sah_init(&s, &cfg), PCL_OK);
    cfg.slew_ms = PCL_SAH_SLEW_MAX_MS + 1u;
    PCL_CHECK_EQ_INT(pcl_sah_init(&s, &cfg), PCL_ERR_OUT_OF_RANGE);

    /* the longest slew at the highest sample rate must not overflow 32-bit sample counts */
    cfg.slew_ms = PCL_SAH_SLEW_MAX_MS;
    cfg.sample_rate_hz = PCL_SAMPLE_RATE_MAX_HZ;
    PCL_CHECK_EQ_INT(pcl_sah_init(&s, &cfg), PCL_OK);
    PCL_CHECK_EQ_INT(s.value.ramp_samples, 7680000);

    cfg.sample_rate_hz = SAMPLE_RATE_HZ;
    cfg.slew_ms = 5u;
    PCL_CHECK_EQ_INT(pcl_sah_init(&s, &cfg), PCL_OK);
    before = s;
    cfg.slew_ms = PCL_SAH_SLEW_MAX_MS + 1u;
    PCL_CHECK_EQ_INT(pcl_sah_init(&s, &cfg), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK(memcmp(&s, &before, sizeof s) == 0);

    PCL_CHECK_EQ_INT(pcl_sah_set_rate(NULL, PCL_HZ(1.0)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_sah_set_rate(&s, PCL_HZ(24001.0)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK(memcmp(&s, &before, sizeof s) == 0);
    PCL_CHECK_EQ_INT(pcl_sah_set_rate(&s, PCL_HZ(2.0)), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_sah_set_slew_ms(NULL, 1u), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_sah_set_slew_ms(&s, PCL_SAH_SLEW_MAX_MS + 1u), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_sah_set_slew_ms(&s, 0u), PCL_OK);
}

/* Internal clock: picks at exactly the instants an independent phase-accumulator model gives. */
static void test_sah_clock_rate(void)
{
    static const double rates[] = { 0.5, 5.0, 100.0, 1000.0 };
    const size_t        frames = (size_t)SAMPLE_RATE_HZ * 20u;
    size_t              r;

    for (r = 0; r < sizeof rates / sizeof rates[0]; r++)
    {
        pcl_sah_t      s;
        const uint64_t inc
            = (uint64_t)floor(rates[r] / (double)SAMPLE_RATE_HZ * 4294967296.0 + 0.5);
        uint32_t     rng = model_seed(5u);
        int32_t      v = model_pick(&rng);
        size_t       i, done = 0, bad = 0, picks = 0, last = 0;
        pcl_sample_t out[BLOCK];
        double       ideal = (double)SAMPLE_RATE_HZ / rates[r], worst_dev = 0.0;

        make_sah(&s, rates[r], 0u, 5u);
        while (done < frames)
        {
            const size_t m = frames - done < BLOCK ? frames - done : BLOCK;
            pcl_sah_process(&s, out, NULL, NULL, 0, m);
            for (i = 0; i < m; i++)
            {
                const size_t n = done + i;
                if ((((uint64_t)(n + 1)) * inc >> 32) > (((uint64_t)n * inc) >> 32))
                {
                    double dev;
                    v = model_pick(&rng);
                    picks++;
                    dev = fabs((double)(n - last) - ideal);
                    if (picks > 1 && dev > worst_dev)
                        worst_dev = dev;
                    last = n;
                }
                if (fabs(pcl_test_to_double(out[i]) - expect_from_q30(v)) > Q31_TOL + 1e-12)
                    bad++;
            }
            done += m;
        }
        printf("    S&H %.1f Hz: %u picks in 20 s, worst interval error %.3f samples\n", rates[r],
            (unsigned)picks, worst_dev);
        PCL_CHECK_EQ_INT(bad, 0);
        PCL_CHECK(llabs((long long)picks - (long long)(rates[r] * 20.0)) <= 1);
        PCL_CHECK(worst_dev <= 1.0 + 1e-9); /* at most one sample of jitter around the period */
    }
}

static void test_sah_rate_zero_is_trigger_only(void)
{
    pcl_sah_t    s;
    pcl_sample_t first, out[BLOCK];
    size_t       i;
    int          same = 1;

    make_sah(&s, 0.0, 0u, 9u);
    pcl_sah_process(&s, out, NULL, NULL, 0, 1);
    first = out[0];
    for (i = 0; i < 400u; i++)
    {
        size_t k;
        pcl_sah_process(&s, out, NULL, NULL, 0, BLOCK);
        for (k = 0; k < BLOCK; k++)
            same = same && out[k] == first;
    }
    PCL_CHECK(same);
}

/* Builds the events for the block [start, start + len) from absolute trigger times. */
static size_t events_for_block(
    const size_t *abs_times, size_t count, size_t start, size_t len, pcl_event_t *ev)
{
    size_t i, n = 0;
    for (i = 0; i < count; i++)
    {
        if (abs_times[i] >= start && abs_times[i] < start + len)
        {
            ev[n].offset = (uint16_t)(abs_times[i] - start);
            ev[n].type = PCL_EVT_CLOCK;
            ev[n].note = 0;
            ev[n].velocity = 0;
            n++;
        }
    }
    return n;
}

static void test_sah_event_offsets(void)
{
    static const size_t times[] = { 0, 1, 7, 7, 100, 255, 256, 257, 511, 1000, 1001, 1002, 4095 };
    const size_t        count = sizeof times / sizeof times[0];
    static const size_t blocks[] = { 1, 7, 256, 1000, SAH_FRAMES };
    pcl_sah_t           s;
    size_t              b, i;

    memset(g_trig, 0, sizeof g_trig);
    for (i = 0; i < count; i++)
        g_trig[times[i]] = 1;
    sah_model(11u, g_trig, SAH_FRAMES, g_expect);

    for (b = 0; b < sizeof blocks / sizeof blocks[0]; b++)
    {
        size_t done = 0;
        make_sah(&s, 0.0, 0u, 11u);
        while (done < SAH_FRAMES)
        {
            const size_t len = SAH_FRAMES - done < blocks[b] ? SAH_FRAMES - done : blocks[b];
            pcl_event_t  ev[32];
            size_t       n = events_for_block(times, count, done, len, ev);
            pcl_sah_process(&s, g_out + done, NULL, ev, n, len);
            done += len;
        }
        PCL_CHECK_EQ_INT(count_mismatches(g_out, g_expect, SAH_FRAMES), 0);
    }
}

static void test_sah_event_filtering(void)
{
    pcl_sah_t   s;
    pcl_event_t ev[4];

    /* NOTE_ON, NOTE_OFF and a CLOCK past the end of the block do nothing; CLOCK at 10 fires. */
    ev[0].offset = 5;
    ev[0].type = PCL_EVT_NOTE_ON;
    ev[0].note = 60;
    ev[0].velocity = 100;
    ev[1].offset = 8;
    ev[1].type = PCL_EVT_NOTE_OFF;
    ev[1].note = 60;
    ev[1].velocity = 0;
    ev[2].offset = 10;
    ev[2].type = PCL_EVT_CLOCK;
    ev[2].note = 0;
    ev[2].velocity = 0;
    ev[3].offset = 64;
    ev[3].type = PCL_EVT_CLOCK;
    ev[3].note = 0;
    ev[3].velocity = 0; /* == frames */

    memset(g_trig, 0, sizeof g_trig);
    g_trig[10] = 1;
    sah_model(21u, g_trig, 64, g_expect);
    make_sah(&s, 0.0, 0u, 21u);
    pcl_sah_process(&s, g_out, NULL, ev, 4, 64);
    PCL_CHECK_EQ_INT(count_mismatches(g_out, g_expect, 64), 0);
}

/* Reference Schmitt trigger: fires on a rise through 0.5, re-arms below 0.25. */
static void gate_model(const double *g, size_t n, unsigned char *trig)
{
    int    high = 0;
    size_t i;
    for (i = 0; i < n; i++)
    {
        trig[i] = 0;
        if (g[i] >= 0.5)
        {
            if (!high)
                trig[i] = 1;
            high = 1;
        }
        else if (g[i] < 0.25)
        {
            high = 0;
        }
    }
}

static void test_sah_gate_input(void)
{
    static const size_t blocks[] = { 1, 7, 64, 256, SAH_FRAMES };
    static double       gate[SAH_FRAMES];
    static pcl_sample_t gate_s[SAH_FRAMES];
    pcl_sah_t           s;
    size_t              i, b;

    /* starts high (fires on the first sample); a held gate does not retrigger; dips to 0.3 stay
       inside the hysteresis band; a dip to 0.1 re-arms; edges straddle block boundaries */
    for (i = 0; i < SAH_FRAMES; i++)
        gate[i] = 0.0;
    for (i = 0; i < 40; i++)
        gate[i] = 0.9;
    for (i = 100; i < 300; i++)
        gate[i] = 0.8;
    gate[150] = 0.3;
    gate[151] = 0.3;
    gate[200] = 0.1; /* re-arm ... */
    gate[201] = 0.6; /* ... fires again */
    for (i = 254; i < 258; i++)
        gate[i] = 0.5; /* exactly the threshold fires */
    gate[500] = 0.49;  /* below the threshold: never fires */
    for (i = 1000; i < 1003; i++)
        gate[i] = 1.0;
    gate[1003] = 0.0;
    gate[1004] = 1.0;
    for (i = 0; i < SAH_FRAMES; i++)
        gate_s[i] = pcl_test_from_double(gate[i]);
    {
        /* model on the quantized levels the library sees */
        static double quantized[SAH_FRAMES];
        for (i = 0; i < SAH_FRAMES; i++)
            quantized[i] = pcl_test_to_double(gate_s[i]);
        gate_model(quantized, SAH_FRAMES, g_trig);
    }
    PCL_CHECK(g_trig[0] == 1);
    sah_model(33u, g_trig, SAH_FRAMES, g_expect);

    for (b = 0; b < sizeof blocks / sizeof blocks[0]; b++)
    {
        size_t done = 0;
        make_sah(&s, 0.0, 0u, 33u);
        while (done < SAH_FRAMES)
        {
            const size_t len = SAH_FRAMES - done < blocks[b] ? SAH_FRAMES - done : blocks[b];
            pcl_sah_process(&s, g_out + done, gate_s + done, NULL, 0, len);
            done += len;
        }
        PCL_CHECK_EQ_INT(count_mismatches(g_out, g_expect, SAH_FRAMES), 0);
    }

    /* the trigger input may alias the output */
    make_sah(&s, 0.0, 0u, 33u);
    memcpy(g_out2, gate_s, sizeof gate_s);
    pcl_sah_process(&s, g_out2, g_out2, NULL, 0, SAH_FRAMES);
    PCL_CHECK_EQ_INT(count_mismatches(g_out2, g_expect, SAH_FRAMES), 0);
}

static void test_sah_all_sources_and_block_independence(void)
{
    static const size_t blocks[] = { 1, 7, 256, 4096 };
    static const size_t ev_times[] = { 50, 51, 3000, 3001 };
    static pcl_sample_t gate_s[SAH_FRAMES];
    pcl_sah_t           s;
    size_t              i, b;
    const double        rate = 37.0;
    const uint64_t      inc = (uint64_t)floor(rate / (double)SAMPLE_RATE_HZ * 4294967296.0 + 0.5);

    for (i = 0; i < SAH_FRAMES; i++)
        gate_s[i] = pcl_test_from_double((i / 500) % 2 == 0 ? 0.0 : 0.9);
    memset(g_trig, 0, sizeof g_trig);
    for (i = 0; i < SAH_FRAMES; i++)
        if ((((uint64_t)(i + 1)) * inc >> 32) > (((uint64_t)i * inc) >> 32))
            g_trig[i] = 1;
    for (i = 0; i < sizeof ev_times / sizeof ev_times[0]; i++)
        g_trig[ev_times[i]] = 1;
    for (i = 1; i < SAH_FRAMES; i++)
        if (((i / 500) % 2 == 1) && ((i - 1) / 500) % 2 == 0)
            g_trig[i] = 1;
    sah_model(77u, g_trig, SAH_FRAMES, g_expect);

    for (b = 0; b < sizeof blocks / sizeof blocks[0]; b++)
    {
        size_t done = 0;
        make_sah(&s, rate, 0u, 77u);
        while (done < SAH_FRAMES)
        {
            const size_t len = SAH_FRAMES - done < blocks[b] ? SAH_FRAMES - done : blocks[b];
            pcl_event_t  ev[8];
            const size_t n
                = events_for_block(ev_times, sizeof ev_times / sizeof ev_times[0], done, len, ev);
            pcl_sah_process(&s, g_out + done, gate_s + done, ev, n, len);
            done += len;
        }
        PCL_CHECK_EQ_INT(count_mismatches(g_out, g_expect, SAH_FRAMES), 0);
    }
}

static void test_sah_determinism(void)
{
    pcl_sah_t s;

    make_sah(&s, 200.0, 3u, 4u);
    pcl_sah_process(&s, g_out, NULL, NULL, 0, 5000);
    pcl_sah_reset(&s);
    pcl_sah_process(&s, g_out2, NULL, NULL, 0, 5000);
    PCL_CHECK(memcmp(g_out, g_out2, 5000 * sizeof g_out[0]) == 0);

    /* set_seed(0) == the default seed == a fresh init with seed 0 */
    make_sah(&s, 200.0, 3u, 0u);
    pcl_sah_process(&s, g_out, NULL, NULL, 0, 5000);
    pcl_sah_set_seed(&s, PCL_NOISE_DEFAULT_SEED);
    pcl_sah_process(&s, g_out2, NULL, NULL, 0, 5000);
    PCL_CHECK(memcmp(g_out, g_out2, 5000 * sizeof g_out[0]) == 0);

    /* the first value is a real pick, not zero */
    {
        uint32_t rng = model_seed(4u);
        make_sah(&s, 0.0, 0u, 4u);
        pcl_sah_process(&s, g_out, NULL, NULL, 0, 1);
        PCL_CHECK_NEAR(pcl_test_to_double(g_out[0]), expect_from_q30(model_pick(&rng)), Q31_TOL);
    }
}

/* Slew: the output starts moving at the trigger sample and arrives exactly slew_samples later. */
static void test_sah_slew(void)
{
    const uint32_t slew_ms = 10u;
    const size_t   ramp = (size_t)SAMPLE_RATE_HZ * slew_ms / 1000u; /* 480 */
    const size_t   t1 = 100, t2 = 100 + ramp / 2;
    uint32_t       rng = model_seed(55u);
    const double   p0 = expect_from_q30(model_pick(&rng));
    const double   p1 = expect_from_q30(model_pick(&rng));
    const double   p2 = expect_from_q30(model_pick(&rng));
    const double   tol = 1e-4;
    pcl_sah_t      s;
    pcl_event_t    ev[2];
    size_t         i;
    int            monotone = 1;

    ev[0].offset = (uint16_t)t1;
    ev[0].type = PCL_EVT_CLOCK;
    ev[0].note = 0;
    ev[0].velocity = 0;
    ev[1].offset = (uint16_t)t2;
    ev[1].type = PCL_EVT_CLOCK;
    ev[1].note = 0;
    ev[1].velocity = 0;

    /* single trigger */
    make_sah(&s, 0.0, slew_ms, 55u);
    pcl_sah_process(&s, g_out, NULL, ev, 1, 2000);
    PCL_CHECK_NEAR(pcl_test_to_double(g_out[t1 - 1]), p0, tol);
    PCL_CHECK_NEAR(pcl_test_to_double(g_out[t1]), p0, tol); /* still the old value */
    PCL_CHECK_NEAR(pcl_test_to_double(g_out[t1 + ramp / 2]), 0.5 * (p0 + p1), tol); /* linear */
    PCL_CHECK_NEAR(pcl_test_to_double(g_out[t1 + ramp]), p1, tol); /* arrives on time */
    PCL_CHECK_NEAR(pcl_test_to_double(g_out[1999]), p1, tol);
    for (i = t1; i + 1 <= t1 + ramp; i++)
    {
        const double a = pcl_test_to_double(g_out[i]), b = pcl_test_to_double(g_out[i + 1]);
        if ((p1 > p0 && b < a) || (p1 < p0 && b > a))
            monotone = 0;
    }
    PCL_CHECK(monotone);
    PCL_CHECK(fabs(pcl_test_to_double(g_out[t1 + ramp - 2]) - p1) > 0.0 || p0 == p1);

    /* retrigger half-way: continues from the current output toward the new value */
    make_sah(&s, 0.0, slew_ms, 55u);
    pcl_sah_process(&s, g_out, NULL, ev, 2, 3000);
    {
        const double mid = 0.5 * (p0 + p1);
        PCL_CHECK_NEAR(pcl_test_to_double(g_out[t2]), mid, tol);
        PCL_CHECK_NEAR(pcl_test_to_double(g_out[t2 + ramp / 2]), 0.5 * (mid + p2), tol);
        PCL_CHECK_NEAR(pcl_test_to_double(g_out[t2 + ramp]), p2, tol);
    }

    /* set_slew_ms(0) mid-ramp jumps to where the ramp was heading */
    make_sah(&s, 0.0, slew_ms, 55u);
    pcl_sah_process(&s, g_out, NULL, ev, 1, t1 + 10);
    PCL_CHECK_EQ_INT(pcl_sah_set_slew_ms(&s, 0u), PCL_OK);
    pcl_sah_process(&s, g_out, NULL, NULL, 0, 1);
    PCL_CHECK_NEAR(pcl_test_to_double(g_out[0]), p1, tol);
}

static void test_sah_distribution(void)
{
    enum
    {
        BINS = 16,
        PICKS = 100000
    };
    pcl_sah_t    s;
    unsigned     hist[BINS];
    double       mean = 0.0;
    size_t       i, done = 0, bins_ok = 1;
    pcl_sample_t out[BLOCK];

    memset(hist, 0, sizeof hist);
    make_sah(&s, 24000.0, 0u, 8u); /* a pick every second sample */
    while (done < 2u * PICKS)
    {
        pcl_sah_process(&s, out, NULL, NULL, 0, BLOCK);
        for (i = 0; i < BLOCK; i += 2)
        {
            const double v = pcl_test_to_double(out[i]);
            int          bin = (int)floor((v + 1.0) * 0.5 * BINS);
            bin = bin < 0 ? 0 : (bin >= BINS ? BINS - 1 : bin);
            hist[bin]++;
            mean += v;
        }
        done += BLOCK;
    }
    {
        const double total = (double)(done / 2), p = 1.0 / BINS;
        const double sigma = sqrt(total * p * (1.0 - p));
        for (i = 0; i < BINS; i++)
            if (fabs((double)hist[i] - total * p) > 5.0 * sigma)
                bins_ok = 0;
        PCL_CHECK(bins_ok);
        PCL_CHECK(fabs(mean / total) < 5.0 * 0.5774 / sqrt(total));
    }
}

int main(void)
{
    PCL_RUN_TEST(test_init_validation);
    PCL_RUN_TEST(test_white_known_value);
    PCL_RUN_TEST(test_matches_models);
    PCL_RUN_TEST(test_seed_semantics);
    PCL_RUN_TEST(test_reset_and_set_colour_determinism);
    PCL_RUN_TEST(test_block_size_independence);
    PCL_RUN_TEST(test_levels);
    PCL_RUN_TEST(test_white_mean_and_flat_spectrum);
    PCL_RUN_TEST(test_pink_spectrum);
    PCL_RUN_TEST(test_brown_spectrum);
    PCL_RUN_TEST(test_ten_minutes_no_saturation);
    PCL_RUN_TEST(test_sah_validation);
    PCL_RUN_TEST(test_sah_clock_rate);
    PCL_RUN_TEST(test_sah_rate_zero_is_trigger_only);
    PCL_RUN_TEST(test_sah_event_offsets);
    PCL_RUN_TEST(test_sah_event_filtering);
    PCL_RUN_TEST(test_sah_gate_input);
    PCL_RUN_TEST(test_sah_all_sources_and_block_independence);
    PCL_RUN_TEST(test_sah_determinism);
    PCL_RUN_TEST(test_sah_slew);
    PCL_RUN_TEST(test_sah_distribution);
    return pcl_test_summary("noise");
}
