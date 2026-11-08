/* Tests for the VCA (gain with modulation input). */
#include <string.h>

#include "pcl_test.h"

#define SAMPLE_RATE_HZ 48000u
#define N_FRAMES 1024u

static void make_vca(pcl_vca_t *vca, double gain)
{
    pcl_vca_config_t cfg;
    cfg.sample_rate_hz = SAMPLE_RATE_HZ;
    cfg.gain = PCL_COEF(gain);
    PCL_CHECK_EQ_INT(pcl_vca_init(vca, &cfg), PCL_OK);
}

static void fill_noise(pcl_sample_t *buf, size_t n, uint32_t seed, double amplitude)
{
    size_t i;
    for (i = 0; i < n; i++)
        buf[i] = pcl_test_from_double(pcl_test_rand(&seed) * amplitude);
}

/* Runs the ramp to completion so the gain under test is steady. */
static void settle(pcl_vca_t *vca)
{
    pcl_sample_t zeros[N_FRAMES] = { 0 };
    pcl_sample_t sink[N_FRAMES];
    pcl_vca_process(vca, zeros, sink, NULL, pcl_smooth_ramp_samples(SAMPLE_RATE_HZ) + 1u);
}

/* ---- init / setter validation ---- */

static void test_init_validation(void)
{
    pcl_vca_t        vca;
    pcl_vca_config_t cfg;

    cfg.sample_rate_hz = SAMPLE_RATE_HZ;
    cfg.gain = PCL_COEF(0.5);

    PCL_CHECK_EQ_INT(pcl_vca_init(&vca, &cfg), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_vca_init(NULL, &cfg), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_vca_init(&vca, NULL), PCL_ERR_NULL_POINTER);

    cfg.sample_rate_hz = 0u;
    PCL_CHECK_EQ_INT(pcl_vca_init(&vca, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.sample_rate_hz = PCL_SAMPLE_RATE_MIN_HZ - 1u;
    PCL_CHECK_EQ_INT(pcl_vca_init(&vca, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.sample_rate_hz = PCL_SAMPLE_RATE_MAX_HZ + 1u;
    PCL_CHECK_EQ_INT(pcl_vca_init(&vca, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.sample_rate_hz = PCL_SAMPLE_RATE_MIN_HZ;
    PCL_CHECK_EQ_INT(pcl_vca_init(&vca, &cfg), PCL_OK);
    cfg.sample_rate_hz = PCL_SAMPLE_RATE_MAX_HZ;
    PCL_CHECK_EQ_INT(pcl_vca_init(&vca, &cfg), PCL_OK);

    cfg.sample_rate_hz = SAMPLE_RATE_HZ;
    cfg.gain = PCL_COEF(0.0);
    PCL_CHECK_EQ_INT(pcl_vca_init(&vca, &cfg), PCL_OK);
    cfg.gain = PCL_COEF(1.99);
    PCL_CHECK_EQ_INT(pcl_vca_init(&vca, &cfg), PCL_OK);
    cfg.gain = PCL_COEF(-0.01);
    PCL_CHECK_EQ_INT(pcl_vca_init(&vca, &cfg), PCL_ERR_OUT_OF_RANGE);

#if PCL_SAMPLE_TYPE == PCL_FLOAT
    /* Q2.30 cannot represent these; they only exist in the float build. */
    cfg.gain = 2.0f;
    PCL_CHECK_EQ_INT(pcl_vca_init(&vca, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.gain = 100.0f;
    PCL_CHECK_EQ_INT(pcl_vca_init(&vca, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.gain = NAN;
    PCL_CHECK_EQ_INT(pcl_vca_init(&vca, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.gain = INFINITY;
    PCL_CHECK_EQ_INT(pcl_vca_init(&vca, &cfg), PCL_ERR_OUT_OF_RANGE);
#endif
}

static void test_init_leaves_state_untouched_on_error(void)
{
    pcl_vca_t        vca, before;
    pcl_vca_config_t cfg;

    make_vca(&vca, 0.5);
    before = vca;
    cfg.sample_rate_hz = 0u;
    cfg.gain = PCL_COEF(0.5);
    PCL_CHECK_EQ_INT(pcl_vca_init(&vca, &cfg), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK(memcmp(&vca, &before, sizeof vca) == 0);
}

static void test_set_gain_validation(void)
{
    pcl_vca_t vca, before;

    make_vca(&vca, 0.5);
    before = vca;

    PCL_CHECK_EQ_INT(pcl_vca_set_gain(NULL, PCL_COEF(0.5)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_vca_set_gain(&vca, PCL_COEF(-0.5)), PCL_ERR_OUT_OF_RANGE);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    PCL_CHECK_EQ_INT(pcl_vca_set_gain(&vca, 2.0f), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_vca_set_gain(&vca, NAN), PCL_ERR_OUT_OF_RANGE);
#endif
    PCL_CHECK(memcmp(&vca, &before, sizeof vca) == 0); /* rejected values change nothing */

    PCL_CHECK_EQ_INT(pcl_vca_set_gain(&vca, PCL_COEF(1.0)), PCL_OK);
}

/* ---- behaviour ---- */

static void test_unity_gain_is_exact_passthrough(void)
{
    pcl_vca_t    vca;
    pcl_sample_t in[N_FRAMES], out[N_FRAMES];

    make_vca(&vca, 1.0);
    fill_noise(in, N_FRAMES, 1u, 1.0);
    pcl_vca_process(&vca, in, out, NULL, N_FRAMES);
    PCL_CHECK(memcmp(in, out, sizeof in) == 0);
}

static void test_zero_gain_is_silent(void)
{
    pcl_vca_t    vca;
    pcl_sample_t in[N_FRAMES], out[N_FRAMES];
    size_t       i;

    make_vca(&vca, 0.0);
    fill_noise(in, N_FRAMES, 2u, 1.0);
    pcl_vca_process(&vca, in, out, NULL, N_FRAMES);
    for (i = 0; i < N_FRAMES; i++)
        PCL_CHECK(pcl_test_to_double(out[i]) == 0.0);
}

static void test_modulation_input(void)
{
    pcl_vca_t    vca;
    pcl_sample_t in[8], mod[8], out[8];
    size_t       i;

    make_vca(&vca, 1.0);
    for (i = 0; i < 8; i++)
        in[i] = pcl_test_from_double(0.5);

    /* mod = 0 silences, mod = -1 inverts, mod = 0.5 halves. */
    mod[0] = pcl_test_from_double(0.0);
    mod[1] = pcl_test_from_double(-1.0);
    mod[2] = pcl_test_from_double(0.5);
    mod[3] = pcl_test_from_double(-0.5);
    mod[4] = mod[5] = mod[6] = mod[7] = pcl_test_from_double(0.0);
    pcl_vca_process(&vca, in, out, mod, 8);

    PCL_CHECK_NEAR(pcl_test_to_double(out[0]), 0.0, PCL_TEST_SAMPLE_TOL);
    PCL_CHECK_NEAR(pcl_test_to_double(out[1]), -0.5, PCL_TEST_SAMPLE_TOL);
    PCL_CHECK_NEAR(pcl_test_to_double(out[2]), 0.25, PCL_TEST_SAMPLE_TOL);
    PCL_CHECK_NEAR(pcl_test_to_double(out[3]), -0.25, PCL_TEST_SAMPLE_TOL);
}

static void test_matches_float_reference_when_settled(void)
{
    static const double gains[] = { 0.0, 0.001, 0.25, 0.75, 1.0, 1.5, 1.99 };
    pcl_sample_t        in[N_FRAMES], mod[N_FRAMES], out[N_FRAMES];
    size_t              g, i;

    fill_noise(in, N_FRAMES, 3u, 1.0);
    fill_noise(mod, N_FRAMES, 4u, 1.0);

    for (g = 0; g < sizeof gains / sizeof gains[0]; g++)
    {
        pcl_vca_t vca;
        make_vca(&vca, 0.5);
        PCL_CHECK_EQ_INT(pcl_vca_set_gain(&vca, PCL_COEF(gains[g])), PCL_OK);
        settle(&vca);

        pcl_vca_process(&vca, in, out, mod, N_FRAMES);
        for (i = 0; i < N_FRAMES; i++)
        {
            const double expected
                = pcl_test_clip(pcl_test_to_double(in[i]) * pcl_test_to_double(mod[i]) * gains[g]);
            PCL_CHECK_NEAR(pcl_test_to_double(out[i]), expected, PCL_TEST_SAMPLE_TOL);
        }

        pcl_vca_process(&vca, in, out, NULL, N_FRAMES);
        for (i = 0; i < N_FRAMES; i++)
        {
            const double expected = pcl_test_clip(pcl_test_to_double(in[i]) * gains[g]);
            PCL_CHECK_NEAR(pcl_test_to_double(out[i]), expected, PCL_TEST_SAMPLE_TOL);
        }
    }
}

static void test_gain_change_ramps_without_a_step(void)
{
    pcl_vca_t    vca;
    pcl_sample_t in[N_FRAMES], out[N_FRAMES];
    const size_t ramp = pcl_smooth_ramp_samples(SAMPLE_RATE_HZ);
    size_t       i;

    make_vca(&vca, 0.0);
    for (i = 0; i < N_FRAMES; i++)
        in[i] = pcl_test_from_double(0.5);

    PCL_CHECK_EQ_INT(pcl_vca_set_gain(&vca, PCL_COEF(1.0)), PCL_OK);
    pcl_vca_process(&vca, in, out, NULL, N_FRAMES);

    PCL_CHECK(pcl_test_to_double(out[0]) < 0.01); /* starts near silence: no zipper step */
    for (i = 1; i < ramp; i++)
        PCL_CHECK(pcl_test_to_double(out[i]) >= pcl_test_to_double(out[i - 1])); /* monotonic */
    for (i = ramp - 1; i < N_FRAMES; i++)
        PCL_CHECK(out[i] == in[i]); /* lands exactly after the ramp, then stays */
}

static void test_saturates_instead_of_wrapping(void)
{
    pcl_vca_t    vca;
    pcl_sample_t in[4], out[4];
    size_t       i;

    make_vca(&vca, 1.99);
    in[0] = pcl_test_from_double(1.0); /* full scale (saturated to the max sample) */
    in[1] = pcl_test_from_double(-1.0);
    in[2] = pcl_test_from_double(0.9);
    in[3] = pcl_test_from_double(-0.9);
    pcl_vca_process(&vca, in, out, NULL, 4);

    for (i = 0; i < 4; i++)
    {
        const double expected = pcl_test_clip(pcl_test_to_double(in[i]) * 1.99);
        PCL_CHECK_NEAR(pcl_test_to_double(out[i]), expected, PCL_TEST_SAMPLE_TOL);
        PCL_CHECK((pcl_test_to_double(out[i]) < 0.0) == (in[i] < 0)); /* sign never flips */
    }
}

/* ---- the contract every module's tests must cover (CLAUDE.md) ---- */

static void test_reset_is_deterministic(void)
{
    pcl_vca_t    vca;
    pcl_sample_t in[N_FRAMES], mod[N_FRAMES], out1[N_FRAMES], out2[N_FRAMES];

    fill_noise(in, N_FRAMES, 5u, 0.9);
    fill_noise(mod, N_FRAMES, 6u, 1.0);

    make_vca(&vca, 0.0);
    PCL_CHECK_EQ_INT(pcl_vca_set_gain(&vca, PCL_COEF(0.9)), PCL_OK);
    pcl_vca_process(&vca, in, out1, mod, 10); /* leave a ramp in flight */

    pcl_vca_reset(&vca);
    pcl_vca_process(&vca, in, out1, mod, N_FRAMES);
    pcl_vca_reset(&vca);
    pcl_vca_process(&vca, in, out2, mod, N_FRAMES);
    PCL_CHECK(memcmp(out1, out2, sizeof out1) == 0);
}

static void process_in_blocks(pcl_vca_t *vca, const pcl_sample_t *in, pcl_sample_t *out,
    const pcl_sample_t *mod, size_t block)
{
    size_t pos;
    for (pos = 0; pos < N_FRAMES; pos += block)
    {
        const size_t n = (N_FRAMES - pos < block) ? (N_FRAMES - pos) : block;
        pcl_vca_process(vca, in + pos, out + pos, mod ? mod + pos : NULL, n);
    }
}

static void test_block_size_independence(void)
{
    static const size_t blocks[] = { 1, 7, 256 };
    pcl_sample_t        in[N_FRAMES], mod[N_FRAMES], expected[N_FRAMES], out[N_FRAMES];
    pcl_vca_t           base;
    size_t              b, with_mod;

    fill_noise(in, N_FRAMES, 7u, 0.9);
    fill_noise(mod, N_FRAMES, 8u, 1.0);

    /* The gain ramp (~240 frames) straddles block boundaries for every block size. */
    make_vca(&base, 0.1);
    PCL_CHECK_EQ_INT(pcl_vca_set_gain(&base, PCL_COEF(1.7)), PCL_OK);

    for (with_mod = 0; with_mod < 2; with_mod++)
    {
        const pcl_sample_t *m = with_mod ? mod : NULL;
        pcl_vca_t           ref = base; /* the state is fully contained in the struct, so copy it */
        pcl_vca_process(&ref, in, expected, m, N_FRAMES);

        for (b = 0; b < sizeof blocks / sizeof blocks[0]; b++)
        {
            pcl_vca_t vca = base;
            process_in_blocks(&vca, in, out, m, blocks[b]);
            PCL_CHECK(memcmp(out, expected, sizeof out) == 0);
            PCL_CHECK(memcmp(&vca, &ref, sizeof vca) == 0); /* and the state matches too */
        }
    }
}

static void test_in_place_matches_out_of_place(void)
{
    pcl_sample_t in[N_FRAMES], mod[N_FRAMES], inplace[N_FRAMES], out[N_FRAMES];
    pcl_vca_t    a, b;

    fill_noise(in, N_FRAMES, 9u, 0.9);
    fill_noise(mod, N_FRAMES, 10u, 1.0);
    make_vca(&a, 0.2);
    PCL_CHECK_EQ_INT(pcl_vca_set_gain(&a, PCL_COEF(1.3)), PCL_OK);
    b = a;

    pcl_vca_process(&a, in, out, mod, N_FRAMES);
    memcpy(inplace, in, sizeof in);
    pcl_vca_process(&b, inplace, inplace, mod, N_FRAMES);
    PCL_CHECK(memcmp(out, inplace, sizeof out) == 0);

    /* Modulation aliasing the output is fine too. */
    memcpy(inplace, mod, sizeof mod);
    pcl_vca_process(&b, in, inplace, inplace, N_FRAMES);
}

static void test_extreme_parameters_stay_finite(void)
{
    pcl_vca_t    vca;
    pcl_sample_t in[N_FRAMES], mod[N_FRAMES], out[N_FRAMES];
    size_t       i;
    int          block;

    /* Full-scale square-ish input, full-scale modulation, gain slammed between limits. */
    for (i = 0; i < N_FRAMES; i++)
    {
        in[i] = pcl_test_from_double((i & 1u) ? 1.0 : -1.0);
        mod[i] = pcl_test_from_double((i & 2u) ? 1.0 : -1.0);
    }

    make_vca(&vca, 0.0);
    for (block = 0; block < 400; block++) /* ~400k frames */
    {
        PCL_CHECK_EQ_INT(pcl_vca_set_gain(&vca, PCL_COEF((block & 1) ? 1.99 : 0.0)), PCL_OK);
        pcl_vca_process(&vca, in, out, mod, N_FRAMES);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
        for (i = 0; i < N_FRAMES; i++)
        {
            PCL_CHECK(isfinite(out[i]));
            PCL_CHECK(fpclassify(out[i]) != FP_SUBNORMAL);
        }
#endif
    }
}

int main(void)
{
    PCL_RUN_TEST(test_init_validation);
    PCL_RUN_TEST(test_init_leaves_state_untouched_on_error);
    PCL_RUN_TEST(test_set_gain_validation);
    PCL_RUN_TEST(test_unity_gain_is_exact_passthrough);
    PCL_RUN_TEST(test_zero_gain_is_silent);
    PCL_RUN_TEST(test_modulation_input);
    PCL_RUN_TEST(test_matches_float_reference_when_settled);
    PCL_RUN_TEST(test_gain_change_ramps_without_a_step);
    PCL_RUN_TEST(test_saturates_instead_of_wrapping);
    PCL_RUN_TEST(test_reset_is_deterministic);
    PCL_RUN_TEST(test_block_size_independence);
    PCL_RUN_TEST(test_in_place_matches_out_of_place);
    PCL_RUN_TEST(test_extreme_parameters_stay_finite);
    return pcl_test_summary("vca");
}
