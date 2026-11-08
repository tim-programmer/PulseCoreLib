#include "pcl/pcl_vca.h"

/* Gain is limited to [0, 2): the Q2.30 range, applied to float builds too for parity. */
#define PCL_VCA_GAIN_LIMIT_F 2.0f

static int vca_gain_valid(pcl_coef_t gain)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    /* Written positively so that NaN fails both comparisons. */
    return gain >= 0.0f && gain < PCL_VCA_GAIN_LIMIT_F;
#else
    return gain >= 0;
#endif
}

/* x * m, both normalized samples. */
static pcl_sample_t vca_modulate(pcl_sample_t x, pcl_sample_t m)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return x * m;
#elif PCL_SAMPLE_TYPE == PCL_Q15
    return pcl_mul_q15(x, m);
#else
    return pcl_mul_q31(x, m);
#endif
}

/* x * g, with g a pcl_coef_t. */
static pcl_sample_t vca_scale(pcl_sample_t x, pcl_coef_t g)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return x * g;
#elif PCL_SAMPLE_TYPE == PCL_Q15
    return pcl_mul_q15_q30(x, g);
#else
    return pcl_mul_q31_q30(x, g);
#endif
}

pcl_status_t pcl_vca_init(pcl_vca_t *vca, const pcl_vca_config_t *cfg)
{
    if (vca == NULL || cfg == NULL)
        return PCL_ERR_NULL_POINTER;
    if (cfg->sample_rate_hz < PCL_SAMPLE_RATE_MIN_HZ || cfg->sample_rate_hz > PCL_SAMPLE_RATE_MAX_HZ
        || !vca_gain_valid(cfg->gain))
        return PCL_ERR_OUT_OF_RANGE;

    pcl_smooth_init(&vca->gain, cfg->gain, pcl_smooth_ramp_samples(cfg->sample_rate_hz));
    return PCL_OK;
}

void pcl_vca_reset(pcl_vca_t *vca) { pcl_smooth_reset(&vca->gain); }

pcl_status_t pcl_vca_set_gain(pcl_vca_t *vca, pcl_coef_t gain)
{
    if (vca == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!vca_gain_valid(gain))
        return PCL_ERR_OUT_OF_RANGE;

    pcl_smooth_set_target(&vca->gain, gain);
    return PCL_OK;
}

void pcl_vca_process(pcl_vca_t *vca, const pcl_sample_t *in, pcl_sample_t *out,
    const pcl_sample_t *gain_mod, size_t frames)
{
    /* Work on a local copy: stores to `out` could otherwise force the compiler to
     * reload the smoother state from memory on every sample. */
    pcl_smooth_t gain = vca->gain;
    size_t       i;

    if (gain_mod != NULL)
    {
        for (i = 0; i < frames; i++)
            out[i] = vca_scale(vca_modulate(in[i], gain_mod[i]), pcl_smooth_next(&gain));
    }
    else
    {
        for (i = 0; i < frames; i++)
            out[i] = vca_scale(in[i], pcl_smooth_next(&gain));
    }

    vca->gain = gain;
}
