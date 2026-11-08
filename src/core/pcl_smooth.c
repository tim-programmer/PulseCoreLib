#include "pcl/pcl_smooth.h"

#define PCL_MS_PER_SECOND 1000u

/* ramp_samples = rate * ms / 1000 must not overflow uint32_t. */
PCL_STATIC_ASSERT(((unsigned long long)(PCL_SMOOTH_MS) * (PCL_SAMPLE_RATE_MAX_HZ)) <= 0xFFFFFFFFull,
    smooth_ms_fits_uint32);

uint32_t pcl_smooth_ramp_samples(uint32_t sample_rate_hz)
{
    uint32_t samples
        = (sample_rate_hz * (uint32_t)PCL_SMOOTH_MS + PCL_MS_PER_SECOND / 2u) / PCL_MS_PER_SECOND;
    return samples > 0u ? samples : 1u;
}

void pcl_smooth_init(pcl_smooth_t *s, pcl_coef_t value, uint32_t ramp_samples)
{
    s->current = value;
    s->target = value;
    s->step = 0;
    s->ramp_samples = ramp_samples > 0u ? ramp_samples : 1u;
    s->remaining = 0u;
}

void pcl_smooth_reset(pcl_smooth_t *s)
{
    s->current = s->target;
    s->step = 0;
    s->remaining = 0u;
}

void pcl_smooth_set_target(pcl_smooth_t *s, pcl_coef_t target)
{
    s->target = target;

    if (s->ramp_samples <= 1u || target == s->current)
    {
        s->current = target;
        s->step = 0;
        s->remaining = 0u;
        return;
    }

#if PCL_SAMPLE_TYPE == PCL_FLOAT
    s->step = (target - s->current) / (float)s->ramp_samples;
#else
    /* The difference can exceed int32_t, so divide in 64 bits (integer only). */
    s->step = (int32_t)(((int64_t)target - (int64_t)s->current) / (int64_t)s->ramp_samples);
#endif
    s->remaining = s->ramp_samples;
}
