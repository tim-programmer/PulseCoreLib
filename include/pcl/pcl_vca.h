/*
 * pcl_vca.h - voltage-controlled amplifier: a gain with a modulation input.
 *
 *     out[n] = in[n] * mod[n] * gain
 *
 * `gain` is a linear factor, 0 <= gain < 2 (just under +6 dB). `mod` is an
 * optional normalized [-1, 1] signal (an envelope, an LFO for tremolo, ...);
 * a negative mod inverts the signal, which makes ring modulation possible.
 * With mod == NULL the VCA is a plain smoothed gain.
 *
 * Rate: audio. The gain is smoothed with a linear ramp of PCL_SMOOTH_MS (~5 ms).
 * Results that exceed full scale saturate in Q15/Q31 builds; the float build does
 * not clip. Inputs are not sanitized: feed finite samples.
 */
#ifndef PCL_VCA_H
#define PCL_VCA_H

#include "pcl/pcl_smooth.h"
#include "pcl/pcl_types.h"

PCL_BEGIN_DECLS

typedef struct
{
    uint32_t   sample_rate_hz; /* PCL_SAMPLE_RATE_MIN_HZ .. PCL_SAMPLE_RATE_MAX_HZ */
    pcl_coef_t gain;           /* linear, 0 .. <2; build with PCL_COEF(x)          */
} pcl_vca_config_t;

typedef struct
{
    /* private - callers must not touch fields directly */
    pcl_smooth_t gain;
} pcl_vca_t;

/*
 * Initializes `vca` with the configured gain, already settled (no initial ramp).
 * Returns PCL_ERR_NULL_POINTER for NULL arguments and PCL_ERR_OUT_OF_RANGE for a
 * sample rate or gain outside the ranges above (NaN gain included). `vca` is left
 * untouched on error. Not for use from the audio thread.
 */
pcl_status_t pcl_vca_init(pcl_vca_t *vca, const pcl_vca_config_t *cfg);

/*
 * Cancels any gain ramp in progress and jumps to the target gain. The target
 * itself is configuration and is kept. Control-thread call.
 */
void pcl_vca_reset(pcl_vca_t *vca);

/*
 * Sets the target gain; the output ramps there over ~PCL_SMOOTH_MS. Same range
 * and errors as the config field; on error the previous gain is kept.
 * Control-thread call.
 */
pcl_status_t pcl_vca_set_gain(pcl_vca_t *vca, pcl_coef_t gain);

/*
 * Processes `frames` samples at audio rate (any count from 1 to PCL_MAX_BLOCK,
 * and beyond). `gain_mod` may be NULL for unmodulated. `in`, `out` and
 * `gain_mod` may alias each other freely (processing is sample by sample).
 * Does not allocate, block or check arguments.
 */
void pcl_vca_process(pcl_vca_t *vca, const pcl_sample_t *in, pcl_sample_t *out,
    const pcl_sample_t *gain_mod, size_t frames);

PCL_END_DECLS

#endif /* PCL_VCA_H */
