/*
 * pcl_reverb.h - mono-in, stereo-out reverb in caller-provided memory, with two algorithms behind
 * one API. Design note: docs/modules/reverb.md.
 *
 *   PCL_REVERB_SMALL    Schroeder-Moorer: 4 parallel combs with damping in their feedback, summed
 *                       with two sign patterns into a pair of all-pass chains (one per channel).
 *                       About 12 KB at 48 kHz with 16-bit storage.
 *   PCL_REVERB_QUALITY  8-line feedback delay network (Jot) with a Hadamard matrix, per-line
 *                       damping, 4 input-diffusion all-passes and 2 slowly modulated lines.
 *                       About 44 KB at 48 kHz with 16-bit storage.
 *
 *     size_t bytes = pcl_reverb_required_bytes(&cfg);    -> caller provides `bytes` of memory
 *     pcl_reverb_init(&rv, &cfg, buffer, bytes);
 *
 * The decay is set as an exact RT60 (the time to fall by 60 dB) below hf_hz; above it the tail
 * decays hf_ratio times faster. Line lengths scale with `size` and are distinct primes, so no two
 * echo patterns coincide.
 *
 * Rate: audio rate. The QUALITY line modulation runs at control rate (every PCL_CONTROL_DIV
 * samples, counted from the start of the stream) with the read positions moving linearly in
 * between. Mix, width and the freeze input fade are smoothed per sample over ~PCL_SMOOTH_MS; decay,
 * damping and predelay changes take effect at once.
 *
 * Fixed point: the loops run in Q4.28 and round toward zero; 16-bit lines round stochastically
 * (unbiased, so the decay time does not depend on the level). A tail always decays to exactly 0 (no
 * limit cycles). Lines saturate at full scale.
 */
#ifndef PCL_REVERB_H
#define PCL_REVERB_H

#include "pcl/pcl_delay.h"
#include "pcl/pcl_smooth.h"
#include "pcl/pcl_types.h"

PCL_BEGIN_DECLS

#define PCL_REVERB_MAX_PREDELAY_MS 500u
#define PCL_REVERB_RT60_MIN_MS 100
#define PCL_REVERB_RT60_MAX_MS 30000
#define PCL_REVERB_SIZE_MIN 0.25
#define PCL_REVERB_HF_RATIO_MIN 0.1
#define PCL_REVERB_HF_MIN_HZ 1000 /* hf_hz: 1000 .. 0.45 * sample rate                       */

#define PCL_REVERB_TANK_LINES 8 /* QUALITY                                                  */
#define PCL_REVERB_COMBS 4      /* SMALL                                                    */
#define PCL_REVERB_ALLPASSES 4  /* SMALL: 2 per channel; QUALITY: the input diffusers       */
#define PCL_REVERB_MOD_LINES 2  /* QUALITY lines with modulated length                      */

typedef enum
{
    PCL_REVERB_SMALL = 0,
    PCL_REVERB_QUALITY,
    PCL_REVERB_VARIANT_COUNT
} pcl_reverb_variant_t;

typedef struct
{
    uint32_t             sample_rate_hz;  /* PCL_SAMPLE_RATE_MIN_HZ .. PCL_SAMPLE_RATE_MAX_HZ   */
    pcl_reverb_variant_t variant;         /* fixed at init                                      */
    pcl_coef_t           size;            /* 0.25 .. 1: room size and memory; fixed at init     */
    uint32_t             max_predelay_ms; /* 0 .. PCL_REVERB_MAX_PREDELAY_MS; fixed at init     */
    pcl_ms_t             predelay_ms;     /* 0 .. max_predelay_ms; PCL_MS(x)                    */
    pcl_ms_t             rt60_ms;         /* 100 .. 30000: decay to -60 dB below hf_hz          */
    pcl_coef_t           hf_ratio;        /* 0.1 .. 1: RT60 above hf_hz / rt60_ms             */
    pcl_hz_t             hf_hz;           /* 1000 .. 0.45 fs: where the damping takes over      */
    pcl_coef_t           width;           /* 0 (mono wet) .. 1 (full stereo)                    */
    pcl_coef_t           mix;             /* 0 (dry) .. 1 (wet only), linear                    */
    pcl_coef_t           mod_depth;       /* 0 .. 1: QUALITY line modulation (ignored by SMALL) */
    int                  freeze;          /* non-zero: hold the tail (input and mod. off)      */
    pcl_delay_storage_t  storage;         /* NATIVE, or Q15 (int16_t) for half the RAM          */
} pcl_reverb_config_t;

/* One circular line in the caller's buffer (private). */
typedef struct
{
    void    *buf;
    uint32_t length; /* samples                                                          */
    uint32_t w;      /* next write index                                                 */
    uint32_t rng;    /* dither for 16-bit writes (LCG)                                   */
} pcl_reverb_line_t;

typedef struct
{
    /* private - callers must not touch fields directly */
    pcl_reverb_line_t pre;                          /* predelay       */
    pcl_reverb_line_t tank[PCL_REVERB_TANK_LINES];  /* combs or FDN   */
    pcl_reverb_line_t ap[PCL_REVERB_ALLPASSES];     /* all-passes     */
    uint32_t          delay[PCL_REVERB_TANK_LINES]; /* nominal tank delays, samples          */
    pcl_wide_t        lp[PCL_REVERB_TANK_LINES];    /* damping low-pass states, Q4.28        */
    pcl_coef_t        g_hf[PCL_REVERB_TANK_LINES];  /* high-band loop gain, Q2.30            */
    pcl_coef_t        g_lf[PCL_REVERB_TANK_LINES];  /* low-band gain minus g_hf, Q2.30       */
    pcl_coef_t        lp_pole;                      /* damping low-pass pole, Q2.30          */
    uint32_t          mod_d[PCL_REVERB_MOD_LINES];  /* read delay, Q20.12 samples            */
    uint32_t          mod_d_end[PCL_REVERB_MOD_LINES];
    int32_t           mod_d_step[PCL_REVERB_MOD_LINES];
    uint32_t          mod_amp;   /* modulation excursion in use, Q20.12 samples            */
    uint32_t          mod_max;   /* excursion at mod_depth 1, Q20.12 samples               */
    uint32_t          lfo_phase; /* modulation LFO, 2^32 = one cycle                       */
    uint32_t          lfo_inc;   /* per control period                                     */
    uint32_t          pre_d;     /* predelay in samples                                    */
    pcl_ms_t          rt60_ms;
    pcl_coef_t        hf_ratio;
    pcl_smooth_t      mix;
    pcl_smooth_t      width;
    pcl_smooth_t      input; /* 1, or 0 while frozen                                       */
    uint32_t          sample_rate_hz;
    uint32_t          max_predelay_ms;
    uint8_t           variant;
    uint8_t           storage;
    uint8_t           freeze;
    uint8_t           pos; /* samples already output in the current control period           */
} pcl_reverb_t;

/*
 * Bytes of caller memory `cfg` needs (including slack for alignment, so any byte buffer works), or
 * 0 if `cfg` is NULL or invalid. Depends on the variant, size, max_predelay_ms, sample rate and
 * storage. Not for the audio thread.
 */
size_t pcl_reverb_required_bytes(const pcl_reverb_config_t *cfg);

/*
 * Initializes `rv` on `buffer` (`bytes` long) and clears every line. Returns PCL_ERR_NULL_POINTER
 * for NULL arguments, PCL_ERR_OUT_OF_RANGE for an invalid config (NaN included), and
 * PCL_ERR_BUFFER_TOO_SMALL if `bytes` < pcl_reverb_required_bytes(cfg); `rv` is untouched on error.
 * The buffer must stay valid and unused by anything else while `rv` is in use. Not for the audio
 * thread (it searches for prime line lengths).
 */
pcl_status_t pcl_reverb_init(
    pcl_reverb_t *rv, const pcl_reverb_config_t *cfg, void *buffer, size_t bytes);

/* Clears every line and filter state and cancels every ramp. O(memory). */
void pcl_reverb_reset(pcl_reverb_t *rv);

/*
 * Setters: validate against the ranges of pcl_reverb_config_t (errors leave the reverb unchanged)
 * and are control-thread calls. Mix, width and freeze's input fade ramp over ~PCL_SMOOTH_MS; the
 * rest switch at once. set_mod_depth is accepted but has no effect on SMALL.
 */
pcl_status_t pcl_reverb_set_predelay(pcl_reverb_t *rv, pcl_ms_t predelay_ms);
pcl_status_t pcl_reverb_set_rt60(pcl_reverb_t *rv, pcl_ms_t rt60_ms);
pcl_status_t pcl_reverb_set_damping(pcl_reverb_t *rv, pcl_coef_t hf_ratio, pcl_hz_t hf_hz);
pcl_status_t pcl_reverb_set_width(pcl_reverb_t *rv, pcl_coef_t width);
pcl_status_t pcl_reverb_set_mix(pcl_reverb_t *rv, pcl_coef_t mix);
pcl_status_t pcl_reverb_set_mod_depth(pcl_reverb_t *rv, pcl_coef_t mod_depth);
void         pcl_reverb_set_freeze(pcl_reverb_t *rv, int freeze);

/*
 * Processes `frames` samples (any count >= 1): mono `in`, stereo `out_l` / `out_r`. With `out_r`
 * NULL, `out_l` gets the mono sum (dry + mix * (wet_l + wet_r) / 2). `in` may alias either output.
 * Fixed point saturates the lines and the outputs; float clamps the lines at +-4. Does not
 * allocate, block or check arguments.
 */
void pcl_reverb_process(pcl_reverb_t *rv, const pcl_sample_t *in, pcl_sample_t *out_l,
    pcl_sample_t *out_r, size_t frames);

PCL_END_DECLS

#endif /* PCL_REVERB_H */
