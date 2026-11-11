/*
 * pcl_unison.h - unison oscillator: up to PCL_UNISON_MAX_VOICES band-limited oscillators,
 * detuned symmetrically around a base pitch, mixed to mono or spread across a stereo field.
 *
 * Each voice is a full pcl_osc_t (see pcl_osc.h), so band-limiting, PWM, hard sync, the
 * control-rate pitch path and the one-sample latency are exactly those of the oscillator.
 * Design note: docs/modules/unison.md.
 *
 * Rate: audio rate. Pitch and PWM modulation are control rate inside each voice, as in
 * pcl_osc. Stereo-width changes ramp the pan gains per sample over ~PCL_SMOOTH_MS.
 *
 * Voice i of N (i = 0 .. N-1) is detuned by detune_oct * (i / (N-1) - 1/2) octaves and
 * panned to stereo_width * (2i / (N-1) - 1), with a constant-power pan law. The mix is
 * scaled by 1 / sqrt(N). Voice i starts at phase = the (i+1)-th xorshift32 draw from the
 * seed (13/17/5 shifts; seed 0 means PCL_NOISE_DEFAULT_SEED), and reset restores those phases.
 */
#ifndef PCL_UNISON_H
#define PCL_UNISON_H

#include "pcl/pcl_osc.h"
#include "pcl/pcl_types.h"

PCL_BEGIN_DECLS

/* Compile-time voice limit: the voices live inside pcl_unison_t. 1 .. 16. */
#ifndef PCL_UNISON_MAX_VOICES
#define PCL_UNISON_MAX_VOICES 8
#endif

/* Frames rendered per voice at a time inside process (a fixed stack buffer). */
#ifndef PCL_UNISON_CHUNK
#define PCL_UNISON_CHUNK 32
#endif

typedef struct
{
    pcl_osc_config_t osc;          /* shared by every voice; osc.frequency is the base pitch    */
    size_t           voices;       /* 1 .. PCL_UNISON_MAX_VOICES                                */
    pcl_oct_t        detune_oct;   /* lowest to highest voice, 0 .. 1 octave; PCL_OCT(x)        */
    pcl_coef_t       stereo_width; /* 0 (all centred) .. 1 (outer voices hard L/R); PCL_COEF(x) */
    uint32_t         seed;         /* start phases; any value, 0 means PCL_NOISE_DEFAULT_SEED   */
} pcl_unison_config_t;

typedef struct
{
    /* private - callers must not touch fields directly */
    pcl_osc_t  voice[PCL_UNISON_MAX_VOICES];
    pcl_coef_t gain_l[PCL_UNISON_MAX_VOICES]; /* mix * pan gain; Q2.30 in fixed point      */
    pcl_coef_t gain_r[PCL_UNISON_MAX_VOICES];
    pcl_coef_t step_l[PCL_UNISON_MAX_VOICES]; /* per-sample ramp step while ramp_left > 0  */
    pcl_coef_t step_r[PCL_UNISON_MAX_VOICES];
    pcl_coef_t target_l[PCL_UNISON_MAX_VOICES];
    pcl_coef_t target_r[PCL_UNISON_MAX_VOICES];
    pcl_coef_t gain_mono; /* 1 / sqrt(N), Q2.30 in fixed point                            */
    pcl_hz_t   base_hz;
    pcl_oct_t  detune;
    pcl_coef_t width;
    uint32_t   seed;
    uint32_t   ramp_samples; /* length of a width ramp                                     */
    uint32_t   ramp_left;    /* samples left in the current width ramp                     */
    uint8_t    voices;
} pcl_unison_t;

/*
 * Initializes `u`: every voice settled at its detuned frequency (no initial glide), at its
 * seeded start phase. Returns PCL_ERR_NULL_POINTER for NULL arguments, PCL_ERR_OUT_OF_RANGE
 * for any field outside its range (the `osc` fields as in pcl_osc_init); `u` is left
 * untouched on error. Not for the audio thread.
 */
pcl_status_t pcl_unison_init(pcl_unison_t *u, const pcl_unison_config_t *cfg);

/*
 * Restores every voice's seeded start phase, cancels all frequency, width and pan-gain
 * ramps and clears the voices' output delay lines. Configuration is kept.
 */
void pcl_unison_reset(pcl_unison_t *u);

/*
 * Base frequency in Hz, 0 .. 0.45 * sample rate. Voices glide to their detuned frequencies
 * over ~PCL_SMOOTH_MS; a voice detuned above 0.45 * sample rate is held there. Control-thread
 * call (one 64-bit divide per voice). Errors leave the previous value in place.
 */
pcl_status_t pcl_unison_set_frequency(pcl_unison_t *u, pcl_hz_t hz);

/* Detune, lowest to highest voice, in octaves 0 .. 1. Voices glide to the new spread. */
pcl_status_t pcl_unison_set_detune(pcl_unison_t *u, pcl_oct_t octaves);

/*
 * Active voice count, 1 .. PCL_UNISON_MAX_VOICES. Takes effect at once: detune offsets glide,
 * pan and mix gains jump, and a re-enabled voice resumes where it stopped (may click).
 */
pcl_status_t pcl_unison_set_voices(pcl_unison_t *u, size_t voices);

/* Stereo width 0 .. 1; the pan gains ramp linearly over ~PCL_SMOOTH_MS. */
pcl_status_t pcl_unison_set_stereo_width(pcl_unison_t *u, pcl_coef_t width);

/* As the pcl_osc setters, applied to every voice (also to inactive ones). */
pcl_status_t pcl_unison_set_waveform(pcl_unison_t *u, pcl_osc_waveform_t waveform);
pcl_status_t pcl_unison_set_pitch_depth(pcl_unison_t *u, pcl_oct_t octaves);
pcl_status_t pcl_unison_set_pulse_width(pcl_unison_t *u, pcl_coef_t width);
pcl_status_t pcl_unison_set_pwm_depth(pcl_unison_t *u, pcl_coef_t depth);

/*
 * Renders `frames` samples (any count >= 1).
 *   out_l     required. With out_r == NULL it receives the mono mix (no pan gains: every
 *             voice at 1 / sqrt(N)); otherwise the left channel.
 *   out_r     right channel, or NULL for mono.
 *   pitch_mod, pwm_mod, sync_in   optional, shared by every voice; exactly as in
 *             pcl_osc_process.
 * Any output may alias any input; out_l and out_r must not alias each other. In fixed point
 * the mix saturates (in-phase peaks of several voices can exceed full scale); float does
 * not clip. Does not allocate, block or check arguments.
 */
void pcl_unison_process(pcl_unison_t *u, pcl_sample_t *out_l, pcl_sample_t *out_r,
    const pcl_sample_t *pitch_mod, const pcl_sample_t *pwm_mod, const pcl_sample_t *sync_in,
    size_t frames);

PCL_END_DECLS

#endif /* PCL_UNISON_H */
