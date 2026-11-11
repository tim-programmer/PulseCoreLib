/*
 * pcl_sampler.h - one sample-playback voice: a caller-owned mono sample played at a MIDI note's
 * pitch, with octave pitch modulation, loops (one-shot, forward, ping-pong, sustain-with-release),
 * reverse playback, sample-accurate note events and declicked starts and stops.
 * Design note: docs/modules/sampler.md.
 *
 * The sample data is never copied: pass a pcl_sampler_sample_t that points at it (both may live in
 * flash). The description must stay valid while the voice plays it. Polyphony means several voices;
 * amplitude envelopes come from pcl_adsr + pcl_vca.
 *
 * Rate: audio rate. pitch_mod is sampled at control rate (every PCL_CONTROL_DIV samples, counted
 * from the start of the stream) and the playback step moves linearly in between. Events are
 * sample-accurate.
 *
 * The position is an integer frame plus a 24-bit fraction in every build, so without pitch
 * modulation float, Q15 and Q31 play from exactly the same positions.
 */
#ifndef PCL_SAMPLER_H
#define PCL_SAMPLER_H

#include "pcl/pcl_event.h"
#include "pcl/pcl_smooth.h"
#include "pcl/pcl_types.h"

PCL_BEGIN_DECLS

#define PCL_SAMPLER_MAX_DECLICK_MS 20u

typedef enum
{
    PCL_SAMPLER_Q15 = 0, /* int16_t frames                                              */
    PCL_SAMPLER_NATIVE,  /* pcl_sample_t frames                                         */
    PCL_SAMPLER_FORMAT_COUNT
} pcl_sampler_format_t;

typedef enum
{
    PCL_SAMPLER_ONE_SHOT = 0, /* plays to the end; NOTE_OFF is ignored (drum-style)       */
    PCL_SAMPLER_FORWARD,      /* loops start..end until NOTE_OFF, then fades out           */
    PCL_SAMPLER_PING_PONG,    /* loops back and forth until NOTE_OFF, then fades out       */
    PCL_SAMPLER_SUSTAIN,      /* loops while held; NOTE_OFF plays on past the loop to the end */
    PCL_SAMPLER_LOOP_COUNT
} pcl_sampler_loop_t;

typedef enum
{
    PCL_SAMPLER_LINEAR = 0,
    PCL_SAMPLER_HERMITE, /* 4-point cubic (Catmull-Rom)                                     */
    PCL_SAMPLER_INTERP_COUNT
} pcl_sampler_interp_t;

/* A sample: mono frames in caller memory (flash or RAM) and how to play them. */
typedef struct
{
    const void          *data;
    uint32_t             frames; /* >= 1                                                 */
    pcl_sampler_format_t format;
    uint32_t             sample_rate_hz; /* the recording's rate, 8000 .. 768000                */
    uint8_t              root_note;      /* MIDI note that plays at the recorded pitch, 0 .. 127 */
    pcl_sampler_loop_t   loop_mode;
    uint32_t             loop_start; /* frames; loop modes need loop_end - loop_start >= 2  */
    uint32_t             loop_end;   /* exclusive, <= frames; ignored for ONE_SHOT          */
} pcl_sampler_sample_t;

typedef struct
{
    uint32_t                    sample_rate_hz; /* output rate                                  */
    const pcl_sampler_sample_t *sample;         /* played from the next NOTE_ON                  */
    pcl_sampler_interp_t        interp;
    pcl_oct_t  pitch_depth_oct; /* octaves per unit of pitch_mod, 0 .. <8         */
    pcl_coef_t gain;            /* 0 .. <2, smoothed; PCL_COEF(x)                  */
    pcl_coef_t velocity_sens;   /* 0 (ignore velocity) .. 1 (gain x velocity/127)   */
    uint32_t   start_offset;    /* frames from the start (from the end if reversed)   */
    int        reverse;         /* non-zero: play backwards                           */
    uint32_t   declick_ms;      /* fade at starts and stops, 0 .. 20                  */
} pcl_sampler_config_t;

/* Private: one playhead. A retrigger moves the sounding note to the tail head to fade it out. */
typedef struct
{
    const pcl_sampler_sample_t *smp;
    int64_t                     pos;       /* frames, Q.24                                    */
    uint32_t                    step_note; /* frames per output sample at the note, Q8.24     */
    uint32_t                    step;      /* in use, Q8.24                                    */
    uint32_t                    step_end;
    int32_t                     step_inc; /* per-sample change during the control period      */
    pcl_coef_t                  amp;      /* velocity gain                                    */
    pcl_coef_t                  fade;     /* declick level, 0 .. 1                            */
    pcl_coef_t                  fade_step;
    uint32_t                    fade_left; /* samples left in the fade                         */
    int8_t                      dir;       /* +1 forward, -1 backward                          */
    uint8_t                     active;
    uint8_t                     looping;  /* the loop is in effect                            */
    uint8_t                     stopping; /* fading out; idle when the fade ends              */
} pcl_sampler_head_t;

typedef struct
{
    /* private - callers must not touch fields directly */
    pcl_sampler_head_t          head[2];   /* [0] the current note, [1] a fading tail      */
    const pcl_sampler_sample_t *sample;    /* for the next NOTE_ON                          */
    uint32_t                    base_step; /* sample rate / output rate, Q8.24, for `sample` */
    pcl_smooth_t                gain;
    pcl_oct_t                   pitch_depth;
    int32_t                     mod_oct; /* the last control point's modulation, Q4.28   */
    pcl_coef_t                  velocity_sens;
    uint32_t                    start_offset;
    uint32_t                    declick_len; /* samples                                   */
    uint32_t                    sample_rate_hz;
    uint8_t                     pos; /* samples already output in the control period    */
    uint8_t                     interp;
    uint8_t                     reverse;
} pcl_sampler_t;

/*
 * Initializes `s` (idle). Returns PCL_ERR_NULL_POINTER for NULL `s`, `cfg` or `cfg->sample`, and
 * PCL_ERR_OUT_OF_RANGE for an invalid config or sample description (NULL data, no frames, unknown
 * format or loop mode, root note > 127, loop points out of order or past the end, start_offset >=
 * frames, NaN); `s` is untouched on error. Not for the audio thread.
 */
pcl_status_t pcl_sampler_init(pcl_sampler_t *s, const pcl_sampler_config_t *cfg);

/* Silences the voice at once (no fade) and cancels ramps; the configuration is kept. */
void pcl_sampler_reset(pcl_sampler_t *s);

/*
 * Setters: validate (errors leave `s` unchanged), control-thread calls. A new sample, start offset
 * or reverse setting applies from the next NOTE_ON; a sounding note keeps its own. The gain is
 * smoothed over ~PCL_SMOOTH_MS; the interpolator and the pitch depth switch at once. A start offset
 * beyond a later sample's end is clamped to its last frame.
 */
pcl_status_t pcl_sampler_set_sample(pcl_sampler_t *s, const pcl_sampler_sample_t *sample);
pcl_status_t pcl_sampler_set_interp(pcl_sampler_t *s, pcl_sampler_interp_t interp);
pcl_status_t pcl_sampler_set_pitch_depth(pcl_sampler_t *s, pcl_oct_t octaves);
pcl_status_t pcl_sampler_set_gain(pcl_sampler_t *s, pcl_coef_t gain);
pcl_status_t pcl_sampler_set_velocity_sens(pcl_sampler_t *s, pcl_coef_t sens);
pcl_status_t pcl_sampler_set_start_offset(pcl_sampler_t *s, uint32_t frames);
pcl_status_t pcl_sampler_set_declick(pcl_sampler_t *s, uint32_t declick_ms);
void         pcl_sampler_set_reverse(pcl_sampler_t *s, int reverse);

/*
 * Starts or releases a note now, as an event at offset 0 of the next process call would. A NOTE_ON
 * while a note sounds fades the old one out over declick_ms while the new one fades in. Velocity 0
 * is a NOTE_OFF (MIDI convention). Control-thread calls.
 */
void pcl_sampler_note_on(pcl_sampler_t *s, uint8_t note, uint8_t velocity);
void pcl_sampler_note_off(pcl_sampler_t *s);

/* Non-zero while anything is audible (a note or a fading tail): a voice allocator's test. */
int pcl_sampler_is_playing(const pcl_sampler_t *s);

/*
 * Renders `frames` samples (any count >= 1) to `out`. Optional: `pitch_mod` (normalized [-1, 1],
 * times pitch_depth_oct octaves, control rate) and `events` (num_events, sorted by offset):
 * PCL_EVT_NOTE_ON (note, velocity) and PCL_EVT_NOTE_OFF take effect at their sample offset; other
 * types and offsets >= frames are ignored. `pitch_mod` may alias `out`. Fixed point saturates.
 * Does not allocate, block or check arguments.
 */
void pcl_sampler_process(pcl_sampler_t *s, pcl_sample_t *out, const pcl_sample_t *pitch_mod,
    const pcl_event_t *events, size_t num_events, size_t frames);

PCL_END_DECLS

#endif /* PCL_SAMPLER_H */
