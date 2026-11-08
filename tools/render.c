/*
 * render - writes short listening-test WAV files from the library.
 *
 *     render noise <outdir> [seconds] [seed]
 *     render lfo   <outdir> [seconds] [seed]
 *     render adsr  <outdir> [seconds] [seed]
 *     render svf   <outdir> [seconds] [seed]
 *     render biquad <outdir> [seconds] [seed]
 *
 * Writes <outdir>/white.wav, pink.wav and brown.wav (default 5 seconds each), plus
 * sah_pitch.wav and sah_pitch_slewed.wav: a saw oscillator whose pitch input is driven by
 * the sample-and-hold at 4 Hz, stepped and with 80 ms slew. The output directory must exist.
 * `lfo` writes five patches built from the library's own modules, so they double as integration
 * checks: lfo_tremolo.wav, lfo_vibrato.wav, lfo_pwm.wav, lfo_wobble.wav and lfo_random.wav.
 * `adsr` writes five envelope patches (adsr_pluck, adsr_pad, adsr_perc, adsr_curves,
 * adsr_retrigger): an oscillator or noise through a VCA driven by the ADSR. The percussion patch is
 * triggered by single-sample pulses in a gate buffer, the others by NOTE_ON / NOTE_OFF events. All
 * files are mono, 16-bit PCM at 48 kHz, in whichever sample type the build uses.
 * `svf` writes four filter patches: svf_sweep (a saw through a resonant low-pass swept by an
 * ADSR), svf_wah (noise through a band-pass swept by an LFO), svf_selfosc (impulses into a
 * self-oscillating filter: a sine at the cutoff, glided from 440 to 880 Hz) and svf_voice (the
 * first complete voice: oscillator -> state-variable filter -> VCA, a filter envelope and an
 * amplitude envelope, four notes).
 * `biquad` writes four patches: biquad_eq_sweep (a +12 dB peaking EQ swept from 100 Hz to 8 kHz
 * over white noise), biquad_tilt (a saw with a low shelf, switched half-way to a high shelf with
 * set_type), biquad_notch (noise through a notch swept by an LFO through cutoff_mod) and
 * biquad_butter4 (noise through two low-pass instances: a 4th-order Butterworth at 1 kHz).
 *
 * `events` writes three patches that run the whole event chain into a voice (saw -> state-variable
 * low-pass -> VCA, with a filter and an amplitude envelope driven by the same note events):
 * events_arp (a clock at 120 bpm and an arpeggiator playing four chords, each in a different mode),
 * events_acid (a 16-step sequence with ties for slides, rests and a swung feel that goes up a
 * fourth and faster half-way) and events_pingpong (an eight-note pentatonic run that turns random
 * half-way). The pitch changes on the sample of each NOTE_ON.
 *
 * The three colours are scaled by a fixed gain each so that all have the same rms (-20 dBFS):
 * they can be compared by ear without touching the volume. The gains and the resulting peaks
 * are printed. Nothing here is part of the library: it is a host utility with file I/O.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pcl/pcl.h"

#define SAMPLE_RATE_HZ 48000u
#define BLOCK_FRAMES 256u
#define DEFAULT_SECONDS 5.0
#define MIN_SECONDS 0.05
#define MAX_SECONDS 600.0
#define TARGET_RMS 0.1 /* -20 dBFS */
#define PATH_MAX_LEN 1024u

#define SAH_RATE_HZ 4.0
#define SAH_SLEW_MS 80u
#define SAH_BASE_HZ 220.0
#define SAH_DEPTH_OCT 1.0
#define SAH_OUTPUT_GAIN 0.3

static double sample_to_double(pcl_sample_t x)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return (double)x;
#elif PCL_SAMPLE_TYPE == PCL_Q15
    return (double)x / 32768.0;
#else
    return (double)x / 2147483648.0;
#endif
}

static pcl_sample_t sample_from_double(double v)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return (float)v;
#elif PCL_SAMPLE_TYPE == PCL_Q15
    return (pcl_sample_t)floor(v * 32768.0 + 0.5);
#else
    return (pcl_sample_t)floor(v * 2147483648.0 + 0.5);
#endif
}

/* Runtime conversions for the host tool (the PCL_HZ/PCL_OCT/PCL_COEF macros are for constants). */
static pcl_hz_t hz_from_double(double hz)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return (float)hz;
#else
    return (pcl_hz_t)(hz * 65536.0 + 0.5);
#endif
}

static pcl_oct_t oct_from_double(double oct)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return (float)oct;
#else
    return (pcl_oct_t)floor(oct * 268435456.0 + 0.5);
#endif
}

static pcl_coef_t coef_from_double(double c)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return (float)c;
#else
    return (pcl_coef_t)floor(c * 1073741824.0 + 0.5);
#endif
}

static int16_t pcm16_from_double(double v)
{
    const double s = floor(v * 32767.0 + 0.5);
    return s >= 32767.0 ? INT16_MAX : (s <= -32768.0 ? INT16_MIN : (int16_t)s);
}

/* ---- WAV output (same layout as examples/sweep_wav.c) ---- */

static void put_le16(unsigned char *p, unsigned v)
{
    p[0] = (unsigned char)(v & 0xFFu);
    p[1] = (unsigned char)((v >> 8) & 0xFFu);
}

static void put_le32(unsigned char *p, uint32_t v)
{
    p[0] = (unsigned char)(v & 0xFFu);
    p[1] = (unsigned char)((v >> 8) & 0xFFu);
    p[2] = (unsigned char)((v >> 16) & 0xFFu);
    p[3] = (unsigned char)((v >> 24) & 0xFFu);
}

static int write_wav_header(FILE *f, uint32_t frames)
{
    unsigned char  h[44];
    const uint32_t data_bytes = frames * 2u;

    memcpy(h + 0, "RIFF", 4);
    put_le32(h + 4, 36u + data_bytes);
    memcpy(h + 8, "WAVEfmt ", 8);
    put_le32(h + 16, 16u);
    put_le16(h + 20, 1u); /* PCM      */
    put_le16(h + 22, 1u); /* channels */
    put_le32(h + 24, SAMPLE_RATE_HZ);
    put_le32(h + 28, SAMPLE_RATE_HZ * 2u);
    put_le16(h + 32, 2u);
    put_le16(h + 34, 16u);
    memcpy(h + 36, "data", 4);
    put_le32(h + 40, data_bytes);
    return fwrite(h, 1, sizeof h, f) == sizeof h;
}

static int write_pcm16(FILE *f, const int16_t *pcm, size_t n)
{
    unsigned char bytes[BLOCK_FRAMES * 2u];
    size_t        i;

    for (i = 0; i < n; i++)
        put_le16(bytes + 2u * i, (unsigned)(uint16_t)pcm[i]);
    return fwrite(bytes, 2, n, f) == n;
}

static FILE *open_wav(const char *dir, const char *name, uint32_t frames, char *path)
{
    FILE *f;

    if (snprintf(path, PATH_MAX_LEN, "%s/%s", dir, name) >= (int)PATH_MAX_LEN)
    {
        fprintf(stderr, "path too long\n");
        return NULL;
    }
    f = fopen(path, "wb");
    if (f == NULL)
    {
        perror(path);
        return NULL;
    }
    if (!write_wav_header(f, frames))
    {
        perror(path);
        fclose(f);
        return NULL;
    }
    return f;
}

/* ---- Noise colours ---- */

static int render_colour(
    const char *dir, const char *name, pcl_noise_colour_t colour, uint32_t frames, uint32_t seed)
{
    pcl_noise_config_t cfg;
    pcl_noise_t        noise;
    pcl_sample_t       block[BLOCK_FRAMES];
    int16_t            pcm[BLOCK_FRAMES];
    char               path[PATH_MAX_LEN];
    double             sum2 = 0.0, gain, peak = 0.0;
    uint32_t           done;
    size_t             i;
    FILE              *f;

    cfg.sample_rate_hz = SAMPLE_RATE_HZ;
    cfg.colour = colour;
    cfg.seed = seed;
    if (pcl_noise_init(&noise, &cfg) != PCL_OK)
    {
        fprintf(stderr, "noise configuration rejected\n");
        return 0;
    }

    /* Pass 1: measure the rms of the whole file. */
    for (done = 0; done < frames;)
    {
        const size_t n = (frames - done < BLOCK_FRAMES) ? (frames - done) : BLOCK_FRAMES;
        pcl_noise_process(&noise, block, n);
        for (i = 0; i < n; i++)
        {
            const double v = sample_to_double(block[i]);
            sum2 += v * v;
        }
        done += (uint32_t)n;
    }
    gain = TARGET_RMS / sqrt(sum2 / (double)frames);

    /* Pass 2: the same stream again (reset restarts it from the seed), scaled and written. */
    pcl_noise_reset(&noise);
    f = open_wav(dir, name, frames, path);
    if (f == NULL)
        return 0;
    for (done = 0; done < frames;)
    {
        const size_t n = (frames - done < BLOCK_FRAMES) ? (frames - done) : BLOCK_FRAMES;
        pcl_noise_process(&noise, block, n);
        for (i = 0; i < n; i++)
        {
            const double v = sample_to_double(block[i]) * gain;
            if (fabs(v) > peak)
                peak = fabs(v);
            pcm[i] = pcm16_from_double(v);
        }
        if (!write_pcm16(f, pcm, n))
        {
            perror(path);
            fclose(f);
            return 0;
        }
        done += (uint32_t)n;
    }
    if (fclose(f) != 0)
    {
        perror(path);
        return 0;
    }
    printf("wrote %-24s %-6s gain %6.3f, peak %.3f of full scale\n", path,
        colour == PCL_NOISE_WHITE ? "white" : (colour == PCL_NOISE_PINK ? "pink" : "brown"), gain,
        peak);
    return 1;
}

/* ---- Sample-and-hold driving the oscillator's pitch ---- */

static int render_sah_pitch(
    const char *dir, const char *name, uint32_t slew_ms, uint32_t frames, uint32_t seed)
{
    pcl_sah_config_t sah_cfg;
    pcl_osc_config_t osc_cfg;
    pcl_sah_t        sah;
    pcl_osc_t        osc;
    pcl_sample_t     pitch[BLOCK_FRAMES];
    pcl_sample_t     audio[BLOCK_FRAMES];
    int16_t          pcm[BLOCK_FRAMES];
    char             path[PATH_MAX_LEN];
    uint32_t         done;
    size_t           i;
    FILE            *f;

    sah_cfg.sample_rate_hz = SAMPLE_RATE_HZ;
    sah_cfg.seed = seed;
    sah_cfg.rate_hz = PCL_HZ(SAH_RATE_HZ);
    sah_cfg.slew_ms = slew_ms;
    osc_cfg.sample_rate_hz = SAMPLE_RATE_HZ;
    osc_cfg.waveform = PCL_OSC_SAW;
    osc_cfg.frequency = PCL_HZ(SAH_BASE_HZ);
    osc_cfg.pitch_depth_oct = PCL_OCT(SAH_DEPTH_OCT);
    osc_cfg.pulse_width = PCL_COEF(0.5);
    osc_cfg.pwm_depth = PCL_COEF(0.0);
    if (pcl_sah_init(&sah, &sah_cfg) != PCL_OK || pcl_osc_init(&osc, &osc_cfg) != PCL_OK)
    {
        fprintf(stderr, "sample-and-hold/oscillator configuration rejected\n");
        return 0;
    }

    f = open_wav(dir, name, frames, path);
    if (f == NULL)
        return 0;
    for (done = 0; done < frames;)
    {
        const size_t n = (frames - done < BLOCK_FRAMES) ? (frames - done) : BLOCK_FRAMES;
        pcl_sah_process(&sah, pitch, NULL, NULL, 0, n);
        pcl_osc_process(&osc, audio, pitch, NULL, NULL, NULL, n);
        for (i = 0; i < n; i++)
            pcm[i] = pcm16_from_double(sample_to_double(audio[i]) * SAH_OUTPUT_GAIN);
        if (!write_pcm16(f, pcm, n))
        {
            perror(path);
            fclose(f);
            return 0;
        }
        done += (uint32_t)n;
    }
    if (fclose(f) != 0)
    {
        perror(path);
        return 0;
    }
    printf("wrote %-24s saw %.0f Hz +/-%.0f octave, S&H %.0f Hz, slew %u ms\n", path, SAH_BASE_HZ,
        SAH_DEPTH_OCT, SAH_RATE_HZ, (unsigned)slew_ms);
    return 1;
}

/* ---- LFO patches ---- */

typedef enum
{
    LFO_TO_PITCH,
    LFO_TO_PWM,
    LFO_TO_GAIN
} lfo_target_t;

typedef struct
{
    const char        *file;
    pcl_osc_waveform_t waveform;
    double             osc_hz;
    double             depth; /* pitch octaves or PWM width swing per unit of modulation */
    pcl_lfo_shape_t    shape;
    double             lfo_hz;
    int                unipolar;
    lfo_target_t       target;
} lfo_patch_t;

#define SEMITONE_OCT (1.0 / 12.0)
#define LFO_PATCH_GAIN 0.3
#define LFO_TREMOLO_GAIN 0.5

/* Writes one patch: a saw/sine/pulse oscillator modulated by an LFO through the chosen input. */
static int render_lfo_patch(const char *dir, const lfo_patch_t *p, uint32_t frames, uint32_t seed)
{
    pcl_lfo_config_t lfo_cfg;
    pcl_osc_config_t osc_cfg;
    pcl_vca_config_t vca_cfg;
    pcl_lfo_t        lfo;
    pcl_osc_t        osc;
    pcl_vca_t        vca;
    pcl_sample_t     mod[BLOCK_FRAMES];
    pcl_sample_t     audio[BLOCK_FRAMES];
    int16_t          pcm[BLOCK_FRAMES];
    char             path[PATH_MAX_LEN];
    uint32_t         done;
    size_t           i;
    FILE            *f;

    lfo_cfg.sample_rate_hz = SAMPLE_RATE_HZ;
    lfo_cfg.shape = p->shape;
    lfo_cfg.frequency = (pcl_hz_t)0;
    lfo_cfg.rate_depth_oct = PCL_OCT(0.0);
    lfo_cfg.pulse_width = PCL_COEF(0.5);
    lfo_cfg.unipolar = p->unipolar;
    lfo_cfg.start_phase = 0u;
    lfo_cfg.seed = seed;

    osc_cfg.sample_rate_hz = SAMPLE_RATE_HZ;
    osc_cfg.waveform = p->waveform;
    osc_cfg.frequency = (pcl_hz_t)0;
    osc_cfg.pitch_depth_oct = PCL_OCT(0.0);
    osc_cfg.pulse_width = PCL_COEF(0.5);
    osc_cfg.pwm_depth = PCL_COEF(0.0);

    vca_cfg.sample_rate_hz = SAMPLE_RATE_HZ;
    vca_cfg.gain = PCL_COEF(LFO_TREMOLO_GAIN);

    if (pcl_lfo_init(&lfo, &lfo_cfg) != PCL_OK || pcl_osc_init(&osc, &osc_cfg) != PCL_OK
        || pcl_vca_init(&vca, &vca_cfg) != PCL_OK
        || pcl_lfo_set_frequency(&lfo, hz_from_double(p->lfo_hz)) != PCL_OK
        || pcl_osc_set_frequency(&osc, hz_from_double(p->osc_hz)) != PCL_OK
        || (p->target == LFO_TO_PITCH
            && pcl_osc_set_pitch_depth(&osc, oct_from_double(p->depth)) != PCL_OK)
        || (p->target == LFO_TO_PWM
            && pcl_osc_set_pwm_depth(&osc, coef_from_double(p->depth)) != PCL_OK))
    {
        fprintf(stderr, "LFO patch configuration rejected\n");
        return 0;
    }
    pcl_lfo_reset(&lfo); /* start the LFO from its start phase now that its rate is set */
    pcl_osc_reset(&osc);
    pcl_vca_reset(&vca);

    f = open_wav(dir, p->file, frames, path);
    if (f == NULL)
        return 0;
    for (done = 0; done < frames;)
    {
        const size_t n = (frames - done < BLOCK_FRAMES) ? (frames - done) : BLOCK_FRAMES;
        double       gain = LFO_PATCH_GAIN;

        pcl_lfo_process(&lfo, mod, NULL, n);
        if (p->target == LFO_TO_PITCH)
            pcl_osc_process(&osc, audio, mod, NULL, NULL, NULL, n);
        else if (p->target == LFO_TO_PWM)
            pcl_osc_process(&osc, audio, NULL, mod, NULL, NULL, n);
        else
        {
            pcl_osc_process(&osc, audio, NULL, NULL, NULL, NULL, n);
            pcl_vca_process(&vca, audio, audio, mod, n); /* in place */
            gain = 1.0;
        }
        for (i = 0; i < n; i++)
            pcm[i] = pcm16_from_double(sample_to_double(audio[i]) * gain);
        if (!write_pcm16(f, pcm, n))
        {
            perror(path);
            fclose(f);
            return 0;
        }
        done += (uint32_t)n;
    }
    if (fclose(f) != 0)
    {
        perror(path);
        return 0;
    }
    printf("wrote %-24s osc %.0f Hz, LFO %.2f Hz\n", path, p->osc_hz, p->lfo_hz);
    return 1;
}

static int render_lfo(const char *dir, double seconds, uint32_t seed)
{
    static const lfo_patch_t patches[] = {
        { "lfo_tremolo.wav", PCL_OSC_SAW, 220.0, 0.0, PCL_LFO_SINE, 5.0, 1, LFO_TO_GAIN },
        { "lfo_vibrato.wav", PCL_OSC_SINE, 440.0, 0.5 * SEMITONE_OCT, PCL_LFO_SINE, 5.5, 0,
            LFO_TO_PITCH },
        { "lfo_pwm.wav", PCL_OSC_PULSE, 110.0, 0.4, PCL_LFO_TRIANGLE, 0.5, 0, LFO_TO_PWM },
        { "lfo_wobble.wav", PCL_OSC_SAW, 110.0, 1.0, PCL_LFO_SQUARE, 2.0, 0, LFO_TO_PITCH },
        { "lfo_random.wav", PCL_OSC_SAW, 220.0, 1.0, PCL_LFO_RANDOM_SMOOTH, 3.0, 0, LFO_TO_PITCH },
    };
    const uint32_t frames = (uint32_t)(seconds * (double)SAMPLE_RATE_HZ);
    size_t         i;

    for (i = 0; i < sizeof patches / sizeof patches[0]; i++)
        if (!render_lfo_patch(dir, &patches[i], frames, seed))
            return 0;
    return 1;
}

/* ---- ADSR patches ---- */

#define ADSR_PATCH_GAIN 0.4
#define ADSR_MAX_GATES 12
#define ADSR_PULSE_SECONDS 0.0 /* a pulse is exactly one sample */

typedef struct
{
    const char *file;
    int         noise; /* 1: white noise source, 0: saw oscillator */
    double      osc_hz;
    uint32_t    attack_ms, decay_ms, release_ms;
    double      sustain, attack_curve, decay_curve, release_curve;
    int         run_through;
    int         pulses; /* gate_s are 1-sample pulse times in a gate buffer, not on/off events */
    size_t      n_gates;
    double gate_s[ADSR_MAX_GATES]; /* seconds: alternating on, off, ... (events) or pulse times */
    double curve2; /* if switch_s > 0, all three curves become this at switch_s seconds */
    double switch_s;
} adsr_patch_t;

static int render_adsr_patch(const char *dir, const adsr_patch_t *p, uint32_t frames, uint32_t seed)
{
    pcl_adsr_config_t  acfg;
    pcl_osc_config_t   ocfg;
    pcl_noise_config_t ncfg;
    pcl_vca_config_t   vcfg;
    pcl_adsr_t         adsr;
    pcl_osc_t          osc;
    pcl_noise_t        noise;
    pcl_vca_t          vca;
    pcl_sample_t       audio[BLOCK_FRAMES];
    pcl_sample_t       env[BLOCK_FRAMES];
    pcl_sample_t       gate[BLOCK_FRAMES];
    pcl_event_t        events[ADSR_MAX_GATES];
    int16_t            pcm[BLOCK_FRAMES];
    char               path[PATH_MAX_LEN];
    uint32_t           done;
    size_t             i, g;
    int                switched = 0;
    FILE              *f;

    acfg.sample_rate_hz = SAMPLE_RATE_HZ;
    acfg.attack_ms = p->attack_ms;
    acfg.decay_ms = p->decay_ms;
    acfg.release_ms = p->release_ms;
    acfg.sustain = coef_from_double(p->sustain);
    acfg.attack_curve = coef_from_double(p->attack_curve);
    acfg.decay_curve = coef_from_double(p->decay_curve);
    acfg.release_curve = coef_from_double(p->release_curve);
    acfg.run_through = p->run_through;
    ocfg.sample_rate_hz = SAMPLE_RATE_HZ;
    ocfg.waveform = PCL_OSC_SAW;
    ocfg.frequency = hz_from_double(p->osc_hz);
    ocfg.pitch_depth_oct = PCL_OCT(0.0);
    ocfg.pulse_width = PCL_COEF(0.5);
    ocfg.pwm_depth = PCL_COEF(0.0);
    ncfg.sample_rate_hz = SAMPLE_RATE_HZ;
    ncfg.colour = PCL_NOISE_WHITE;
    ncfg.seed = seed;
    vcfg.sample_rate_hz = SAMPLE_RATE_HZ;
    vcfg.gain = PCL_COEF(1.0);
    if (pcl_adsr_init(&adsr, &acfg) != PCL_OK || pcl_osc_init(&osc, &ocfg) != PCL_OK
        || pcl_noise_init(&noise, &ncfg) != PCL_OK || pcl_vca_init(&vca, &vcfg) != PCL_OK)
    {
        fprintf(stderr, "ADSR patch configuration rejected\n");
        return 0;
    }

    f = open_wav(dir, p->file, frames, path);
    if (f == NULL)
        return 0;
    for (done = 0; done < frames;)
    {
        const size_t n = (frames - done < BLOCK_FRAMES) ? (frames - done) : BLOCK_FRAMES;
        size_t       cnt = 0;

        if (p->switch_s > 0.0 && !switched && (double)done >= p->switch_s * (double)SAMPLE_RATE_HZ)
        {
            switched = 1;
            (void)pcl_adsr_set_attack_curve(&adsr, coef_from_double(p->curve2));
            (void)pcl_adsr_set_decay_curve(&adsr, coef_from_double(p->curve2));
            (void)pcl_adsr_set_release_curve(&adsr, coef_from_double(p->curve2));
        }
        if (p->noise)
            pcl_noise_process(&noise, audio, n);
        else
            pcl_osc_process(&osc, audio, NULL, NULL, NULL, NULL, n);

        for (i = 0; i < n; i++)
            gate[i] = (pcl_sample_t)0;
        for (g = 0; g < p->n_gates; g++)
        {
            const double at = p->gate_s[g] * (double)SAMPLE_RATE_HZ;
            if (at >= (double)done && at < (double)(done + n))
            {
                if (p->pulses)
                {
                    gate[(size_t)at - done] = sample_from_double(0.9);
                }
                else
                {
                    events[cnt].offset = (uint16_t)((size_t)at - done);
                    events[cnt].type = (g % 2u == 0u) ? PCL_EVT_NOTE_ON : PCL_EVT_NOTE_OFF;
                    events[cnt].note = 60;
                    events[cnt].velocity = 100;
                    cnt++;
                }
            }
        }
        pcl_adsr_process(&adsr, env, p->pulses ? gate : NULL, p->pulses ? NULL : events,
            p->pulses ? 0u : cnt, n);
        pcl_vca_process(&vca, audio, audio, env, n); /* in place */
        for (i = 0; i < n; i++)
            pcm[i] = pcm16_from_double(sample_to_double(audio[i]) * ADSR_PATCH_GAIN);
        if (!write_pcm16(f, pcm, n))
        {
            perror(path);
            fclose(f);
            return 0;
        }
        done += (uint32_t)n;
    }
    if (fclose(f) != 0)
    {
        perror(path);
        return 0;
    }
    printf("wrote %-24s A %u / D %u / S %.2f / R %u ms\n", path, (unsigned)p->attack_ms,
        (unsigned)p->decay_ms, p->sustain, (unsigned)p->release_ms);
    return 1;
}

static int render_adsr(const char *dir, double seconds, uint32_t seed)
{
    static const adsr_patch_t patches[] = {
        { "adsr_pluck.wav", 0, 220.0, 2, 250, 300, 0.2, 0.5, 0.5, 0.5, 0, 0, 8,
            { 0.0, 0.5, 1.2, 1.7, 2.4, 2.9, 3.6, 4.1 }, 0.0, 0.0 },
        { "adsr_pad.wav", 0, 110.0, 800, 500, 1200, 0.7, 0.5, 0.5, 0.5, 0, 0, 2, { 0.2, 3.0 }, 0.0,
            0.0 },
        { "adsr_perc.wav", 1, 0.0, 0, 120, 50, 0.0, 0.0, 0.5, 0.5, 1, 1, 8,
            { 0.1, 0.6, 1.1, 1.6, 2.1, 2.6, 3.1, 3.6 }, 0.0, 0.0 },
        /* linear, then (from 1.9 s) curve 0.6 for the note at 2.5 s */
        { "adsr_curves.wav", 0, 220.0, 5, 400, 400, 0.3, 0.0, 0.0, 0.0, 0, 0, 4,
            { 0.0, 0.8, 2.5, 3.3 }, 0.6, 1.9 },
        { "adsr_retrigger.wav", 0, 220.0, 20, 200, 800, 0.5, 0.5, 0.5, 0.5, 0, 0, 8,
            { 0.0, 0.5, 0.8, 1.2, 1.4, 1.45, 1.5, 2.5 }, 0.0, 0.0 },
    };
    const uint32_t frames = (uint32_t)(seconds * (double)SAMPLE_RATE_HZ);
    size_t         i;

    for (i = 0; i < sizeof patches / sizeof patches[0]; i++)
        if (!render_adsr_patch(dir, &patches[i], frames, seed))
            return 0;
    return 1;
}

/* ---- SVF patches ---- */

typedef enum
{
    SVF_SWEEP,
    SVF_WAH,
    SVF_SELFOSC,
    SVF_VOICE
} svf_patch_t;

#define SVF_PATCH_GAIN 0.4
#define SVF_INPUT_GAIN                                                                             \
    0.15 /* headroom for the resonant peak: the filter output saturates at 1.0                     \
          */
#define SVF_NOTE_PERIOD_S 0.64 /* 120 blocks: note changes land on block boundaries */

static size_t note_events(
    const double *times, size_t n_times, uint32_t done, size_t n, pcl_event_t *ev)
{
    size_t cnt = 0, g;

    for (g = 0; g < n_times; g++)
    {
        const double at = times[g] * (double)SAMPLE_RATE_HZ;
        if (at >= (double)done && at < (double)(done + n))
        {
            ev[cnt].offset = (uint16_t)((size_t)at - done);
            ev[cnt].type = (g % 2u == 0u) ? PCL_EVT_NOTE_ON : PCL_EVT_NOTE_OFF;
            ev[cnt].note = 60;
            ev[cnt].velocity = 100;
            cnt++;
        }
    }
    return cnt;
}

static int render_svf_patch(
    const char *dir, svf_patch_t kind, const char *file, uint32_t frames, uint32_t seed)
{
    static const double sweep_times[] = { 0.0, 0.9, 1.2, 2.1, 2.4, 3.3 };
    static const int    voice_notes[] = { 45, 52, 57, 60, 57, 52, 48, 45 };
    pcl_svf_config_t    scfg;
    pcl_svf_t           svf;
    pcl_osc_config_t    ocfg;
    pcl_osc_t           osc;
    pcl_noise_config_t  ncfg;
    pcl_noise_t         noise;
    pcl_lfo_config_t    lcfg;
    pcl_lfo_t           lfo;
    pcl_adsr_config_t   fcfg, acfg;
    pcl_adsr_t          fenv, aenv;
    pcl_vca_config_t    vcfg;
    pcl_vca_t           vca;
    pcl_sample_t        audio[BLOCK_FRAMES];
    pcl_sample_t        mod[BLOCK_FRAMES];
    pcl_sample_t        amp[BLOCK_FRAMES];
    pcl_sample_t        out[BLOCK_FRAMES];
    pcl_event_t         events[16];
    double              times[16];
    size_t              n_times = 0;
    int16_t             pcm[BLOCK_FRAMES];
    char                path[PATH_MAX_LEN];
    uint32_t            done;
    size_t              i, k;
    double              gain = kind == SVF_SWEEP ? 1.4 : (kind == SVF_VOICE ? 0.8 : SVF_PATCH_GAIN);
    double              res = 0.0, depth = 0.0; /* for the log line */
    int                 moved = 0;
    FILE               *f;

    scfg.sample_rate_hz = SAMPLE_RATE_HZ;
    scfg.saturate = 0;
    switch (kind)
    {
    case SVF_SWEEP:
        res = 0.7;
        depth = 5.5;
        scfg.cutoff_hz = hz_from_double(150.0);
        break;
    case SVF_WAH:
        res = 0.85;
        depth = 2.0;
        scfg.cutoff_hz = hz_from_double(400.0);
        gain = 4.0;
        break;
    case SVF_SELFOSC:
        res = 1.0;
        depth = 0.0;
        scfg.cutoff_hz = hz_from_double(440.0);
        gain = 6.0;
        break;
    default: /* SVF_VOICE */
        res = 0.75;
        depth = 4.5;
        scfg.cutoff_hz = hz_from_double(200.0);
        break;
    }
    scfg.resonance = coef_from_double(res);
    scfg.mod_depth_oct = oct_from_double(depth);
    ocfg.sample_rate_hz = SAMPLE_RATE_HZ;
    ocfg.waveform = PCL_OSC_SAW;
    ocfg.frequency = hz_from_double(110.0);
    ocfg.pitch_depth_oct = PCL_OCT(0.0);
    ocfg.pulse_width = PCL_COEF(0.5);
    ocfg.pwm_depth = PCL_COEF(0.0);
    ncfg.sample_rate_hz = SAMPLE_RATE_HZ;
    ncfg.colour = PCL_NOISE_WHITE;
    ncfg.seed = seed;
    lcfg.sample_rate_hz = SAMPLE_RATE_HZ;
    lcfg.shape = PCL_LFO_SINE;
    lcfg.frequency = hz_from_double(1.2);
    lcfg.rate_depth_oct = PCL_OCT(0.0);
    lcfg.pulse_width = PCL_COEF(0.5);
    lcfg.unipolar = 0;
    lcfg.start_phase = 0u;
    lcfg.seed = seed;
    /* filter envelope: bright then closing; amplitude envelope: a gentle pluck */
    fcfg.sample_rate_hz = SAMPLE_RATE_HZ;
    fcfg.attack_ms = 3;
    fcfg.decay_ms = kind == SVF_SWEEP ? 700u : 350u;
    fcfg.release_ms = 250;
    fcfg.sustain = coef_from_double(kind == SVF_SWEEP ? 0.0 : 0.25);
    fcfg.attack_curve = fcfg.decay_curve = fcfg.release_curve = coef_from_double(0.5);
    fcfg.run_through = 0;
    acfg = fcfg;
    acfg.attack_ms = 3;
    acfg.decay_ms = 400;
    acfg.release_ms = 300;
    acfg.sustain = coef_from_double(0.7);
    vcfg.sample_rate_hz = SAMPLE_RATE_HZ;
    vcfg.gain = PCL_COEF(1.0);
    if (pcl_svf_init(&svf, &scfg) != PCL_OK || pcl_osc_init(&osc, &ocfg) != PCL_OK
        || pcl_noise_init(&noise, &ncfg) != PCL_OK || pcl_lfo_init(&lfo, &lcfg) != PCL_OK
        || pcl_adsr_init(&fenv, &fcfg) != PCL_OK || pcl_adsr_init(&aenv, &acfg) != PCL_OK
        || pcl_vca_init(&vca, &vcfg) != PCL_OK)
    {
        fprintf(stderr, "SVF patch configuration rejected\n");
        return 0;
    }
    if (kind == SVF_SWEEP)
    {
        for (k = 0; k < sizeof sweep_times / sizeof sweep_times[0]; k++)
            times[n_times++] = sweep_times[k];
    }
    else if (kind == SVF_VOICE)
    {
        for (k = 0; k < sizeof voice_notes / sizeof voice_notes[0]; k++)
        {
            times[n_times++] = (double)k * SVF_NOTE_PERIOD_S;
            times[n_times++] = (double)k * SVF_NOTE_PERIOD_S + 0.45;
        }
    }

    f = open_wav(dir, file, frames, path);
    if (f == NULL)
        return 0;
    for (done = 0; done < frames;)
    {
        const size_t n = (frames - done < BLOCK_FRAMES) ? (frames - done) : BLOCK_FRAMES;
        const size_t cnt = note_events(times, n_times, done, n, events);

        switch (kind)
        {
        case SVF_SWEEP:
            pcl_osc_process(&osc, audio, NULL, NULL, NULL, NULL, n);
            for (i = 0; i < n; i++)
                audio[i] = sample_from_double(sample_to_double(audio[i]) * SVF_INPUT_GAIN);
            pcl_adsr_process(&fenv, mod, NULL, events, cnt, n);
            pcl_adsr_process(&aenv, amp, NULL, events, cnt, n);
            pcl_svf_process_mode(&svf, audio, out, PCL_SVF_LP, mod, n);
            pcl_vca_process(&vca, out, out, amp, n);
            break;
        case SVF_WAH:
            pcl_noise_process(&noise, audio, n);
            pcl_lfo_process(&lfo, mod, NULL, n);
            pcl_svf_process_mode(&svf, audio, out, PCL_SVF_BP_NORM, mod, n);
            break;
        case SVF_SELFOSC:
        {
            /* an impulse at 0.1 s and one at 1.1 s; the cutoff goes up an octave at 1.0 s */
            const double t0 = 0.1 * (double)SAMPLE_RATE_HZ;
            const double t1 = 1.1 * (double)SAMPLE_RATE_HZ;

            if (!moved && (double)done >= 1.0 * (double)SAMPLE_RATE_HZ)
            {
                moved = 1;
                (void)pcl_svf_set_cutoff(&svf, hz_from_double(880.0));
            }
            for (i = 0; i < n; i++)
                audio[i] = (pcl_sample_t)0;
            if (t0 >= (double)done && t0 < (double)(done + n))
                audio[(size_t)t0 - done] = sample_from_double(0.9);
            if (t1 >= (double)done && t1 < (double)(done + n))
                audio[(size_t)t1 - done] = sample_from_double(0.9);
            pcl_svf_process_mode(&svf, audio, out, PCL_SVF_LP, NULL, n);
            break;
        }
        default: /* SVF_VOICE */
        {
            size_t note;
            for (note = 0; note < sizeof voice_notes / sizeof voice_notes[0]; note++)
            {
                const double at = (double)note * SVF_NOTE_PERIOD_S * (double)SAMPLE_RATE_HZ;
                if (at >= (double)done && at < (double)(done + n))
                    (void)pcl_osc_set_frequency(&osc, pcl_midi_to_hz((uint8_t)voice_notes[note]));
            }
            pcl_osc_process(&osc, audio, NULL, NULL, NULL, NULL, n);
            for (i = 0; i < n; i++)
                audio[i] = sample_from_double(sample_to_double(audio[i]) * SVF_INPUT_GAIN);
            pcl_adsr_process(&fenv, mod, NULL, events, cnt, n);
            pcl_adsr_process(&aenv, amp, NULL, events, cnt, n);
            pcl_svf_process_mode(&svf, audio, out, PCL_SVF_LP, mod, n);
            pcl_vca_process(&vca, out, out, amp, n);
            break;
        }
        }
        for (i = 0; i < n; i++)
            pcm[i] = pcm16_from_double(sample_to_double(out[i]) * gain);
        if (!write_pcm16(f, pcm, n))
        {
            perror(path);
            fclose(f);
            return 0;
        }
        done += (uint32_t)n;
    }
    if (fclose(f) != 0)
    {
        perror(path);
        return 0;
    }
    printf("wrote %-24s resonance %.2f, cutoff depth %.1f oct\n", path, res, depth);
    return 1;
}

static int render_svf(const char *dir, double seconds, uint32_t seed)
{
    const uint32_t frames = (uint32_t)(seconds * (double)SAMPLE_RATE_HZ);

    return render_svf_patch(dir, SVF_SWEEP, "svf_sweep.wav", frames, seed)
        && render_svf_patch(dir, SVF_WAH, "svf_wah.wav", frames, seed)
        && render_svf_patch(dir, SVF_SELFOSC, "svf_selfosc.wav", frames, seed)
        && render_svf_patch(dir, SVF_VOICE, "svf_voice.wav", frames, seed);
}

/* ---- Biquad patches ---- */

#define BQ_INPUT_GAIN 0.25 /* headroom: a +12 dB boost of full-scale noise would saturate */

typedef enum
{
    BQ_EQ_SWEEP,
    BQ_TILT,
    BQ_NOTCH,
    BQ_BUTTER4
} bq_patch_t;

static int render_biquad_patch(
    const char *dir, bq_patch_t kind, const char *file, double seconds, uint32_t seed)
{
    const uint32_t      frames = (uint32_t)(seconds * (double)SAMPLE_RATE_HZ);
    pcl_biquad_config_t bcfg;
    pcl_biquad_t        bq, bq2;
    pcl_noise_config_t  ncfg;
    pcl_noise_t         noise;
    pcl_osc_config_t    ocfg;
    pcl_osc_t           osc;
    pcl_lfo_config_t    lcfg;
    pcl_lfo_t           lfo;
    pcl_sample_t        audio[BLOCK_FRAMES];
    pcl_sample_t        mod[BLOCK_FRAMES];
    int16_t             pcm[BLOCK_FRAMES];
    char                path[PATH_MAX_LEN];
    uint32_t            done;
    size_t              i;
    double              gain = 0.3;
    int                 switched = 0;
    FILE               *f;

    bcfg.sample_rate_hz = SAMPLE_RATE_HZ;
    bcfg.type = PCL_BIQUAD_PEAK;
    bcfg.cutoff_hz = hz_from_double(100.0);
    bcfg.q = PCL_QUALITY(4.0);
    bcfg.gain_db = PCL_DB(12.0);
    bcfg.mod_depth_oct = PCL_OCT(0.0);
    switch (kind)
    {
    case BQ_EQ_SWEEP:
        gain = 0.5;
        break;
    case BQ_TILT:
        bcfg.type = PCL_BIQUAD_LOWSHELF;
        bcfg.cutoff_hz = hz_from_double(400.0);
        bcfg.q = PCL_QUALITY(0.7071);
        gain = 0.25;
        break;
    case BQ_NOTCH:
        bcfg.type = PCL_BIQUAD_NOTCH;
        bcfg.cutoff_hz = hz_from_double(1000.0);
        bcfg.q = PCL_QUALITY(6.0);
        bcfg.mod_depth_oct = oct_from_double(2.0);
        gain = 1.0;
        break;
    default: /* BQ_BUTTER4: two sections, Q = 1 / (2 cos(pi/8)) and 1 / (2 cos(3 pi/8)) */
        bcfg.type = PCL_BIQUAD_LP;
        bcfg.cutoff_hz = hz_from_double(1000.0);
        bcfg.q = PCL_QUALITY(0.5412);
        bcfg.gain_db = PCL_DB(0.0);
        gain = 0.5;
        break;
    }
    ocfg.sample_rate_hz = SAMPLE_RATE_HZ;
    ocfg.waveform = PCL_OSC_SAW;
    ocfg.frequency = hz_from_double(110.0);
    ocfg.pitch_depth_oct = PCL_OCT(0.0);
    ocfg.pulse_width = PCL_COEF(0.5);
    ocfg.pwm_depth = PCL_COEF(0.0);
    ncfg.sample_rate_hz = SAMPLE_RATE_HZ;
    ncfg.colour = PCL_NOISE_WHITE;
    ncfg.seed = seed;
    lcfg.sample_rate_hz = SAMPLE_RATE_HZ;
    lcfg.shape = PCL_LFO_SINE;
    lcfg.frequency = hz_from_double(0.5);
    lcfg.rate_depth_oct = PCL_OCT(0.0);
    lcfg.pulse_width = PCL_COEF(0.5);
    lcfg.unipolar = 0;
    lcfg.start_phase = 0u;
    lcfg.seed = seed;
    if (pcl_biquad_init(&bq, &bcfg) != PCL_OK || pcl_osc_init(&osc, &ocfg) != PCL_OK
        || pcl_noise_init(&noise, &ncfg) != PCL_OK || pcl_lfo_init(&lfo, &lcfg) != PCL_OK)
    {
        fprintf(stderr, "biquad patch configuration rejected\n");
        return 0;
    }
    bcfg.q = PCL_QUALITY(1.3066); /* the second Butterworth section */
    if (pcl_biquad_init(&bq2, &bcfg) != PCL_OK)
    {
        fprintf(stderr, "biquad patch configuration rejected\n");
        return 0;
    }

    f = open_wav(dir, file, frames, path);
    if (f == NULL)
        return 0;
    for (done = 0; done < frames;)
    {
        const size_t n = (frames - done < BLOCK_FRAMES) ? (frames - done) : BLOCK_FRAMES;
        const double t = (double)done / (double)SAMPLE_RATE_HZ;

        switch (kind)
        {
        case BQ_EQ_SWEEP:
            /* centre 100 Hz -> 8 kHz, exponentially, over the whole file */
            (void)pcl_biquad_set_cutoff(&bq, hz_from_double(100.0 * pow(80.0, t / seconds)));
            pcl_noise_process(&noise, audio, n);
            for (i = 0; i < n; i++)
                audio[i] = sample_from_double(sample_to_double(audio[i]) * BQ_INPUT_GAIN);
            pcl_biquad_process(&bq, audio, audio, NULL, n);
            break;
        case BQ_TILT:
            if (!switched && t >= 0.5 * seconds)
            {
                switched = 1;
                (void)pcl_biquad_set_type(&bq, PCL_BIQUAD_HIGHSHELF);
                (void)pcl_biquad_set_cutoff(&bq, hz_from_double(2000.0));
                (void)pcl_biquad_set_gain(&bq, PCL_DB(-12.0));
            }
            pcl_osc_process(&osc, audio, NULL, NULL, NULL, NULL, n);
            for (i = 0; i < n; i++)
                audio[i] = sample_from_double(sample_to_double(audio[i]) * 0.2);
            pcl_biquad_process(&bq, audio, audio, NULL, n);
            break;
        case BQ_NOTCH:
            pcl_noise_process(&noise, audio, n);
            for (i = 0; i < n; i++)
                audio[i] = sample_from_double(sample_to_double(audio[i]) * BQ_INPUT_GAIN);
            pcl_lfo_process(&lfo, mod, NULL, n);
            pcl_biquad_process(&bq, audio, audio, mod, n);
            break;
        default: /* BQ_BUTTER4 */
            pcl_noise_process(&noise, audio, n);
            for (i = 0; i < n; i++)
                audio[i] = sample_from_double(sample_to_double(audio[i]) * 0.5);
            pcl_biquad_process(&bq, audio, audio, NULL, n);
            pcl_biquad_process(&bq2, audio, audio, NULL, n);
            break;
        }
        for (i = 0; i < n; i++)
            pcm[i] = pcm16_from_double(sample_to_double(audio[i]) * gain);
        if (!write_pcm16(f, pcm, n))
        {
            perror(path);
            fclose(f);
            return 0;
        }
        done += (uint32_t)n;
    }
    if (fclose(f) != 0)
    {
        perror(path);
        return 0;
    }
    printf("wrote %s\n", path);
    return 1;
}

static int render_biquad(const char *dir, double seconds, uint32_t seed)
{
    return render_biquad_patch(dir, BQ_EQ_SWEEP, "biquad_eq_sweep.wav", seconds, seed)
        && render_biquad_patch(dir, BQ_TILT, "biquad_tilt.wav", seconds, seed)
        && render_biquad_patch(dir, BQ_NOTCH, "biquad_notch.wav", seconds, seed)
        && render_biquad_patch(dir, BQ_BUTTER4, "biquad_butter4.wav", seconds, seed);
}

/* ---- event patches: clock -> arpeggiator / sequencer -> voice ---- */

typedef enum
{
    EVT_ARP,
    EVT_ACID,
    EVT_PINGPONG
} events_patch_t;

#define EVT_MAX_ABS 4096u
#define EVT_ARP_CHORD_S 2.0 /* seconds per chord */
#define EVT_ARP_HOLD_S 1.9  /* the keys are released this long after the chord starts */
#define EVT_ARP_CHORDS 4u

typedef struct
{
    uint32_t at; /* absolute sample */
    uint8_t  type;
    uint8_t  note;
    uint8_t  velocity;
} abs_event_t;

/* One monophonic voice: saw -> state-variable low-pass (filter envelope) -> VCA (amp envelope). */
typedef struct
{
    pcl_osc_t  osc;
    pcl_svf_t  svf;
    pcl_adsr_t fenv, aenv;
    pcl_vca_t  vca;
    double     in_gain; /* level into the filter: keeps the resonant peak below full scale */
    double     gain;    /* output gain of the patch */
} voice_t;

static int voice_init(voice_t *v, double cutoff_hz, double res, double depth_oct,
    unsigned fdecay_ms, double fsustain, unsigned arel_ms, double in_gain, double gain)
{
    pcl_osc_config_t  ocfg;
    pcl_svf_config_t  scfg;
    pcl_adsr_config_t fcfg, acfg;
    pcl_vca_config_t  vcfg;

    ocfg.sample_rate_hz = SAMPLE_RATE_HZ;
    ocfg.waveform = PCL_OSC_SAW;
    ocfg.frequency = hz_from_double(110.0);
    ocfg.pitch_depth_oct = PCL_OCT(0.0);
    ocfg.pulse_width = PCL_COEF(0.5);
    ocfg.pwm_depth = PCL_COEF(0.0);
    scfg.sample_rate_hz = SAMPLE_RATE_HZ;
    scfg.saturate = 0;
    scfg.cutoff_hz = hz_from_double(cutoff_hz);
    scfg.resonance = coef_from_double(res);
    scfg.mod_depth_oct = oct_from_double(depth_oct);
    fcfg.sample_rate_hz = SAMPLE_RATE_HZ;
    fcfg.attack_ms = 2;
    fcfg.decay_ms = fdecay_ms;
    fcfg.release_ms = 120;
    fcfg.sustain = coef_from_double(fsustain);
    fcfg.attack_curve = fcfg.decay_curve = fcfg.release_curve = coef_from_double(0.5);
    fcfg.run_through = 0;
    acfg = fcfg;
    acfg.attack_ms = 2;
    acfg.decay_ms = 120;
    acfg.release_ms = arel_ms;
    acfg.sustain = coef_from_double(0.8);
    vcfg.sample_rate_hz = SAMPLE_RATE_HZ;
    vcfg.gain = PCL_COEF(1.0);
    v->in_gain = in_gain;
    v->gain = gain;
    return pcl_osc_init(&v->osc, &ocfg) == PCL_OK && pcl_svf_init(&v->svf, &scfg) == PCL_OK
        && pcl_adsr_init(&v->fenv, &fcfg) == PCL_OK && pcl_adsr_init(&v->aenv, &acfg) == PCL_OK
        && pcl_vca_init(&v->vca, &vcfg) == PCL_OK;
}

/* Renders `n` samples. The pitch changes on the sample of each NOTE_ON (the block is cut there);
   the envelopes take the events themselves. */
static void voice_process(
    voice_t *v, const pcl_event_t *ev, size_t cnt, pcl_sample_t *out, size_t n)
{
    pcl_sample_t audio[BLOCK_FRAMES], mod[BLOCK_FRAMES], amp[BLOCK_FRAMES];
    size_t       pos = 0, k, i;

    for (k = 0; k <= cnt; k++)
    {
        const size_t end = (k < cnt) ? ev[k].offset : n;

        if (end > pos)
        {
            pcl_osc_process(&v->osc, audio + pos, NULL, NULL, NULL, NULL, end - pos);
            pos = end;
        }
        if (k < cnt && ev[k].type == PCL_EVT_NOTE_ON && ev[k].velocity > 0)
            (void)pcl_osc_set_frequency(&v->osc, pcl_midi_to_hz(ev[k].note));
    }
    for (i = 0; i < n; i++)
        audio[i] = sample_from_double(sample_to_double(audio[i]) * v->in_gain);
    pcl_adsr_process(&v->fenv, mod, NULL, ev, cnt, n);
    pcl_adsr_process(&v->aenv, amp, NULL, ev, cnt, n);
    pcl_svf_process_mode(&v->svf, audio, out, PCL_SVF_LP, mod, n);
    pcl_vca_process(&v->vca, out, out, amp, n);
}

/* Merges two sorted arrays; at one offset `a` goes first. */
static size_t merge_events(
    const pcl_event_t *a, size_t na, const pcl_event_t *b, size_t nb, pcl_event_t *out)
{
    size_t i = 0, j = 0, n = 0;

    while (i < na || j < nb)
        out[n++] = (j >= nb || (i < na && a[i].offset <= b[j].offset)) ? a[i++] : b[j++];
    return n;
}

/* The keys of the arpeggiator patch: four chords, each pressed together and released a little
   before the next, repeating. Appends to `ev` and returns the new count. */
static size_t arp_key_script(abs_event_t *ev, uint32_t frames)
{
    static const uint8_t chords[EVT_ARP_CHORDS][4]
        = { { 57, 60, 64, 67 }, { 53, 57, 60, 64 }, { 48, 52, 55, 60 }, { 62, 55, 59, 67 } };
    size_t   n = 0;
    uint32_t c;

    for (c = 0;; c++)
    {
        const uint32_t on = (uint32_t)((double)c * EVT_ARP_CHORD_S * (double)SAMPLE_RATE_HZ);
        const uint32_t off = on + (uint32_t)(EVT_ARP_HOLD_S * (double)SAMPLE_RATE_HZ);
        unsigned       k;

        if (on >= frames || n + 8u > EVT_MAX_ABS)
            break;
        for (k = 0; k < 4u; k++)
        {
            ev[n].at = on;
            ev[n].type = PCL_EVT_NOTE_ON;
            ev[n].note = chords[c % EVT_ARP_CHORDS][k];
            ev[n].velocity = (uint8_t)(100 - 6u * k);
            n++;
        }
        for (k = 0; k < 4u; k++)
        {
            ev[n].at = off;
            ev[n].type = PCL_EVT_NOTE_OFF;
            ev[n].note = chords[c % EVT_ARP_CHORDS][k];
            ev[n].velocity = 0;
            n++;
        }
    }
    return n;
}

static int render_events_patch(
    const char *dir, events_patch_t kind, const char *file, uint32_t frames, uint32_t seed)
{
    static abs_event_t keys[EVT_MAX_ABS];
    pcl_clock_config_t ccfg;
    pcl_clock_t        clock;
    pcl_arp_t          arp;
    pcl_seq_t          seq;
    voice_t            voice;
    pcl_event_t        cev[64], kev[16], merged[80], nev[160];
    pcl_sample_t       out[BLOCK_FRAMES];
    int16_t            pcm[BLOCK_FRAMES];
    char               path[PATH_MAX_LEN];
    size_t             n_keys = 0, next_key = 0, i;
    uint32_t           done, last_chord = 0xFFFFFFFFu;
    double             bpm = 120.0;
    int                changed = 0;
    FILE              *f;

    switch (kind)
    {
    case EVT_ARP:
        bpm = 120.0;
        n_keys = arp_key_script(keys, frames);
        break;
    case EVT_ACID:
        bpm = 126.0;
        break;
    default: /* EVT_PINGPONG */
        bpm = 140.0;
        break;
    }
    ccfg.sample_rate_hz = SAMPLE_RATE_HZ;
    ccfg.bpm = hz_from_double(bpm);
    ccfg.ppqn = 24;
    ccfg.running = 1;
    if (pcl_clock_init(&clock, &ccfg) != PCL_OK)
    {
        fprintf(stderr, "clock configuration rejected\n");
        return 0;
    }
    if (kind == EVT_ARP)
    {
        pcl_arp_config_t acfg;

        acfg.mode = PCL_ARP_UP;
        acfg.octaves = 2;
        acfg.division = 6;
        acfg.gate = 3;
        acfg.latch = 0;
        acfg.seed = seed;
        if (pcl_arp_init(&arp, &acfg) != PCL_OK
            || !voice_init(&voice, 300.0, 0.4, 3.5, 250, 0.2, 80, 0.15, 0.9))
        {
            fprintf(stderr, "arpeggiator patch configuration rejected\n");
            return 0;
        }
    }
    else
    {
        /* acid line: A minor, ties for slides, rests, a swung sixteenth feel; the pingpong patch is
           an eight-note pentatonic run */
        static const pcl_seq_step_t acid[16] = { { 33, 100, 0, 0 }, { 33, 70, 0, 0 },
            { 45, 110, 0, PCL_SEQ_TIE }, { 43, 90, 0, 0 }, { 33, 100, 0, 0 }, { 33, 0, 0, 0 },
            { 36, 85, 0, 0 }, { 33, 70, 0, PCL_SEQ_TIE }, { 45, 110, 0, 0 }, { 33, 100, 0, 0 },
            { 33, 0, 0, 0 }, { 48, 100, 0, PCL_SEQ_TIE }, { 46, 95, 0, 0 }, { 33, 80, 0, 0 },
            { 40, 90, 0, 0 }, { 33, 100, 6, 0 } };
        static const uint8_t        penta[8] = { 57, 60, 62, 64, 67, 69, 72, 76 };
        pcl_seq_config_t            scfg;

        scfg.length = kind == EVT_ACID ? 16u : 8u;
        scfg.division = 6;
        scfg.gate = kind == EVT_ACID ? 3u : 2u;
        scfg.swing = kind == EVT_ACID ? 2u : 0u;
        scfg.direction = kind == EVT_ACID ? PCL_SEQ_FORWARD : PCL_SEQ_PINGPONG;
        scfg.transpose = 0;
        scfg.seed = seed;
        if (pcl_seq_init(&seq, &scfg) != PCL_OK
            || !(kind == EVT_ACID ? voice_init(&voice, 150.0, 0.85, 5.0, 220, 0.05, 60, 0.07, 0.9)
                                  : voice_init(&voice, 400.0, 0.3, 3.0, 300, 0.3, 100, 0.15, 1.0)))
        {
            fprintf(stderr, "sequencer patch configuration rejected\n");
            return 0;
        }
        for (i = 0; i < scfg.length; i++)
        {
            pcl_seq_step_t st;

            if (kind == EVT_ACID)
                st = acid[i];
            else
            {
                st.note = penta[i];
                st.velocity = 100;
                st.gate = 0;
                st.flags = 0;
            }
            (void)pcl_seq_set_step(&seq, i, &st);
        }
    }

    f = open_wav(dir, file, frames, path);
    if (f == NULL)
        return 0;
    for (done = 0; done < frames;)
    {
        const size_t n = (frames - done < BLOCK_FRAMES) ? (frames - done) : BLOCK_FRAMES;
        size_t       ncev = pcl_clock_process(&clock, n, cev, 64);
        size_t       nnev;

        if (kind == EVT_ARP)
        {
            size_t         nk = 0, nm;
            const uint32_t chord
                = (uint32_t)((double)done / (EVT_ARP_CHORD_S * (double)SAMPLE_RATE_HZ));

            if (chord != last_chord)
            {
                /* a different way of playing each chord */
                static const pcl_arp_mode_t modes[EVT_ARP_CHORDS]
                    = { PCL_ARP_UP, PCL_ARP_UPDOWN, PCL_ARP_RANDOM, PCL_ARP_AS_PLAYED };

                last_chord = chord;
                (void)pcl_arp_set_mode(&arp, modes[chord % EVT_ARP_CHORDS]);
                (void)pcl_arp_set_octaves(&arp, chord % EVT_ARP_CHORDS == 2u ? 3u : 2u);
                (void)pcl_arp_set_division(&arp, chord % EVT_ARP_CHORDS == 2u ? 3u : 6u);
                (void)pcl_arp_set_gate(&arp, chord % EVT_ARP_CHORDS == 2u ? 2u : 3u);
            }
            while (next_key < n_keys && keys[next_key].at < done + n && nk < 16u)
            {
                kev[nk].offset = (uint16_t)(keys[next_key].at - done);
                kev[nk].type = keys[next_key].type;
                kev[nk].note = keys[next_key].note;
                kev[nk].velocity = keys[next_key].velocity;
                nk++;
                next_key++;
            }
            nm = merge_events(kev, nk, cev, ncev, merged);
            nnev = pcl_arp_process(&arp, merged, nm, nev, 160);
        }
        else
        {
            if (kind == EVT_ACID && !changed && done >= frames / 2u)
            {
                /* half-way: up a fourth, and a little faster */
                changed = 1;
                pcl_seq_set_transpose(&seq, 5);
                (void)pcl_clock_set_bpm(&clock, hz_from_double(bpm + 8.0));
            }
            if (kind == EVT_PINGPONG && !changed && done >= frames / 2u)
            {
                changed = 1;
                (void)pcl_seq_set_direction(&seq, PCL_SEQ_RANDOM);
            }
            nnev = pcl_seq_process(&seq, cev, ncev, nev, 160);
        }
        voice_process(&voice, nev, nnev, out, n);
        for (i = 0; i < n; i++)
            pcm[i] = pcm16_from_double(sample_to_double(out[i]) * voice.gain);
        if (!write_pcm16(f, pcm, n))
        {
            perror(path);
            fclose(f);
            return 0;
        }
        done += (uint32_t)n;
    }
    if (fclose(f) != 0)
    {
        perror(path);
        return 0;
    }
    printf("wrote %-24s %.0f bpm, dropped %u events\n", path, bpm,
        (unsigned)(pcl_clock_dropped(&clock)
            + (kind == EVT_ARP ? pcl_arp_dropped(&arp) : pcl_seq_dropped(&seq))));
    return 1;
}

static int render_events(const char *dir, double seconds, uint32_t seed)
{
    const uint32_t frames = (uint32_t)(seconds * (double)SAMPLE_RATE_HZ);

    return render_events_patch(dir, EVT_ARP, "events_arp.wav", frames, seed)
        && render_events_patch(dir, EVT_ACID, "events_acid.wav", frames, seed)
        && render_events_patch(dir, EVT_PINGPONG, "events_pingpong.wav", frames, seed);
}

static int render_noise(const char *dir, double seconds, uint32_t seed)
{
    const uint32_t frames = (uint32_t)(seconds * (double)SAMPLE_RATE_HZ);

    return render_colour(dir, "white.wav", PCL_NOISE_WHITE, frames, seed)
        && render_colour(dir, "pink.wav", PCL_NOISE_PINK, frames, seed)
        && render_colour(dir, "brown.wav", PCL_NOISE_BROWN, frames, seed)
        && render_sah_pitch(dir, "sah_pitch.wav", 0u, frames, seed)
        && render_sah_pitch(dir, "sah_pitch_slewed.wav", SAH_SLEW_MS, frames, seed);
}

int main(int argc, char **argv)
{
    double   seconds = DEFAULT_SECONDS;
    uint32_t seed = 0u;

    if (argc < 3 || argc > 5
        || (strcmp(argv[1], "noise") != 0 && strcmp(argv[1], "lfo") != 0
            && strcmp(argv[1], "adsr") != 0 && strcmp(argv[1], "svf") != 0
            && strcmp(argv[1], "biquad") != 0 && strcmp(argv[1], "events") != 0))
    {
        fprintf(stderr, "usage: %s noise|lfo|adsr|svf|biquad|events <outdir> [seconds] [seed]\n",
            argv[0]);
        return 2;
    }
    if (argc >= 4)
    {
        seconds = atof(argv[3]);
        if (!(seconds >= MIN_SECONDS && seconds <= MAX_SECONDS))
        {
            fprintf(stderr, "seconds must be between %.2f and %.0f\n", MIN_SECONDS, MAX_SECONDS);
            return 2;
        }
    }
    if (argc == 5)
        seed = (uint32_t)strtoul(argv[4], NULL, 0);

    if (strcmp(argv[1], "lfo") == 0)
        return render_lfo(argv[2], seconds, seed) ? 0 : 1;
    if (strcmp(argv[1], "adsr") == 0)
        return render_adsr(argv[2], seconds, seed) ? 0 : 1;
    if (strcmp(argv[1], "svf") == 0)
        return render_svf(argv[2], seconds, seed) ? 0 : 1;
    if (strcmp(argv[1], "biquad") == 0)
        return render_biquad(argv[2], seconds, seed) ? 0 : 1;
    if (strcmp(argv[1], "events") == 0)
        return render_events(argv[2], seconds, seed) ? 0 : 1;
    return render_noise(argv[2], seconds, seed) ? 0 : 1;
}
