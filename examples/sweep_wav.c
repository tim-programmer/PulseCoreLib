/*
 * sweep_wav - renders a slow exponential sweep from 20 Hz to 20 kHz to a WAV file.
 *
 *     sweep_wav <out.wav> [sine|triangle|saw|pulse] [seconds]
 *
 * Defaults: saw, 20 seconds. The output is mono, 16-bit PCM at 48 kHz (the rate is fixed:
 * the oscillator tops out at 0.45 * sample rate, and 20 kHz needs 48 kHz or more).
 * Turn the volume down before playing the top end of the sweep.
 *
 * What it demonstrates:
 *   - The oscillator's pitch input: the base frequency is the geometric middle of the range
 *     (632.46 Hz) and the pitch depth is half the span in octaves (4.98), so a modulation
 *     signal ramping from -1 to +1 sweeps 20 Hz .. 20 kHz at a constant octaves-per-second.
 *   - The VCA's modulation input: a short fade in and out, so the file starts and ends
 *     without clicks.
 * Everything is block-based and allocation-free; only this example does file I/O.
 *
 * In Q15/Q31 builds the pitch input is quantized (one Q15 step is about 0.26 cent at this
 * depth), which is far below audibility on a sweep this slow.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pcl/pcl.h"

#define SAMPLE_RATE_HZ 48000u
#define BLOCK_FRAMES 256u
#define SWEEP_START_HZ 20.0
#define SWEEP_END_HZ 20000.0
#define FADE_SECONDS 0.05
#define OUTPUT_GAIN 0.5
#define DEFAULT_SECONDS 20.0
#define MIN_SECONDS 0.1
#define MAX_SECONDS 600.0

/* ---- Sample conversion (host-side; the library itself uses no float in Q15/Q31) ---- */

static pcl_sample_t sample_from_double(double v)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return (float)v;
#elif PCL_SAMPLE_TYPE == PCL_Q15
    const double s = floor(v * 32768.0 + 0.5);
    return s >= 32767.0 ? INT16_MAX : (s <= -32768.0 ? INT16_MIN : (int16_t)s);
#else
    const double s = floor(v * 2147483648.0 + 0.5);
    return s >= 2147483647.0 ? INT32_MAX : (s <= -2147483648.0 ? INT32_MIN : (int32_t)s);
#endif
}

static int16_t pcm16_from_sample(pcl_sample_t x)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    const double s = floor((double)x * 32767.0 + 0.5);
    return s >= 32767.0 ? INT16_MAX : (s <= -32768.0 ? INT16_MIN : (int16_t)s);
#elif PCL_SAMPLE_TYPE == PCL_Q15
    return x;
#else
    return (int16_t)(x >> 16);
#endif
}

/* ---- WAV output ---- */

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

/* 44-byte canonical header: mono, 16-bit PCM. */
static int write_wav_header(FILE *f, uint32_t sample_rate_hz, uint32_t frames)
{
    unsigned char  h[44];
    const uint32_t data_bytes = frames * 2u;

    memcpy(h + 0, "RIFF", 4);
    put_le32(h + 4, 36u + data_bytes);
    memcpy(h + 8, "WAVEfmt ", 8);
    put_le32(h + 16, 16u); /* fmt chunk size */
    put_le16(h + 20, 1u);  /* PCM            */
    put_le16(h + 22, 1u);  /* channels       */
    put_le32(h + 24, sample_rate_hz);
    put_le32(h + 28, sample_rate_hz * 2u); /* byte rate   */
    put_le16(h + 32, 2u);                  /* block align */
    put_le16(h + 34, 16u);                 /* bits        */
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

/* ---- Main ---- */

static int parse_waveform(const char *name, pcl_osc_waveform_t *wf)
{
    static const char *const names[] = { "sine", "triangle", "saw", "pulse" };
    int                      i;

    for (i = 0; i < 4; i++)
    {
        if (strcmp(name, names[i]) == 0)
        {
            *wf = (pcl_osc_waveform_t)i;
            return 1;
        }
    }
    return 0;
}

int main(int argc, char **argv)
{
    pcl_osc_waveform_t waveform = PCL_OSC_SAW;
    double             seconds = DEFAULT_SECONDS;
    const char        *path;
    FILE              *file;
    uint32_t           total_frames;
    uint32_t           done = 0;
    double             fade_frames;
    pcl_osc_config_t   osc_cfg;
    pcl_vca_config_t   vca_cfg;
    pcl_osc_t          osc;
    pcl_vca_t          vca;
    pcl_sample_t       pitch_mod[BLOCK_FRAMES];
    pcl_sample_t       fade[BLOCK_FRAMES];
    pcl_sample_t       audio[BLOCK_FRAMES];
    int16_t            pcm[BLOCK_FRAMES];

    if (argc < 2 || argc > 4)
    {
        fprintf(stderr, "usage: %s <out.wav> [sine|triangle|saw|pulse] [seconds]\n", argv[0]);
        return 2;
    }
    path = argv[1];
    if (argc >= 3 && !parse_waveform(argv[2], &waveform))
    {
        fprintf(stderr, "unknown waveform '%s' (use sine, triangle, saw or pulse)\n", argv[2]);
        return 2;
    }
    if (argc == 4)
    {
        seconds = atof(argv[3]);
        if (!(seconds >= MIN_SECONDS && seconds <= MAX_SECONDS))
        {
            fprintf(stderr, "seconds must be between %.1f and %.0f\n", MIN_SECONDS, MAX_SECONDS);
            return 2;
        }
    }

    total_frames = (uint32_t)(seconds * (double)SAMPLE_RATE_HZ);
    fade_frames = FADE_SECONDS * (double)SAMPLE_RATE_HZ;

    osc_cfg.sample_rate_hz = SAMPLE_RATE_HZ;
    osc_cfg.waveform = waveform;
    osc_cfg.frequency = PCL_HZ(sqrt(SWEEP_START_HZ * SWEEP_END_HZ));
    osc_cfg.pitch_depth_oct = PCL_OCT(0.5 * log2(SWEEP_END_HZ / SWEEP_START_HZ));
    osc_cfg.pulse_width = PCL_COEF(0.5);
    osc_cfg.pwm_depth = PCL_COEF(0.0);
    vca_cfg.sample_rate_hz = SAMPLE_RATE_HZ;
    vca_cfg.gain = PCL_COEF(OUTPUT_GAIN);
    if (pcl_osc_init(&osc, &osc_cfg) != PCL_OK || pcl_vca_init(&vca, &vca_cfg) != PCL_OK)
    {
        fprintf(stderr, "oscillator/VCA configuration rejected\n");
        return 1;
    }

    file = fopen(path, "wb");
    if (file == NULL)
    {
        perror(path);
        return 1;
    }
    if (!write_wav_header(file, SAMPLE_RATE_HZ, total_frames))
    {
        perror(path);
        fclose(file);
        return 1;
    }

    while (done < total_frames)
    {
        const uint32_t n
            = (total_frames - done < BLOCK_FRAMES) ? (total_frames - done) : BLOCK_FRAMES;
        uint32_t i;

        for (i = 0; i < n; i++)
        {
            const double pos = (double)(done + i);
            const double last = (double)(total_frames - 1u);
            const double up = pos / fade_frames;
            const double down = (last - pos) / fade_frames;
            const double env = up < down ? up : down;

            /* -1 at the first frame, +1 at the last: octaves = mod * pitch_depth_oct. */
            pitch_mod[i] = sample_from_double(last > 0.0 ? -1.0 + 2.0 * pos / last : 0.0);
            fade[i] = sample_from_double(env > 1.0 ? 1.0 : env);
        }

        pcl_osc_process(&osc, audio, pitch_mod, NULL, NULL, NULL, n);
        pcl_vca_process(&vca, audio, audio, fade, n); /* in place */

        for (i = 0; i < n; i++)
            pcm[i] = pcm16_from_sample(audio[i]);
        if (!write_pcm16(file, pcm, n))
        {
            perror(path);
            fclose(file);
            return 1;
        }
        done += n;
    }

    if (fclose(file) != 0)
    {
        perror(path);
        return 1;
    }
    printf(
        "wrote %s: %.1f s of %s, %.0f Hz -> %.0f Hz (exponential), %u Hz mono 16-bit, %u bytes\n",
        path, seconds, argc >= 3 ? argv[2] : "saw", SWEEP_START_HZ, SWEEP_END_HZ, SAMPLE_RATE_HZ,
        44u + total_frames * 2u);
    return 0;
}
