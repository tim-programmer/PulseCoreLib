/*
 * play_sample - plays a short melody from a WAV file through pcl_sampler (a desktop check of the
 * sampler on real recordings).
 *
 *     play_sample <in.wav> <out.wav> [root_note]
 *
 * Reads 16-bit PCM (mono, or stereo averaged to mono) with tools/wav_read.c. If the file has a
 * `smpl` loop the sample plays in SUSTAIN mode (looping while each note is held), otherwise
 * one-shot. The melody is root, +4, +7, +12, +7, +4, root semitones, 0.4 s apart, each held 0.3 s.
 * Output: mono 16-bit PCM at 48 kHz. Host-only: file I/O and malloc are fine here, never in src/.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pcl/pcl.h"
#include "wav_read.h"

#define OUT_RATE_HZ 48000u
#define BLOCK 256u
#define NOTE_PERIOD_S 0.4
#define NOTE_HOLD_S 0.3
#define TAIL_S 1.5

static const int g_melody[] = { 0, 4, 7, 12, 7, 4, 0 };

static void put_u32(FILE *f, uint32_t v)
{
    const unsigned char b[4] = { (unsigned char)v, (unsigned char)(v >> 8),
        (unsigned char)(v >> 16), (unsigned char)(v >> 24) };
    fwrite(b, 1, 4, f);
}

static void put_u16(FILE *f, uint16_t v)
{
    const unsigned char b[2] = { (unsigned char)v, (unsigned char)(v >> 8) };
    fwrite(b, 1, 2, f);
}

static int16_t to_pcm16(pcl_sample_t s)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    const float v = s * 32768.0f;
    return v >= 32767.0f ? 32767 : (v <= -32768.0f ? -32768 : (int16_t)v);
#elif PCL_SAMPLE_TYPE == PCL_Q15
    return s;
#else
    return (int16_t)(s >> 16);
#endif
}

int main(int argc, char **argv)
{
    wav_data_t           w;
    char                 err[128];
    pcl_sampler_sample_t smp;
    pcl_sampler_config_t cfg;
    pcl_sampler_t        s;
    const size_t         notes = sizeof g_melody / sizeof g_melody[0];
    const uint32_t       frames
        = (uint32_t)(((double)notes * NOTE_PERIOD_S + TAIL_S) * (double)OUT_RATE_HZ);
    uint32_t done = 0;
    int      root = 60;
    FILE    *f;

    if (argc < 3 || argc > 4)
    {
        fprintf(stderr, "usage: %s <in.wav> <out.wav> [root_note]\n", argv[0]);
        return 2;
    }
    if (argc == 4)
        root = atoi(argv[3]);
    if (root < 0 || root > 115)
    {
        fprintf(stderr, "root_note must be 0 .. 115 (the melody goes an octave up)\n");
        return 2;
    }
    if (wav_read_pcm16(argv[1], &w, err, sizeof err) != 0)
    {
        fprintf(stderr, "%s: %s\n", argv[1], err);
        return 1;
    }
    smp.data = w.frames;
    smp.frames = w.count;
    smp.format = PCL_SAMPLER_Q15;
    smp.sample_rate_hz = w.sample_rate_hz;
    smp.root_note = (uint8_t)root;
    smp.loop_mode = w.has_loop ? PCL_SAMPLER_SUSTAIN : PCL_SAMPLER_ONE_SHOT;
    smp.loop_start = w.loop_start;
    smp.loop_end = w.loop_end;
    cfg.sample_rate_hz = OUT_RATE_HZ;
    cfg.sample = &smp;
    cfg.interp = PCL_SAMPLER_HERMITE;
    cfg.pitch_depth_oct = PCL_OCT(0.0);
    cfg.gain = PCL_COEF(1.0);
    cfg.velocity_sens = PCL_COEF(0.0);
    cfg.start_offset = 0;
    cfg.reverse = 0;
    cfg.declick_ms = 3;
    if (pcl_sampler_init(&s, &cfg) != PCL_OK)
    {
        fprintf(stderr, "%s: the sampler rejected this file (rate %u Hz, %u frames)\n", argv[1],
            (unsigned)w.sample_rate_hz, (unsigned)w.count);
        wav_free(&w);
        return 1;
    }

    f = fopen(argv[2], "wb");
    if (f == NULL)
    {
        perror(argv[2]);
        wav_free(&w);
        return 1;
    }
    fwrite("RIFF", 1, 4, f);
    put_u32(f, 36u + frames * 2u);
    fwrite("WAVEfmt ", 1, 8, f);
    put_u32(f, 16u);
    put_u16(f, 1u);
    put_u16(f, 1u);
    put_u32(f, OUT_RATE_HZ);
    put_u32(f, OUT_RATE_HZ * 2u);
    put_u16(f, 2u);
    put_u16(f, 16u);
    fwrite("data", 1, 4, f);
    put_u32(f, frames * 2u);

    while (done < frames)
    {
        const uint32_t n = frames - done < BLOCK ? frames - done : BLOCK;
        pcl_event_t    ev[4];
        size_t         cnt = 0, k;
        pcl_sample_t   out[BLOCK];
        int16_t        pcm[BLOCK];

        for (k = 0; k < notes; k++)
        {
            const uint32_t on = (uint32_t)((double)k * NOTE_PERIOD_S * OUT_RATE_HZ);
            const uint32_t off = on + (uint32_t)(NOTE_HOLD_S * OUT_RATE_HZ);
            if (on >= done && on < done + n && cnt < 4u)
            {
                ev[cnt].offset = (uint16_t)(on - done);
                ev[cnt].type = PCL_EVT_NOTE_ON;
                ev[cnt].note = (uint8_t)(root + g_melody[k]);
                ev[cnt].velocity = 110;
                cnt++;
            }
            if (off >= done && off < done + n && cnt < 4u)
            {
                ev[cnt].offset = (uint16_t)(off - done);
                ev[cnt].type = PCL_EVT_NOTE_OFF;
                ev[cnt].note = (uint8_t)(root + g_melody[k]);
                ev[cnt].velocity = 0;
                cnt++;
            }
        }
        pcl_sampler_process(&s, out, NULL, ev, cnt, n);
        for (k = 0; k < n; k++)
            pcm[k] = to_pcm16(out[k]);
        for (k = 0; k < n; k++)
            put_u16(f, (uint16_t)pcm[k]);
        done += n;
    }
    if (fclose(f) != 0)
    {
        perror(argv[2]);
        wav_free(&w);
        return 1;
    }
    printf("wrote %s: %u frames from %s (%u Hz, %u frames, %s)\n", argv[2], (unsigned)frames,
        argv[1], (unsigned)w.sample_rate_hz, (unsigned)w.count,
        w.has_loop ? "sustain loop from smpl" : "one-shot");
    wav_free(&w);
    return 0;
}
