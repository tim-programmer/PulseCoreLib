/*
 * wav_read.h - minimal WAV reader for the host tools (never part of the library: file I/O and heap
 * belong outside src/, CLAUDE.md hard rules 1 and 6).
 *
 * Reads 16-bit PCM, mono or stereo (stereo is averaged to mono), and the first loop of a `smpl`
 * chunk if there is one. Other formats are refused with a message.
 */
#ifndef WAV_READ_H
#define WAV_READ_H

#include <stddef.h>
#include <stdint.h>

typedef struct
{
    int16_t *frames; /* malloc'd; free with wav_free                     */
    uint32_t count;
    uint32_t sample_rate_hz;
    int      has_loop;   /* a `smpl` chunk gave a loop                        */
    uint32_t loop_start; /* frames                                            */
    uint32_t loop_end;   /* exclusive (smpl stores the last frame, inclusive) */
} wav_data_t;

/* Returns 0 on success; on failure, 1 with a message in err. */
int  wav_read_pcm16(const char *path, wav_data_t *out, char *err, size_t err_len);
void wav_free(wav_data_t *w);

#endif /* WAV_READ_H */
