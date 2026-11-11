/* wav_read.c - see wav_read.h. Host-only. */
#include "wav_read.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define WAV_FORMAT_PCM 1u
#define WAV_SMPL_LOOP_OFFSET 36u /* the first loop record inside a smpl chunk */
#define WAV_SMPL_LOOP_BYTES 24u

static uint32_t le32(const unsigned char *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint16_t le16(const unsigned char *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

static int fail(char *err, size_t n, const char *msg)
{
    snprintf(err, n, "%s", msg);
    return 1;
}

int wav_read_pcm16(const char *path, wav_data_t *out, char *err, size_t err_len)
{
    FILE                *f = fopen(path, "rb");
    unsigned char       *buf = NULL;
    long                 size;
    size_t               pos;
    uint16_t             channels = 0, bits = 0;
    const unsigned char *data = NULL;
    uint32_t             data_bytes = 0;
    int                  have_fmt = 0;

    memset(out, 0, sizeof *out);
    if (f == NULL)
        return fail(err, err_len, "cannot open file");
    if (fseek(f, 0, SEEK_END) != 0 || (size = ftell(f)) < 12 || fseek(f, 0, SEEK_SET) != 0)
    {
        fclose(f);
        return fail(err, err_len, "not a WAV file (too short)");
    }
    buf = (unsigned char *)malloc((size_t)size);
    if (buf == NULL || fread(buf, 1, (size_t)size, f) != (size_t)size)
    {
        free(buf);
        fclose(f);
        return fail(err, err_len, "read error");
    }
    fclose(f);
    if (memcmp(buf, "RIFF", 4) != 0 || memcmp(buf + 8, "WAVE", 4) != 0)
    {
        free(buf);
        return fail(err, err_len, "not a RIFF/WAVE file");
    }

    for (pos = 12; pos + 8 <= (size_t)size;)
    {
        const uint32_t       len = le32(buf + pos + 4);
        const unsigned char *body = buf + pos + 8;
        if (len > (size_t)size - pos - 8)
            break; /* truncated chunk: use what was found */
        if (memcmp(buf + pos, "fmt ", 4) == 0 && len >= 16)
        {
            if (le16(body) != WAV_FORMAT_PCM)
            {
                free(buf);
                return fail(err, err_len, "only PCM WAV files are supported");
            }
            channels = le16(body + 2);
            out->sample_rate_hz = le32(body + 4);
            bits = le16(body + 14);
            have_fmt = 1;
        }
        else if (memcmp(buf + pos, "data", 4) == 0)
        {
            data = body;
            data_bytes = len;
        }
        else if (memcmp(buf + pos, "smpl", 4) == 0
            && len >= WAV_SMPL_LOOP_OFFSET + WAV_SMPL_LOOP_BYTES && le32(body + 28) >= 1u)
        {
            out->has_loop = 1;
            out->loop_start = le32(body + WAV_SMPL_LOOP_OFFSET + 8);
            out->loop_end = le32(body + WAV_SMPL_LOOP_OFFSET + 12) + 1u;
        }
        pos += 8u + len + (len & 1u); /* chunks are padded to even sizes */
    }
    if (!have_fmt || data == NULL)
    {
        free(buf);
        return fail(err, err_len, "missing fmt or data chunk");
    }
    if (bits != 16u || (channels != 1u && channels != 2u))
    {
        free(buf);
        return fail(err, err_len, "only 16-bit mono or stereo PCM is supported");
    }

    out->count = data_bytes / (2u * channels);
    out->frames = (int16_t *)malloc((size_t)out->count * sizeof(int16_t) + 1u);
    if (out->frames == NULL)
    {
        free(buf);
        return fail(err, err_len, "out of memory");
    }
    {
        uint32_t i;
        for (i = 0; i < out->count; i++)
        {
            const unsigned char *p = data + (size_t)i * 2u * channels;
            int32_t              v = (int16_t)le16(p);
            if (channels == 2u)
                v = (v + (int16_t)le16(p + 2)) / 2;
            out->frames[i] = (int16_t)v;
        }
    }
    if (out->has_loop && (out->loop_end > out->count || out->loop_start + 2u > out->loop_end))
        out->has_loop = 0; /* an unusable loop: play one-shot */
    free(buf);
    return 0;
}

void wav_free(wav_data_t *w)
{
    free(w->frames);
    w->frames = NULL;
    w->count = 0;
}
