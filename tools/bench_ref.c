/*
 * bench_ref - writes the golden output of the benchmark cases (platform/bench) in
 * the same wire format the MCX N947 firmware streams, so the two can be compared byte for byte.
 *
 *     bench_ref <out.bin>
 */
#include <stdio.h>
#include <stdlib.h>

#include "pcl_bench.h"

static FILE *g_file;

static void sink(const void *data, size_t bytes)
{
    if (fwrite(data, 1, bytes, g_file) != bytes)
    {
        perror("bench_ref: write");
        exit(1);
    }
}

int main(int argc, char **argv)
{
    size_t id;

    if (argc != 2)
    {
        fprintf(stderr, "usage: %s <out.bin>\n", argv[0]);
        return 2;
    }
    g_file = fopen(argv[1], "wb");
    if (g_file == NULL)
    {
        perror(argv[1]);
        return 1;
    }
    pcl_bench_emit_banner(sink);
    for (id = 0; id < pcl_bench_case_count(); id++)
    {
        if (pcl_bench_emit(id, NULL, sink) != PCL_OK)
        {
            fprintf(stderr, "bench_ref: case %zu failed\n", id);
            return 1;
        }
    }
    return fclose(g_file) == 0 ? 0 : 1;
}
