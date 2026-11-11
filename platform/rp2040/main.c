/*
 * Benchmark firmware for an RP2040 (Raspberry Pi Pico: Cortex-M0+, no FPU), the same cases and wire
 * format as platform/mcxn947.
 *
 * It prints a hardware line and the banner over USB serial about once a second. A host sends 'R';
 * the firmware then renders every case from platform/bench, times each block, and streams the
 * results. Runs from flash through the XIP cache at the SDK's default 125 MHz.
 *
 * Timing: the M0+ has no DWT cycle counter, so the core's 24-bit SysTick counts processor cycles and
 * cycle_counter() widens it to 32 bits by accumulating the difference between reads. That is exact as
 * long as reads are less than 2^24 cycles (134 ms) apart, which holds within a case (one block is far
 * shorter). Interrupts are off while a case runs, so USB servicing never lands inside a measurement;
 * they come back on with the first byte of the case's output.
 *
 * Driven by scripts/rp2040_bench.py; see the README.
 */
#include <stdio.h>

#include "hardware/clocks.h"
#include "hardware/structs/systick.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#include "pcl_bench.h"

#define BENCH_COMMAND_RUN 'R'
#define BENCH_BANNER_PERIOD_US 1000000u
#define SYSTICK_MASK 0x00FFFFFFu
#define SYSTICK_CSR_ENABLE_CPU_CLOCK 0x5u /* ENABLE | CLKSOURCE = processor clock, no interrupt */

static uint32_t g_systick_last;
static uint32_t g_cycles;
static uint32_t g_irq_state;
static int      g_irq_off;

static void systick_init(void)
{
    systick_hw->csr = 0u;
    systick_hw->rvr = SYSTICK_MASK;
    systick_hw->cvr = 0u;
    systick_hw->csr = SYSTICK_CSR_ENABLE_CPU_CLOCK;
    g_systick_last = systick_hw->cvr & SYSTICK_MASK;
}

/* A 32-bit cycle count from the 24-bit down-counter (see the file comment). */
static uint32_t cycle_counter(void)
{
    const uint32_t now = systick_hw->cvr & SYSTICK_MASK;

    g_cycles += (g_systick_last - now) & SYSTICK_MASK;
    g_systick_last = now;
    return g_cycles;
}

static void usb_sink(const void *data, size_t bytes)
{
    if (g_irq_off)
    {
        restore_interrupts(g_irq_state);
        g_irq_off = 0;
    }
    (void)stdio_put_string((const char *)data, (int)bytes, false, false);
}

static void hw_info(void)
{
    char      line[64];
    const int n = snprintf(line, sizeof line, "HPCLHW cpu_hz=%lu board=rp2040\n",
        (unsigned long)clock_get_hz(clk_sys));

    if (n > 0)
        usb_sink(line, (size_t)n);
}

int main(void)
{
    stdio_init_all();
    systick_init();

    for (;;)
    {
        int    cmd;
        size_t id;

        hw_info();
        pcl_bench_emit_banner(usb_sink);
        stdio_flush();
        cmd = getchar_timeout_us(BENCH_BANNER_PERIOD_US);
        if (cmd != BENCH_COMMAND_RUN)
            continue;

        for (id = 0; id < pcl_bench_case_count(); id++)
        {
            g_irq_state = save_and_disable_interrupts();
            g_irq_off = 1;
            (void)cycle_counter(); /* resynchronise after the idle time between cases */
            (void)pcl_bench_emit(id, cycle_counter, usb_sink);
            if (g_irq_off)
            {
                restore_interrupts(g_irq_state);
                g_irq_off = 0;
            }
        }
        usb_sink("DONE\n", 5u);
        stdio_flush();
    }
}
