/*
 * Oscillator benchmark firmware for the NXP FRDM-MCXN947 (MCX N947, Cortex-M33 + FPU).
 *
 * On reset it prints a banner on the board's console UART (LPUART4, bridged to a USB serial
 * port by the on-board MCU-Link). A host sends 'R'; the firmware then renders every case from
 * platform/bench, times each block with the core's DWT cycle counter, and streams the results.
 * Run from flash at 150 MHz with the SDK's default configuration (flash cache enabled).
 *
 * Driven by scripts/mcxn947_bench.py; see the README.
 */
#include "board_hw.h"
#include "clock_config.h"
#include "fsl_clock.h"
#include "fsl_common.h"
#include "fsl_lpflexcomm.h"
#include "fsl_lpuart.h"
#include "pin_mux.h"

#include "pcl_bench.h"

#ifndef BENCH_UART_BAUD
#define BENCH_UART_BAUD 921600u
#endif
#define BENCH_UART_CLK_HZ 12000000u /* FRO12M attached to FLEXCOMM4 */
#define BENCH_COMMAND_RUN 'R'

static void uart_init(void)
{
    lpuart_config_t cfg;

    CLOCK_SetClkDiv(kCLOCK_DivFlexcom4Clk, 1u);
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM4);
    /* Select the UART function of the shared FLEXCOMM block. LPUART_Init() does not do this
     * itself, and touching an LPUART register before it faults the bus. */
    (void)LP_FLEXCOMM_Init(4u, LP_FLEXCOMM_PERIPH_LPUART);
    LPUART_GetDefaultConfig(&cfg);
    cfg.baudRate_Bps = BENCH_UART_BAUD;
    cfg.enableTx = true;
    cfg.enableRx = true;
    (void)LPUART_Init(LPUART4, &cfg, BENCH_UART_CLK_HZ);
}

static void uart_sink(const void *data, size_t bytes)
{
    (void)LPUART_WriteBlocking(LPUART4, (const uint8_t *)data, bytes);
}

static uint32_t cycle_counter(void) { return DWT->CYCCNT; }

/*
 * Measures the core clock by counting DWT cycles while a known number of bytes leaves the UART
 * (each byte is 10 bit-times). Returns Hz. Sends spaces, which the host ignores.
 */
static uint32_t measure_cpu_hz(void)
{
    static const uint8_t spaces[200] = { [0 ... 199] = ' ' };
    const uint32_t       t0 = DWT->CYCCNT;
    uint32_t             cycles;

    (void)LPUART_WriteBlocking(LPUART4, spaces, sizeof spaces);
    while ((LPUART_GetStatusFlags(LPUART4) & (uint32_t)kLPUART_TransmissionCompleteFlag) == 0u)
    {
    }
    cycles = DWT->CYCCNT - t0;
    /* cycles / seconds, seconds = bytes * 10 / baud */
    return (uint32_t)(((uint64_t)cycles * BENCH_UART_BAUD) / (sizeof spaces * 10u));
}

static void hw_info(void)
{
    char           line[64];
    const uint32_t hz = measure_cpu_hz();
    const unsigned lpcac = (SYSCON->LPCAC_CTRL & SYSCON_LPCAC_CTRL_DIS_LPCAC_MASK) == 0u;
    int            n = 0;
    const char    *head = "\nHPCLHW cpu_hz=";
    uint32_t       div;
    int            started = 0;

    while (*head != '\0')
        line[n++] = *head++;
    for (div = 1000000000u; div != 0u; div /= 10u)
    {
        const uint32_t digit = (hz / div) % 10u;
        if (digit != 0u || started || div == 1u)
        {
            line[n++] = (char)('0' + digit);
            started = 1;
        }
    }
    for (head = " lpcac="; *head != '\0';)
        line[n++] = *head++;
    line[n++] = (char)('0' + lpcac);
    line[n++] = '\n';
    uart_sink(line + 1, (size_t)(n - 1));
}

static void cycle_counter_init(void)
{
    DCB->DEMCR |= DCB_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

int main(void)
{
    BOARD_InitPins();
    BOARD_BootClockPLL150M();
    uart_init();
    cycle_counter_init();
#ifdef BENCH_DISABLE_LPCAC
    SYSCON->LPCAC_CTRL |= SYSCON_LPCAC_CTRL_DIS_LPCAC_MASK; /* flash cache off, for comparison */
#endif

    for (;;)
    {
        uint8_t cmd = 0;
        size_t  id;

        hw_info();
        pcl_bench_emit_banner(uart_sink);
        /* Wait for the host; repeat the banner every so often so it can attach at any time. */
        for (uint32_t spin = 0; spin < 150000000u / 4u; spin++)
        {
            if ((LPUART_GetStatusFlags(LPUART4) & (uint32_t)kLPUART_RxDataRegFullFlag) != 0u)
            {
                cmd = (uint8_t)LPUART_ReadByte(LPUART4);
                break;
            }
        }
        if (cmd != BENCH_COMMAND_RUN)
            continue;

        for (id = 0; id < pcl_bench_case_count(); id++)
            (void)pcl_bench_emit(id, cycle_counter, uart_sink);
        uart_sink("DONE\n", 5u);
    }
}
