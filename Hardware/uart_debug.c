/**
 * @file    uart_debug.c
 * @brief   Debug UART implementation (UART0, 115200bps)
 *
 * Hardware: UART0, PA10(TX), PA11(RX)
 * SysConfig initialization is in Debug/ti_msp_dl_config.c
 */

#include "uart_debug.h"
#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* ========================================================================
 * Public Functions
 * ======================================================================== */

void Debug_Printf(const char *fmt, ...)
{
    /* UART0 is configured in loopback mode (DataScope).
     * For actual debug output, redirect to UART1 or disable loopback in SysConfig.
     * For now, output to UART1 (Bluetooth port). */
    char buf[128];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    for (int i = 0; i < len && i < (int)sizeof(buf); i++) {
        while (DL_UART_isBusy(UART_0_INST));
        DL_UART_Main_transmitDataBlocking(UART_0_INST, buf[i]);
    }
}

void Debug_SendBinary(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        while (DL_UART_isBusy(UART_0_INST));
        DL_UART_Main_transmitDataBlocking(UART_0_INST, data[i]);
    }
}

void Debug_PutChar(char ch)
{
    while (DL_UART_isBusy(UART_0_INST));
    DL_UART_Main_transmitDataBlocking(UART_0_INST, ch);
}
