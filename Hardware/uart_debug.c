/**
 * @file    uart_debug.c
 * @brief   调试 UART 实现 (UART0, 115200bps)
 *
 * 硬件：UART0, PA10(TX), PA11(RX)
 * SysConfig 初始化在 Debug/ti_msp_dl_config.c 中
 */

#include "uart_debug.h"
#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* ========================================================================
 * 公开函数
 * ======================================================================== */

void Debug_Printf(const char *fmt, ...)
{
    /* UART0 配置为回环模式 (DataScope)。
     * 如需实际调试输出，请重定向到 UART1 或禁用 SysConfig 中的回环。
     * 目前输出到 UART1（蓝牙端口）。 */
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
