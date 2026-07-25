/**
 * @file    uart_bt.c
 * @brief   Bluetooth UART implementation (UART1, 9600bps, DMA)
 *
 * Hardware: UART1, PB6(TX), PB7(RX), DMA CH0
 * SysConfig initialization is in Debug/ti_msp_dl_config.c
 *
 * Most BT logic is in Application/bt_protocol.c (to be filled by team member).
 * This file provides low-level UART+DMA plumbing.
 */

#include "uart_bt.h"
#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* ========================================================================
 * Module Variables
 * ======================================================================== */
#define BT_PACKET_SIZE  200
static volatile uint8_t g_bt_buffer[BT_PACKET_SIZE];

/* ========================================================================
 * Public Functions
 * ======================================================================== */

void BT_Init(void)
{
    /* DMA and UART1 already initialized by SysConfig.
     * Configure DMA for first reception. */
    BT_DMAConfig();
}

bool BT_DataAvailable(void)
{
    uint8_t recv = BT_PACKET_SIZE - DL_DMA_getTransferSize(DMA, DMA_CH0_CHAN_ID);
    return (recv > 0);
}

uint8_t BT_ReadByte(void)
{
    /* Direct register read — used when not in DMA mode */
    if (DL_UART_Main_isRXFIFOEmpty(UART_1_INST)) {
        return 0;
    }
    return DL_UART_Main_receiveData(UART_1_INST);
}

void BT_SendBytes(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        while (DL_UART_isBusy(UART_1_INST));
        DL_UART_Main_transmitDataBlocking(UART_1_INST, data[i]);
    }
}

void BT_Printf(const char *fmt, ...)
{
    char buf[200];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    BT_SendBytes((const uint8_t *)buf, (len > 0) ? (uint16_t)len : 0);
}

void BT_DMAConfig(void)
{
    DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID);
    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)(&UART_1_INST->RXDATA));
    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)&g_bt_buffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, BT_PACKET_SIZE);
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
}
