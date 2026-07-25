/**
 * @file    debug_scope.c
 * @brief   虚拟示波器 — 封装 DataScope_DP 协议
 * @brief   Virtual oscilloscope — wraps DataScope_DP protocol
 *
 * Sends real-time variable data to upper-computer visualization
 * software (e.g., WitMotion DataScope) via UART0 debug port.
 *
 * Protocol: See Control/DataScope_DP.C for the underlying implementation.
 *
 * Usage:
 *   1. Scope_SendChannel(value, channel) — queue one channel's data
 *   2. Scope_SendFrame(num_channels)     — flush all channels as one frame
 *
 * Example (3 channels at 200Hz):
 *   Scope_SendChannel(enc_speed_l, 1);
 *   Scope_SendChannel(enc_speed_r, 2);
 *   Scope_SendChannel(batt_voltage, 3);
 *   Scope_SendFrame(3);
 */

#include "debug_scope.h"
#include "uart_debug.h"
#include "DataScope_DP.h"

/* ========================================================================
 * 公开函数
 * Public Functions
 * ======================================================================== */

void Scope_SendChannel(float data, uint8_t channel)
{
    DataScope_Get_Channel_Data(data, channel);
}

/**
 * @brief  Generate and send one complete data frame over debug UART
 * @param  num_channels  Number of channels (1~10) in this frame
 *
 * Calls DataScope_Data_Generate() to build the protocol frame,
 * then sends the raw bytes over UART0 to the host PC.
 */
void Scope_SendFrame(uint8_t num_channels)
{
    if (num_channels == 0 || num_channels > 10) {
        return;
    }

    /* Generate the binary protocol frame */
    uint8_t count = DataScope_Data_Generate(num_channels);

    /* Send over UART0 debug port to host PC */
    if (count > 0 && count <= sizeof(DataScope_OutPut_Buffer)) {
        Debug_SendBinary(DataScope_OutPut_Buffer, count);
    }
}
