/**
 * @file    debug_scope.c
 * @brief   虚拟示波器 — 封装 DataScope_DP 协议
 *
 * 通过 UART0 调试端口向上位机可视化软件（如 WitMotion DataScope）
 * 发送实时变量数据。
 *
 * 协议：底层实现见 Control/DataScope_DP.C。
 *
 * 用法：
 *   1. Scope_SendChannel(value, channel) — 排队一个通道的数据
 *   2. Scope_SendFrame(num_channels)     — 将所有通道作为一帧数据刷新发送
 *
 * 示例（200Hz，3 通道）：
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
 * ======================================================================== */

void Scope_SendChannel(float data, uint8_t channel)
{
    DataScope_Get_Channel_Data(data, channel);
}

/**
 * @brief  生成并通过调试 UART 发送一个完整数据帧
 * @param  num_channels  本帧中的通道数 (1~10)
 *
 * 调用 DataScope_Data_Generate() 构建协议帧，
 * 然后通过 UART0 将原始字节发送到上位机。
 */
void Scope_SendFrame(uint8_t num_channels)
{
    if (num_channels == 0 || num_channels > 10) {
        return;
    }

    /* 生成二进制协议帧 */
    uint8_t count = DataScope_Data_Generate(num_channels);

    /* 通过 UART0 调试端口发送到上位机 */
    if (count > 0 && count <= sizeof(DataScope_OutPut_Buffer)) {
        Debug_SendBinary(DataScope_OutPut_Buffer, count);
    }
}
