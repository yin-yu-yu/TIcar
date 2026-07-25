/**
 * @file    debug_scope.c
 * @brief   虚拟示波器 — 封装 DataScope_DP 协议
 *
 * 底层协议实现见 Control/DataScope_DP.C。
 */

#include "debug_scope.h"

/* 前向声明，来自 Control/DataScope_DP.C */
extern unsigned char DataScope_OutPut_Buffer[42];

void DataScope_Get_Channel_Data(float Data, unsigned char Channel);
unsigned char DataScope_Data_Generate(unsigned char Channel_Number);

/* ========================================================================
 * 公开函数
 * ======================================================================== */

void Scope_SendChannel(float data, uint8_t channel)
{
    DataScope_Get_Channel_Data(data, channel);
}

void Scope_SendFrame(uint8_t num_channels)
{
    uint8_t count = DataScope_Data_Generate(num_channels);
    /* TODO: 通过 UART0 调试端口发送 DataScope_OutPut_Buffer
     * for (i = 0; i < count; i++) {
     *     Debug_PutChar(DataScope_OutPut_Buffer[i]);
     * } */
    (void)count;
}
