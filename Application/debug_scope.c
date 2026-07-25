/**
 * @file    debug_scope.c
 * @brief   Virtual oscilloscope — wraps DataScope_DP protocol
 *
 * See Control/DataScope_DP.C for the underlying protocol implementation.
 */

#include "debug_scope.h"

/* Forward declarations from Control/DataScope_DP.C */
extern unsigned char DataScope_OutPut_Buffer[42];

void DataScope_Get_Channel_Data(float Data, unsigned char Channel);
unsigned char DataScope_Data_Generate(unsigned char Channel_Number);

/* ========================================================================
 * Public Functions
 * ======================================================================== */

void Scope_SendChannel(float data, uint8_t channel)
{
    DataScope_Get_Channel_Data(data, channel);
}

void Scope_SendFrame(uint8_t num_channels)
{
    uint8_t count = DataScope_Data_Generate(num_channels);
    /* TODO: Send DataScope_OutPut_Buffer over UART0 debug port
     * for (i = 0; i < count; i++) {
     *     Debug_PutChar(DataScope_OutPut_Buffer[i]);
     * } */
    (void)count;
}
