/**
 * @file    debug_scope.h
 * @brief   Virtual oscilloscope / DataScope debug interface
 *
 * Sends real-time variable data to upper-computer visualization
 * software via UART for debugging purposes.
 *
 * Uses the DataScope_DP protocol (Control/DataScope_DP.C).
 */

#ifndef _DEBUG_SCOPE_H_
#define _DEBUG_SCOPE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Public Functions
 * ======================================================================== */

/**
 * @brief  Send one channel of data to the virtual oscilloscope
 * @param  data     Float value to send
 * @param  channel  Channel index (1~10)
 */
void Scope_SendChannel(float data, uint8_t channel);

/**
 * @brief  Flush all buffered channels as a complete data frame
 * @param  num_channels  Number of channels in this frame
 */
void Scope_SendFrame(uint8_t num_channels);

#ifdef __cplusplus
}
#endif

#endif /* _DEBUG_SCOPE_H_ */
