/**
 * @file    uart_debug.h
 * @brief   Debug UART interface (UART0, 115200bps)
 *
 * Used for debug printf, DataScope virtual oscilloscope,
 * and development-time diagnostics.
 *
 * Hardware: UART0, PA10(TX), PA11(RX)
 */

#ifndef _UART_DEBUG_H_
#define _UART_DEBUG_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Public Functions
 * ======================================================================== */

/**
 * @brief  Send formatted debug string (printf style)
 * @param  fmt  Format string
 * @param  ...  Variable arguments
 */
void Debug_Printf(const char *fmt, ...);

/**
 * @brief  Send raw binary data over debug UART
 * @param  data  Data buffer
 * @param  len   Number of bytes to send
 */
void Debug_SendBinary(const uint8_t *data, uint16_t len);

/**
 * @brief  Send a single character
 * @param  ch  Character to send
 */
void Debug_PutChar(char ch);

#ifdef __cplusplus
}
#endif

#endif /* _UART_DEBUG_H_ */
