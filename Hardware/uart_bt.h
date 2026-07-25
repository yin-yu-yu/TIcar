/**
 * @file    uart_bt.h
 * @brief   Bluetooth UART interface (UART1, 9600bps, DMA receive)
 *
 * Communicates with HC-05/HC-06 Bluetooth module for
 * smartphone APP remote control and parameter tuning.
 *
 * Hardware: UART1, PB6(TX), PB7(RX), DMA CH0
 */

#ifndef _UART_BT_H_
#define _UART_BT_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Public Functions
 * ======================================================================== */

/**
 * @brief  Initialize Bluetooth UART + DMA reception
 */
void BT_Init(void);

/**
 * @brief  Check if new data has arrived via Bluetooth
 * @return true if unprocessed bytes are available
 */
bool BT_DataAvailable(void);

/**
 * @brief  Read a single byte from Bluetooth buffer
 * @return Received byte (0 if buffer empty)
 */
uint8_t BT_ReadByte(void);

/**
 * @brief  Send raw bytes over Bluetooth
 * @param  data  Data buffer
 * @param  len   Number of bytes to send
 */
void BT_SendBytes(const uint8_t *data, uint16_t len);

/**
 * @brief  Send formatted string over Bluetooth (printf style)
 * @param  fmt  Format string
 * @param  ...  Variable arguments
 */
void BT_Printf(const char *fmt, ...);

/**
 * @brief  Configure DMA for next reception burst
 */
void BT_DMAConfig(void);

#ifdef __cplusplus
}
#endif

#endif /* _UART_BT_H_ */
