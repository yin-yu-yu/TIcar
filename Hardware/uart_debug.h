/**
 * @file    uart_debug.h
 * @brief   调试 UART 接口（UART0、115200bps）
 *
 * 用于调试 printf、DataScope 虚拟示波器和开发期诊断。
 *
 * 硬件：UART0、PA10（TX）、PA11（RX）
 */

#ifndef _UART_DEBUG_H_
#define _UART_DEBUG_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * 公共函数
 * ======================================================================== */

/**
 * @brief  发送格式化调试字符串（printf 风格）
 * @param  fmt  格式字符串
 * @param  ...  可变参数
 */
void Debug_Printf(const char *fmt, ...);

/**
 * @brief  通过调试 UART 发送原始二进制数据
 * @param  data  数据缓冲区
 * @param  len   待发送字节数
 */
void Debug_SendBinary(const uint8_t *data, uint16_t len);

/**
 * @brief  发送单个字符
 * @param  ch  待发送字符
 */
void Debug_PutChar(char ch);

#ifdef __cplusplus
}
#endif

#endif /* _UART_DEBUG_H_ */
