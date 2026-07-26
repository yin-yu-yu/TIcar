/**
 * @file    uart_bt.h
 * @brief   蓝牙 UART 接口（UART1、9600bps、DMA 接收）
 *
 * 与 HC-05/HC-06 蓝牙模块通信，用于手机 APP 遥控和参数调整。
 *
 * 硬件：UART1、PB6（TX）、PB7（RX）、DMA CH0
 */

#ifndef _UART_BT_H_
#define _UART_BT_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * 公共函数
 * ======================================================================== */

/**
 * @brief  初始化蓝牙 UART 和 DMA 接收
 */
void BT_Init(void);

/**
 * @brief  检查是否有新的蓝牙数据到达
 * @return 存在未处理字节时返回 true
 */
bool BT_DataAvailable(void);

/**
 * @brief  从蓝牙缓冲区读取一个字节
 * @return 接收到的字节（缓冲区为空时返回 0）
 */
uint8_t BT_ReadByte(void);

/**
 * @brief  通过蓝牙发送原始字节
 * @param  data  数据缓冲区
 * @param  len   待发送字节数
 */
void BT_SendBytes(const uint8_t *data, uint16_t len);

/**
 * @brief  通过蓝牙发送格式化字符串（printf 风格）
 * @param  fmt  格式字符串
 * @param  ...  可变参数
 */
void BT_Printf(const char *fmt, ...);

/**
 * @brief  为下一次接收配置 DMA
 */
void BT_DMAConfig(void);

#ifdef __cplusplus
}
#endif

#endif /* _UART_BT_H_ */
