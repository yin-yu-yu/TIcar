/**
 * @file    debug_scope.h
 * @brief   虚拟示波器 / DataScope 调试接口
 *
 * 通过 UART 向上位机可视化软件发送实时变量数据，
 * 用于调试。
 *
 * 使用 DataScope_DP 协议（Control/DataScope_DP.C）。
 */

#ifndef _DEBUG_SCOPE_H_
#define _DEBUG_SCOPE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * 公共函数
 * ======================================================================== */

/**
 * @brief  向虚拟示波器发送一个通道的数据
 * @param  data     待发送的浮点数值
 * @param  channel  通道索引（1~10）
 */
void Scope_SendChannel(float data, uint8_t channel);

/**
 * @brief  将已缓冲的全部通道作为完整数据帧发送
 * @param  num_channels  本帧中的通道数量
 */
void Scope_SendFrame(uint8_t num_channels);

#ifdef __cplusplus
}
#endif

#endif /* _DEBUG_SCOPE_H_ */
