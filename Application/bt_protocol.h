/**
 * @file    bt_protocol.h
 * @brief   蓝牙通信协议（兼容 WheelTec APP）
 *
 * OWNER:  Team Member
 * STATUS: STUB — to be filled by team member
 *
 * 解析 UART1 DMA 缓冲区中的 APP 命令。
 * 协议采用以 0x7B/0x7D 为帧界的 WheelTec 二进制格式。
 *
 * 支持的命令：
 *   - 方向控制（摇杆、转向）
 *   - 速度调整
 *   - PID 参数设置
 *   - 模式切换
 */

#ifndef _BT_PROTOCOL_H_
#define _BT_PROTOCOL_H_

#include <stdint.h>
#include <stdbool.h>
#include "state_machine.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * 类型定义
 * ======================================================================== */

typedef enum {
    BT_CMD_NONE = 0,
    BT_CMD_MOVE,       /* 摇杆方向（1~8）                */
    BT_CMD_STEER,      /* 左/右转向                      */
    BT_CMD_SPEED_UP,   /* 提高速度                       */
    BT_CMD_SPEED_DOWN, /* 降低速度                       */
    BT_CMD_PID_SET,    /* 设置 PID 参数                  */
    BT_CMD_PID_QUERY,  /* 请求 PID 参数报告              */
    BT_CMD_MODE_SWITCH,/* 切换运行模式                   */
} BT_CmdType_t;

/* ========================================================================
 * 公共函数
 * ======================================================================== */

/**
 * @brief  初始化蓝牙协议解析器
 */
void BT_Protocol_Init(void);

/**
 * @brief  处理蓝牙接收缓冲区
 * @note   应在主循环或定时任务中调用
 *
 * 从 DMA 缓冲区提取完整数据帧并分发命令。
 */
void BT_Protocol_Handler(void);

/**
 * @brief  通过蓝牙发送机器人状态
 * @param  state     当前机器人状态
 * @param  batt_v    电池电压（V）
 * @param  speed_l   左轮速度（mm/s）
 * @param  speed_r   右轮速度（mm/s）
 */
void BT_Protocol_SendStatus(RobotState_t state, float batt_v,
                            float speed_l, float speed_r);

/**
 * @brief  发送 PID 参数报告（供 APP 显示）
 */
void BT_Protocol_SendPID(void);

#ifdef __cplusplus
}
#endif

#endif /* _BT_PROTOCOL_H_ */
