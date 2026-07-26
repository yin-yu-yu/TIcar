/**
 * @file    state_machine.h
 * @brief   机器人状态机：顶层行为控制器
 *
 * OWNER:  Team Lead (队长)
 * STATUS: Active development
 *
 * 管理机器人的运行状态及其转换：
 *
 *   STATE_INIT ──(self-test ok)──▶ STATE_IDLE
 *                                      │
 *                     ┌────────────────┼────────────────┐
 *                     ▼                ▼                 ▼
 *              STATE_RC_DRIVE   STATE_LINE_FOLLOW   STATE_LOW_BATTERY
 *                     │                │                 │
 *                     └────────────────┼─────────────────┘
 *                                      ▼
 *                                STATE_IDLE
 *
 * STATE_ERROR is reachable from any state on critical failure.
 *
 * CALL FLOW: SM_Run() is called at 200Hz from TIMER_0 ISR.
 */

#ifndef _STATE_MACHINE_H_
#define _STATE_MACHINE_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * 类型定义
 * ======================================================================== */

typedef enum {
    STATE_INIT = 0,         /* 上电自检/初始化                         */
    STATE_IDLE,             /* 待机，等待命令                          */
    STATE_RC_DRIVE,         /* 遥控驾驶（蓝牙 APP）                    */
    STATE_LINE_FOLLOW,      /* 自动循迹（红外传感器）                  */
    STATE_LOW_BATTERY,      /* 低电量警告（LED 和 OLED 提示）          */
    STATE_ERROR,            /* 严重故障（停止电机并报警）              */
    STATE_COUNT
} RobotState_t;

/* ========================================================================
 * 公共函数
 * ======================================================================== */

/**
 * @brief  初始化状态机
 * @note   应在硬件初始化完成后、启动时调用一次
 */
void SM_Init(void);

/**
 * @brief  执行一次状态机迭代
 * @note   应在 TIMER_0 中断服务程序中以 CONTROL_FREQ_HZ（200Hz）调用
 *
 * 读取传感器输入，判断状态转换，并调用相应状态处理函数。
 */
void SM_Run(void);

/**
 * @brief  请求状态转换
 * @param  next  目标状态
 * @note   若当前状态不允许转换到该状态，请求将被拒绝
 */
void SM_Transition(RobotState_t next);

/**
 * @brief  获取当前机器人状态
 * @return 当前状态枚举值
 */
RobotState_t SM_GetState(void);

/**
 * @brief  获取可读的状态名称（供 OLED 显示）
 * @param  state  状态值
 * @return 状态名称字符串（如 "IDLE"、"RC_DRIVE"）
 */
const char* SM_GetStateName(RobotState_t state);

/**
 * @brief  检查机器人是否处于可安全移动的状态
 * @return 若允许电机运行则返回 true
 */
bool SM_IsMoving(void);

/**
 * @brief  向状态机报告电池电压
 * @param  voltage  电池电压（V）
 * @note   由主循环调用；状态机据此判断低电量状态
 */
void SM_ReportBattery(float voltage);

#ifdef __cplusplus
}
#endif

#endif /* _STATE_MACHINE_H_ */
