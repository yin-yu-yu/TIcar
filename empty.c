/**
 * @file    empty.c
 * @brief   主程序入口 — 系统初始化和模式分发
 *
 * ⚠️ 此文件为稳定入口，请勿频繁修改。
 *    队员调试代码 → debugmode.c
 *    比赛模式代码 → compmode.c
 *
 * 构建模式（由 CHASSIS_DEBUG 宏控制）：
 *   #define CHASSIS_DEBUG   → 调试模式（运行接口测试 + 队员调试区）
 *   //#define CHASSIS_DEBUG → 比赛模式（完整状态机自主/遥控操作）
 *
 * 架构：
 *   main() 初始化硬件 → 分发到 DebugMode_Run() 或 CompMode_Run()
 *   TIMER_0 ISR (200Hz, Control/control.c) 处理实时控制（编码器、PID、状态机）
 */

#define CHASSIS_DEBUG    /* ⚠️ 调试时取消注释；比赛时注释掉 */

#include "board.h"
#include "show.h"
#include "uart_callback.h"
#include "debugmode.h"
#include "compmode.h"

/* ---- 全局变量（两个模式共享）---- */
u8  Car_Mode = Diff_Car;
int Motor_Left, Motor_Right;
u8  PID_Send;
float RC_Velocity = 200.0f, RC_Turn_Velocity;
float Move_X, Move_Y, Move_Z, PS2_ON_Flag;
float Velocity_Left, Velocity_Right;
u16  test_num, show_cnt;
float Voltage = 8.4f;

/* ========================================================================
 * main() — 初始化后分发给当前模式
 * ======================================================================== */
int main(void)
{
    {
        /* ---- 系统和外设初始化（SysConfig 生成）---- */
        SYSCFG_DL_init();

        /* ---- 清除挂起的中断 ---- */
        NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
        NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
        NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
        NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
        NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
        NVIC_ClearPendingIRQ(ADC12_VOLTAGE_INST_INT_IRQN);

        /* ---- 使能中断 ---- */
        NVIC_EnableIRQ(ENCODERA_INT_IRQN);
        NVIC_EnableIRQ(ENCODERB_INT_IRQN);
        NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
        NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
        NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
        NVIC_EnableIRQ(ADC12_VOLTAGE_INST_INT_IRQN);

        /* ---- 硬件初始化 ---- */
        OLED_Init();
        BT_Init();

        /* ---- 状态机初始化 (含 MotionControl_Init, LineFollow_Init, BT_Protocol_Init, MPU6050_Init) ---- */
        SM_Init();
    }

    /* ---- 模式分发（两个模式均在独立文件中，不会返回）---- */
#ifdef CHASSIS_DEBUG
    DebugMode_Run();   /* debugmode.c — 13 项接口测试 + 队员调试循环 */
#else
    CompMode_Run();    /* compmode.c  — 比赛/遥控主循环 */
#endif
}
