/**
 * @file    empty.c
 * @brief   主程序入口 — 系统初始化和模式分发
 *
 * ⚠️ 此文件为稳定入口，请勿频繁修改。
 *    队员调试代码 → debugmode.c
 *    比赛模式代码 → compmode.c
 *
 * 模式切换：
 *   编译默认：CHASSIS_DEBUG 宏决定初始模式
 *   运行切换：长按按键（>500ms）在调试/比赛模式间切换
 *
 * 架构：
 *   main() 初始化硬件 → 循环分发到 DebugMode_Run() / CompMode_Run()
 *   TIMER_0 ISR (200Hz, Control/control.c) 处理实时控制 + 按键扫描
 */

/* 默认进入比赛/巡线状态机。
 * 如需运行会主动驱动电机的 13 项接口自检，再取消下面一行注释。 */
/* #define CHASSIS_DEBUG */

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

/* ---- 模式切换（ISR 写 g_ModeSwitchRequest，主循环响应）---- */
volatile bool g_DebugMode =
#ifdef CHASSIS_DEBUG
    true;
#else
    false;
#endif
volatile bool g_ModeSwitchRequest = false;

/* ========================================================================
 * main() — 初始化后循环分发到当前模式
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

    /* ---- 模式分发循环（长按按键切换）---- */
    while (1)
    {
        if (g_DebugMode)
            DebugMode_Run();   /* debugmode.c — 返回时切换到比赛模式 */
        else
            CompMode_Run();    /* compmode.c  — 返回时切换到调试模式 */
    }
}
