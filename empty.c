/**
 * @file    empty.c
 * @brief   主程序入口
 *
 * 构建模式（由 CHASSIS_DEBUG 宏控制）：
 *   #define CHASSIS_DEBUG   → 调试模式（各成员添加测试代码）
 *   //#define CHASSIS_DEBUG → 比赛模式（完整状态机）
 *
 * 架构：
 *   main() 循环处理低频任务（蓝牙解析、显示）
 *   TIMER_0 ISR (200Hz) 处理实时控制（编码器、PID、状态机）
 */

#define CHASSIS_DEBUG    /* ⚠️ 调试时取消注释；比赛时注释掉 */

#include "board.h"
#include "show.h"
#include "uart_callback.h"

/* ---- 全局变量 ---- */
u8  Car_Mode = Diff_Car;
int Motor_Left, Motor_Right;
u8  PID_Send;
float RC_Velocity = 200.0f, RC_Turn_Velocity;
float Move_X, Move_Y, Move_Z, PS2_ON_Flag;
float Velocity_Left, Velocity_Right;
u16  test_num, show_cnt;
float Voltage = 8.4f;

/* ========================================================================
 * main()
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

        /* ---- 状态机初始化 ---- */
        SM_Init();
    }
    /* ====================================================================
     * 调试模式 — 各成员在下方添加自己的测试代码
     * ==================================================================== */
#ifdef CHASSIS_DEBUG
    OLED_ShowString(0, 0, "DEBUG MODE");
    OLED_Refresh_Gram();
    delay_ms(3000);
    while (1)
    {
        Voltage = Get_battery_volt();
        BTBufferHandler();

        /* ---- 团队成员调试区 ---- */
        /* [成员A] 电机测试：
         * Motor_SetPWM(2000, 2000); delay_ms(1000); Motor_Stop(); delay_ms(1000); */

        /* [成员B] 编码器测试：
         * int32_t a = Encoder_GetCountA(); Debug_Printf("EncA=%ld\r\n", a); */

        /* [成员C] 红外传感器测试：
         * uint8_t ir = IR_GetSensorState(); OLED_ShowNumber(0,20,ir,1,12); */

        /* [成员D] MPU6050 测试：
         * if (MPU6050_DataReady()) { float yaw = MPU6050_GetYaw();
         *     Debug_Printf("Yaw=%.1f\r\n", yaw); } */

        /* ---- 显示刷新 ---- */
        oled_show();
        OLED_Refresh_Gram();
    }

    /* ====================================================================
     * 比赛模式 — 完整自主/遥控操作，使用状态机
     * ==================================================================== */
#else
    OLED_ShowString(0, 0, "COMP MODE");
    OLED_Refresh_Gram();
    delay_ms(3000);
    while (1)
    {
        Voltage = Get_battery_volt();
        SM_ReportBattery(Voltage);

        BTBufferHandler();       /* 处理接收到的蓝牙数据 */
        APP_Show();              /* 通过蓝牙发送状态到 APP */
        oled_show();             /* 更新 OLED 显示 */
        OLED_Refresh_Gram();

        /* 低电量：LED 闪烁在状态机 (TIMER ISR) 中处理 */
    }
#endif
}
