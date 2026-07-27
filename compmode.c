/**
 * @file    compmode.c
 * @brief   Competition mode — autonomous / remote-control main loop
 *
 * OWNER:   Team Lead (队长)
 * STATUS:  Active development
 *
 * This is the robot's normal operating mode. The while(1) loop handles
 * low-frequency tasks: Bluetooth protocol, status reporting, and OLED display.
 * Real-time control (PID, state machine) runs at 200Hz in TIMER_0 ISR.
 *
 * ## State machine state → OLED label mapping
 *
 *   INIT   → 自检中       IDLE     → 待机
 *   RC_DRIVE → 遥控        LINE_FOLLOW → 巡线
 *   LOW_BAT → 低电量       ERROR    → 故障
 */

#include "compmode.h"
#include "board.h"
#include "show.h"
#include "uart_callback.h"
#include "IR_Module.h"

/* 临时调试开关：1=蓝牙只发送循迹模式，0=恢复A/B/C状态帧。 */
#define BT_ONLY_IR_MODE 1

/* ---- Extern globals used in competition loop ---- */
extern float Voltage;
extern float Velocity_Left, Velocity_Right;
extern u8    PID_Send;
extern volatile bool g_DebugMode;
extern volatile bool g_ModeSwitchRequest;

/* ========================================================================
 * OLED status display (competition mode)
 * ======================================================================== */

/**
 * @brief  Render robot status to OLED in competition layout.
 *
 * Row 0: Car mode + run mode + state
 * Row 1: IR sensor bits
 * Row 2-3: Left/right target vs actual speed
 * Row 4: Battery voltage + stop flag
 */
static void CompMode_OLED_Show(void)
{
    oled_show();
}

/* ========================================================================
 * Public function
 * ======================================================================== */

void CompMode_Run(void)
{
    static bool first_run = true;

    /* ---- Only show boot banner on first entry after reset ---- */
    if (first_run) {
        OLED_ShowString(0, 0, "COMP MODE");
        OLED_Refresh_Gram();
        delay_ms(3000);
        first_run = false;
    }

    /* ====================================================================
     * Competition main loop
     * ==================================================================== */
    while (1)
    {
        /* ---- Mode switch: long-press key → debug mode ---- */
        if (g_ModeSwitchRequest) {
            g_ModeSwitchRequest = false;
            g_DebugMode = true;
            Motor_Stop();
            return;
        }

        static uint8_t last_ir_mode = 0xFFU;

        /* ---- Battery monitoring ---- */
        Voltage = Get_battery_volt();
        SM_ReportBattery(Voltage);

        /* ---- Bluetooth protocol (new API — replaces legacy BTBufferHandler) ---- */
        BT_Protocol_Handler();

        /* ---- 原有A/B/C状态帧；循迹调试期间暂时屏蔽 ---- */
#if !BT_ONLY_IR_MODE
        static uint8_t bt_toggle = 0;
        if (PID_Send) {
            BT_Protocol_SendPID();
            PID_Send = 0;
        } else if (bt_toggle == 0) {
            /* A-packet: motor speeds + battery */
            BT_Protocol_SendStatus(SM_GetState(), Voltage,
                                   Velocity_Left, Velocity_Right);
        } else {
            /* B-packet: attitude angles (real data after MPU6050 integration) */
            BT_Printf("{B%d:%d:%d}$", 0, 0, 0);
        }
        bt_toggle = !bt_toggle;
#endif

        /* 循迹模式变化时发送一次，避免在400Hz控制中断里阻塞串口。
         * 帧格式示例：{IR:9:STRAIGHT}$\r\n、{IR:F:LOST}$\r\n */
        uint8_t ir_mode = IR_GetCurrentMode();
        if (ir_mode != last_ir_mode) {
            BT_Printf("{IR:%X:%s}$\r\n", ir_mode, IR_GetModeName(ir_mode));
            last_ir_mode = ir_mode;
        }

        /* ---- OLED display refresh（oled_show内部已完成一次刷新） ---- */
        CompMode_OLED_Show();

        /* 低速任务限频到20Hz，避免ADC/UART/OLED持续占满CPU。 */
        delay_ms(50);

        /* ---- Low battery: LED flashing handled in state machine (TIMER ISR) ---- */
    }
}
