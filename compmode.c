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

/* ---- Extern globals used in competition loop ---- */
extern float Voltage;
extern float Velocity_Left, Velocity_Right;
extern u8    PID_Send;

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
    OLED_ShowString(0, 0, "COMP MODE");
    OLED_Refresh_Gram();
    delay_ms(3000);

    /* ====================================================================
     * Competition main loop
     * ==================================================================== */
    while (1)
    {
        static uint8_t bt_toggle = 0;

        /* ---- Battery monitoring ---- */
        Voltage = Get_battery_volt();
        SM_ReportBattery(Voltage);

        /* ---- Bluetooth protocol (new API — replaces legacy BTBufferHandler) ---- */
        BT_Protocol_Handler();

        /* ---- Status reporting over Bluetooth (new API — replaces legacy APP_Show) ----
         * Alternates between: A-packet (status), B-packet (attitude), C-packet (PID) */
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

        /* ---- OLED display refresh ---- */
        CompMode_OLED_Show();
        OLED_Refresh_Gram();

        /* ---- Low battery: LED flashing handled in state machine (TIMER ISR) ---- */
    }
}
