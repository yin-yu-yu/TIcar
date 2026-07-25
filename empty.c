/**
 * @file    empty.c
 * @brief   Main application entry point
 *
 * Build modes (controlled by CHASSIS_DEBUG define):
 *   #define CHASSIS_DEBUG   → Debug mode (each member adds test code)
 *   //#define CHASSIS_DEBUG → Competition mode (full state machine)
 *
 * Architecture:
 *   main() loop handles low-frequency tasks (BT parse, display)
 *   TIMER_0 ISR (200Hz) handles real-time control (encoder, PID, state machine)
 */

#define CHASSIS_DEBUG    /* ⚠️ Uncomment for debug; comment for competition */

#include "board.h"
#include "show.h"
#include "uart_callback.h"

/* ---- Global variables ---- */
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
    /* ---- System & peripheral init (SysConfig generated) ---- */
    SYSCFG_DL_init();

    /* ---- Clear pending interrupts ---- */
    NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(ADC12_VOLTAGE_INST_INT_IRQN);

    /* ---- Enable interrupts ---- */
    NVIC_EnableIRQ(ENCODERA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(ADC12_VOLTAGE_INST_INT_IRQN);

    /* ---- Hardware init ---- */
    OLED_Init();
    BT_Init();

    /* ---- State machine init ---- */
    SM_Init();

    /* ====================================================================
     * Debug Mode — each team member adds their own test code below
     * ==================================================================== */
#ifdef CHASSIS_DEBUG
    OLED_ShowString(0, 0, "DEBUG MODE");
    OLED_Refresh_Gram();
    delay_ms(3000);
    while (1)
    {
        Voltage = Get_battery_volt();
        BTBufferHandler();

        /* ---- Team member debug sections ---- */
        /* [Member-A] Motor test:
         * Motor_SetPWM(2000, 2000); delay_ms(1000); Motor_Stop(); delay_ms(1000); */

        /* [Member-B] Encoder test:
         * int32_t a = Encoder_GetCountA(); Debug_Printf("EncA=%ld\r\n", a); */

        /* [Member-C] IR sensor test:
         * uint8_t ir = IR_GetSensorState(); OLED_ShowNumber(0,20,ir,1,12); */

        /* [Member-D] MPU6050 test:
         * if (MPU6050_DataReady()) { float yaw = MPU6050_GetYaw();
         *     Debug_Printf("Yaw=%.1f\r\n", yaw); } */

        /* ---- Display refresh ---- */
        oled_show();
        OLED_Refresh_Gram();
    }

    /* ====================================================================
     * Competition Mode — full autonomous/RC operation with state machine
     * ==================================================================== */
#else
    OLED_ShowString(0, 0, "COMP MODE");
    OLED_Refresh_Gram();
    delay_ms(3000);
    while (1)
    {
        Voltage = Get_battery_volt();
        SM_ReportBattery(Voltage);

        BTBufferHandler();       /* Process incoming Bluetooth data    */
        APP_Show();              /* Send status to APP over Bluetooth  */
        oled_show();             /* Update OLED display                */
        OLED_Refresh_Gram();

        /* Low-battery: LED flash handled inside state machine (TIMER ISR) */
    }
#endif
}
