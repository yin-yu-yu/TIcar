/**
 * @file    board.h
 * @brief   Master board header — aggregates all subsystem headers
 *
 * Include this single header in any module to access all APIs.
 */

#ifndef _BOARD_H_
#define _BOARD_H_

/* ---- Standard libraries ---- */
#include <stdio.h>
#include <string.h>

/* ---- BSP (SysConfig generated) ---- */
#include "ti_msp_dl_config.h"

/* ---- Config ---- */
#include "robot_config.h"
#include "pid_config.h"

/* ---- Hardware Layer ---- */
#include "oled.h"
#include "led.h"
#include "key.h"
#include "motor.h"
#include "encoder.h"
#include "adc.h"
#include "ir_track.h"
#include "uart_debug.h"
#include "uart_bt.h"


/* ---- Middleware ---- */
#include "pid.h"
#include "kinematics.h"
#include "odometry.h"
#include "filter.h"

/* ---- Application ---- */
#include "state_machine.h"
#include "motion_control.h"
#include "line_follow.h"
#include "bt_protocol.h"
#include "debug_scope.h"

/* ---- System (placeholder for MPU6050 clock) ---- */
#include "clock.h"
#include "mpu6050.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Standard Type Aliases
 * ======================================================================== */
#define ABS(a)  ((a) > 0 ? (a) : (-(a)))

typedef int32_t   s32;
typedef int16_t   s16;
typedef int8_t    s8;

typedef const int32_t  sc32;
typedef const int16_t  sc16;
typedef const int8_t   sc8;

typedef __IO int32_t   vs32;
typedef __IO int16_t   vs16;
typedef __IO int8_t    vs8;

typedef __I int32_t   vsc32;
typedef __I int16_t   vsc16;
typedef __I int8_t    vsc8;

typedef uint32_t  u32;
typedef uint16_t  u16;
typedef uint8_t   u8;

typedef const uint32_t  uc32;
typedef const uint16_t  uc16;
typedef const uint8_t   uc8;

typedef __IO uint32_t  vu32;
typedef __IO uint16_t  vu16;
typedef __IO uint8_t   vu8;

typedef __I uint32_t  vuc32;
typedef __I uint16_t  vuc16;
typedef __I uint8_t   vuc8;

/* ========================================================================
 * Car Mode Enumeration
 * ======================================================================== */
typedef enum {
    Mec_Car       = 0,
    Omni_Car      = 1,
    Akm_Car       = 2,
    Diff_Car      = 3,
    FourWheel_Car = 4,
    Tank_Car      = 5
} CarMode;

/* ========================================================================
 * SysTick / Timing
 * ======================================================================== */
#define SysTickMAX_COUNT    0xFFFFFF
#define SysTickFre          80000000
#define SysTick_MS(x)       ((SysTickFre / 1000U) * (uint32_t)(x))
#define SysTick_US(x)       ((SysTickFre / 1000000U) * (uint32_t)(x))

uint32_t Systick_getTick(void);
void     delay_ms(uint32_t ms);
void     delay_us(uint32_t us);
void     delay_1us(unsigned long __us);
void     delay_1ms(unsigned long ms);

/* ========================================================================
 * Global Variables — Active Use
 * ======================================================================== */
extern u8   Car_Mode;
extern u8   Flag_Stop, Flag_Show;
extern u8   PID_Send;
extern int  Motor_Left, Motor_Right;
extern int  Run_Mode;

extern float Voltage;
extern float RC_Velocity, RC_Turn_Velocity;
extern float Move_X, Move_Y, Move_Z, PS2_ON_Flag;
extern float Velocity_Left, Velocity_Right;
extern u16   test_num, show_cnt;

extern float Velocity_KP, Velocity_KI;
extern float BaseSpeed;
extern float Turn90Angle, TurnMaxAngle, TurnMidAngle, TurnMinAngle, ForwardLimit;

/* ---- Encoder counts (from encoder ISR) ---- */
extern int Get_Encoder_countA, Get_Encoder_countB;

/* ---- BT control flags (from uart_callback.c) ---- */
extern int Flag_Left, Flag_Right, Flag_Direction, Turn_Flag;

/* ---- Mode switch (debug ↔ competition, long-press key) ---- */
extern volatile bool g_DebugMode;
extern volatile bool g_ModeSwitchRequest;

/* ========================================================================
 * Global Variables — LEGACY (balance car remnants, to be removed)
 *   These are kept for compilation compatibility with existing modules.
 *   Team members: do NOT add new references to these.
 *   They will be removed after all modules are migrated to new API.
 * ======================================================================== */
/* -- @deprecated Balance control (unused, MPU6050 pending) -- */
extern u8   Way_Angle;
extern int  Middle_angle;
extern float Angle_Balance, Gyro_Balance, Gyro_Turn;
extern float Balance_Kp, Balance_Kd, Turn_Kp, Turn_Kd;
extern float Acceleration_Z;

/* -- @deprecated Radar/avoid/follow (unused) -- */
extern u8   Flag_follow, Flag_avoid, delay_50, delay_flag;
extern u8   LD_Successful_Receive_flag;
extern int  Temperature;
extern u32  Distance;

/* -- @deprecated Lap/track (unused) -- */
extern u8   one_frame_data_success_flag, one_lap_data_success_flag;
extern int  lap_count, PointDataProcess_count, test_once_flag, Dividing_point;
extern u16  determine;

/* -- @deprecated Misc -- */
extern u8   recv0_flag;
extern volatile unsigned long tick_ms;

#ifdef __cplusplus
}
#endif

#endif /* _BOARD_H_ */
