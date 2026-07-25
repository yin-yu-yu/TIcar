/**
 * @file    empty.h
 * @brief   Top-level include header — aggregates all subsystem headers
 *
 * Include this file in any module to get access to all hardware,
 * middleware, and application APIs.
 */

#ifndef _MAIN_H_
#define _MAIN_H_

/* ---- BSP (SysConfig generated) ---- */
#include "ti_msp_dl_config.h"

/* ---- System ---- */
#include "clock.h"

/* ---- Hardware Layer ---- */
#include "adc.h"
#include "encoder.h"
#include "ir_track.h"
#include "key.h"
#include "led.h"
#include "motor.h"
#include "mpu6050.h"
#include "oled.h"
#include "uart_debug.h"
#include "uart_bt.h"

/* ---- Middleware Layer ---- */
#include "pid.h"
#include "kinematics.h"
#include "odometry.h"
#include "filter.h"

/* ---- Application Layer ---- */
#include "state_machine.h"
#include "motion_control.h"
#include "line_follow.h"
#include "bt_protocol.h"
#include "debug_scope.h"

/* ---- Config ---- */
#include "robot_config.h"
#include "pid_config.h"

#endif /* #ifndef _MAIN_H_ */
