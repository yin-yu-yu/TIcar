/**
 * @file    robot_config.h
 * @brief   Robot mechanical parameters and physical constants
 *
 * ALL tunable robot parameters are centralized here.
 * No magic numbers scattered in source files.
 *
 * Usage: #include "robot_config.h"
 */

#ifndef _ROBOT_CONFIG_H_
#define _ROBOT_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * 1. Chassis Type
 * ======================================================================== */
typedef enum {
    CHASSIS_MECANUM      = 0,
    CHASSIS_OMNI         = 1,
    CHASSIS_ACKERMANN    = 2,
    CHASSIS_DIFFERENTIAL = 3,
    CHASSIS_4WD          = 4,
    CHASSIS_TANK         = 5,
} ChassisType_t;

#define CHASSIS_TYPE    CHASSIS_DIFFERENTIAL

/* ========================================================================
 * 2. Wheel & Mechanical Parameters
 * ======================================================================== */
#define WHEEL_DIAMETER_MM       65.0f       /* Wheel diameter (mm)              */
#define WHEEL_PERIMETER_M       0.2042f     /* Wheel perimeter (m)              */
#define WHEEL_SPACING_M         0.1610f     /* Distance between two wheels (m)  */
#define MOTOR_GEAR_RATIO        28.0f       /* Motor gearbox reduction ratio    */
#define ENCODER_PPR             13.0f       /* Encoder pulses per revolution    */
#define ENCODER_MULTIPLES       2           /* Encoder quadrature multiplier    */

/* ========================================================================
 * 3. Motion Limits
 * ======================================================================== */
#define MAX_LINEAR_SPEED_MPS    0.5f        /* Max forward/backward speed (m/s) */
#define MAX_ANGULAR_SPEED_RPS   1.5f        /* Max angular speed (rad/s)        */
#define TURN_RADIUS_MIN_M       0.3f        /* Min turning radius (caster limitation) */

/* ========================================================================
 * 4. PWM Parameters
 * ======================================================================== */
#define PWM_PERIOD              8000        /* Timer period for PWM             */
#define PWM_MAX                 8000        /* Max PWM duty value               */
#define PWM_DEAD_ZONE           0           /* Motor dead zone (0 = none)       */

/* ========================================================================
 * 5. Control Loop
 * ======================================================================== */
#define CONTROL_FREQ_HZ         200         /* Main control loop frequency      */
#define CONTROL_PERIOD_MS       5           /* 1000 / CONTROL_FREQ_HZ           */
#define CONTROL_DT_S            0.005f      /* Control period in seconds        */

/* ========================================================================
 * 6. Battery Monitoring
 * ======================================================================== */
#define BATT_VOLTAGE_DIVIDER     11.0f      /* ADC voltage divider ratio        */
#define BATT_ADC_REF_V           3.3f       /* ADC reference voltage            */
#define BATT_ADC_RESOLUTION      4096.0f    /* 12-bit ADC                       */
#define BATT_WARN_THRESHOLD_V    7.0f       /* Low battery warning (V)          */
#define BATT_CRITICAL_THRESHOLD_V 6.5f      /* Critical low battery (V)         */
#define BATT_FULL_V              8.4f       /* Full battery voltage (2S LiPo)   */

/* ========================================================================
 * 7. IR Tracking Sensor
 * ======================================================================== */
#define IR_SENSOR_COUNT          4          /* Number of IR tracking sensors    */

#ifdef __cplusplus
}
#endif

#endif /* _ROBOT_CONFIG_H_ */
