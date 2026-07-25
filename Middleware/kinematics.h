/**
 * @file    kinematics.h
 * @brief   Differential drive kinematics (forward & inverse)
 *
 * Inverse kinematics:  ChassisCmd → WheelSpeed
 * Forward kinematics:  WheelSpeed  → ChassisCmd (for odometry)
 *
 * Model: Two-wheel differential drive with caster front wheel.
 *        CANNOT turn in place — must maintain vx != 0 for rotation.
 */

#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Type Definitions
 * ======================================================================== */

/** Chassis-level motion command (body frame) */
typedef struct {
    float vx;    /* Forward velocity (m/s), positive = forward       */
    float wz;    /* Angular velocity (rad/s), positive = CCW (left)  */
} ChassisCmd_t;

/** Per-wheel speed outputs */
typedef struct {
    float left;   /* Left wheel speed (m/s)   */
    float right;  /* Right wheel speed (m/s)  */
} WheelSpeed_t;

/* ========================================================================
 * Public Functions
 * ======================================================================== */

/**
 * @brief  Initialize kinematics with robot geometry
 * @note   Reads parameters from robot_config.h
 */
void Kinematics_Init(void);

/**
 * @brief  Inverse kinematics: chassis command → wheel speeds
 * @param  cmd  Desired chassis motion (vx, wz)
 * @return      Required wheel speeds (m/s)
 *
 * Formula:
 *   V_left  = vx - wz * WheelSpacing/2
 *   V_right = vx + wz * WheelSpacing/2
 *
 * Constraints:
 *   - Minimum turn radius enforced (TURN_RADIUS_MIN_M)
 *   - Output speeds clamped to MAX_LINEAR_SPEED_MPS
 */
WheelSpeed_t Kinematics_Inverse(ChassisCmd_t cmd);

/**
 * @brief  Forward kinematics: wheel speeds → chassis motion
 * @param  wheels  Measured wheel speeds (m/s)
 * @return         Estimated chassis motion
 *
 * Formula:
 *   vx = (V_left + V_right) / 2
 *   wz = (V_right - V_left) / WheelSpacing
 */
ChassisCmd_t Kinematics_Forward(WheelSpeed_t wheels);

/**
 * @brief  Compute minimum turn radius at given speed
 * @param  vx  Forward speed (m/s)
 * @return     Minimum achievable turn radius (m)
 *
 * For differential drive with caster:
 *   R = max(vx/wz_max, TURN_RADIUS_MIN_M)
 */
float Kinematics_MinTurnRadius(float vx);

#ifdef __cplusplus
}
#endif

#endif /* _KINEMATICS_H_ */
