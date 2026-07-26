/**
 * @file    mpu6050.h
 * @brief   MPU6050 6-axis IMU driver interface
 *
 * Hardware: MPU6050 (3-axis gyro + 3-axis accelerometer)
 * Interface: Software I2C (PA0=SDA, PA1=SCL) via bsp_siic.h
 *
 * STATUS: COMPLETE — Ported from WHEELTEC_C07A_BalanceCar
 *         Uses Mahony AHRS (quaternion + PI correction) for
 *         attitude estimation without DMP firmware dependency.
 *
 * Accel output: g (1/16384 LSB/g for ±2g range)
 * Gyro output:  deg/s (1/16.4 LSB/dps for ±2000dps range)
 * Angle output: deg (roll/pitch/yaw from Mahony AHRS)
 */

#ifndef _MPU6050_H_
#define _MPU6050_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Public Functions
 * ======================================================================== */

/**
 * @brief  Initialize MPU6050 via software I2C
 * @note   Verifies WHO_AM_I, configures gyro ±2000dps, accel ±2g,
 *         DLPF ~44Hz, sample rate 200Hz. Idempotent via g_initialized flag.
 */
void MPU6050_Init(void);

/**
 * @brief  Check if new sensor data has been read
 * @return true after a successful MPU6050_Read() call
 */
bool MPU6050_DataReady(void);

/**
 * @brief  Read gyroscope, accelerometer, and attitude angles
 * @param  gyro   [out] 3-axis gyroscope data (dps): [gx, gy, gz]
 * @param  accel  [out] 3-axis accelerometer data (g): [ax, ay, az]
 * @param  angle  [out] 3-axis Euler angles (degrees): [roll, pitch, yaw]
 */
void MPU6050_Read(float gyro[3], float accel[3], float angle[3]);

/**
 * @brief  Get Yaw (Z-axis rotation) angle
 * @return Yaw angle in degrees
 */
float MPU6050_GetYaw(void);

/**
 * @brief  Get Pitch angle
 * @return Pitch angle in degrees
 */
float MPU6050_GetPitch(void);

/**
 * @brief  Get Roll angle
 * @return Roll angle in degrees
 */
float MPU6050_GetRoll(void);

/**
 * @brief  Get Z-axis angular velocity (gyro)
 * @return Angular velocity in degrees per second
 */
float MPU6050_GetGyroZ(void);

#ifdef __cplusplus
}
#endif

#endif /* _MPU6050_H_ */
