/**
 * @file    mpu6050.h
 * @brief   MPU6050 6-axis IMU driver interface
 *
 * Hardware: MPU6050 (3-axis gyro + 3-axis accelerometer)
 * Interface: I2C (software or hardware, to be configured)
 *
 * STATUS: STUB — Team member to port from reference project
 *         See README.md for AI agent prompt
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
 * @brief  Initialize MPU6050 and DMP (Digital Motion Processor)
 * @note   Configures I2C, initializes DMP for angle output
 */
void MPU6050_Init(void);

/**
 * @brief  Check if new MPU6050 data is available
 * @return true if DMP data ready, false otherwise
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
