/**
 * @file    mpu6050.h
 * @brief   MPU6050 六轴 IMU 驱动接口
 *
 * 硬件：MPU6050（三轴陀螺仪 + 三轴加速度计）
 * 接口：通过 bsp_siic.h 使用软件 I2C（PA0=SDA，PA1=SCL）
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
 * 公共函数
 * ======================================================================== */

/**
 * @brief  通过软件 I2C 初始化 MPU6050
 * @note   Verifies WHO_AM_I, configures gyro ±2000dps, accel ±2g,
 *         DLPF ~44Hz, sample rate 200Hz. Idempotent via g_initialized flag.
 */
void MPU6050_Init(void);

/**
 * @brief  检查是否已读取到新的传感器数据
 * @return MPU6050_Read() 成功调用后返回 true
 */
bool MPU6050_DataReady(void);

/**
 * @brief  读取陀螺仪、加速度计和姿态角
 * @param  gyro   [out] 三轴陀螺仪数据（dps）：[gx, gy, gz]
 * @param  accel  [out] 三轴加速度计数据（g）：[ax, ay, az]
 * @param  angle  [out] 三轴欧拉角（度）：[roll, pitch, yaw]
 */
void MPU6050_Read(float gyro[3], float accel[3], float angle[3]);

/**
 * @brief  获取偏航角（绕 Z 轴旋转）
 * @return 偏航角（度）
 */
float MPU6050_GetYaw(void);

/**
 * @brief  获取俯仰角
 * @return 俯仰角（度）
 */
float MPU6050_GetPitch(void);

/**
 * @brief  获取横滚角
 * @return 横滚角（度）
 */
float MPU6050_GetRoll(void);

/**
 * @brief  获取 Z 轴角速度（陀螺仪）
 * @return 角速度（度/秒）
 */
float MPU6050_GetGyroZ(void);

#ifdef __cplusplus
}
#endif

#endif /* _MPU6050_H_ */
