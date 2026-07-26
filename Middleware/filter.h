/**
 * @file    filter.h
 * @brief   信号处理滤波器
 *
 * 提供互补滤波、低通滤波、移动平均、限幅和速率限制功能。
 */

#ifndef _FILTER_H_
#define _FILTER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * 公共函数
 * ======================================================================== */

/**
 * @brief  一阶低通滤波器
 * @param  input    当前输入采样值
 * @param  prev     上次滤波输出（会原地更新）
 * @param  alpha    平滑系数（0~1，越小越平滑）
 * @return 滤波后的输出
 */
float Filter_LowPass(float input, float *prev, float alpha);

/**
 * @brief  用于角度融合的互补滤波器
 * @param  accel_angle   加速度计角度（有噪声、无漂移）
 * @param  gyro_rate     陀螺仪角速度（deg/s）
 * @param  dt            时间间隔（秒）
 * @param  alpha         置信系数：加速度计权重（0~1，通常为 0.02）
 * @param  fused_angle   上次融合角度（会原地更新）
 * @return 融合后的角度
 */
float Filter_Complementary(float accel_angle, float gyro_rate, float dt,
                           float alpha, float *fused_angle);

/**
 * @brief  速率限制器：限制数值的变化速度
 * @param  target      Desired value
 * @param  current     Current value
 * @param  max_rate    Maximum rate of change per second
 * @param  dt          Time delta (seconds)
 * @return Rate-limited output
 */
float Filter_RateLimit(float target, float current, float max_rate, float dt);

/**
 * @brief  简单死区滤波器
 * @param  value    Input value
 * @param  zone     Dead zone threshold (|value| < zone → 0)
 * @return Filtered value
 */
float Filter_DeadZone(float value, float zone);

#ifdef __cplusplus
}
#endif

#endif /* _FILTER_H_ */
