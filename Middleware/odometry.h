/**
 * @file    odometry.h
 * @brief   车轮里程计：编码器脉冲 → 位置与速度
 *
 * 使用差速驱动的航位推算模型，将编码器计数换算为实际位置
 * （x、y、theta）。
 */

#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * 类型定义
 * ======================================================================== */

typedef struct {
    float x;       /* 全局 X 坐标（m）                     */
    float y;       /* 全局 Y 坐标（m）                     */
    float theta;   /* 航向角（rad），范围为 -PI ~ +PI       */
    float vx;      /* 当前前向速度（m/s）                  */
    float vz;      /* 当前角速度（rad/s）                  */
} Odom_t;

/* ========================================================================
 * 公共函数
 * ======================================================================== */

/**
 * @brief  初始化里程计（将位姿重置到原点）
 */
void Odom_Init(void);

/**
 * @brief  根据编码器计数更新里程计
 * @param  encL     左编码器脉冲数（距上次调用）
 * @param  encR     右编码器脉冲数（距上次调用）
 * @param  dt       时间间隔（秒）
 *
 * @note   应在定时器中断服务程序中以 CONTROL_FREQ_HZ（200Hz）调用
 * @note   每次读取后，编码器中断服务程序会将 encL/encR 复位为零
 */
void Odom_Update(int32_t encL, int32_t encR, float dt);

/**
 * @brief  获取当前位姿和速度
 * @return 里程计状态副本
 */
Odom_t Odom_GetPose(void);

/**
 * @brief  将里程计重置为原点（0、0、0）
 */
void Odom_Reset(void);

/**
 * @brief  获取累计行驶距离（m）
 * @return 累计行驶距离
 */
float Odom_GetDistance(void);

#ifdef __cplusplus
}
#endif

#endif /* _ODOMETRY_H_ */
