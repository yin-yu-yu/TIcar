/**
 * @file    pid.h
 * @brief   通用 PID 控制器（增量式 PI，可选微分项）
 *
 * 实现增量式 PID：
 *   output += Kp*(err - prev_err) + Ki*err + Kd*(err - 2*prev_err + prev_prev_err)
 *
 * 功能：
 *   - 输出限幅（out_min ~ out_max）
 *   - 积分抗饱和（达到限幅时停止积分）
 *   - 使用 PID_Reset() 清除全部状态
 */

#ifndef _PID_H_
#define _PID_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * 类型定义
 * ======================================================================== */

typedef struct {
    float Kp;            /* 比例增益                                       */
    float Ki;            /* 积分增益                                       */
    float Kd;            /* 微分增益（0 表示仅使用 PI）                    */
    float setpoint;      /* 目标值                                         */
    float integral;      /* 累积积分误差                                   */
    float prev_error;    /* 上一次迭代的误差                               */
    float prev_prev_error; /* 上上次迭代的误差（供微分项使用）             */
    float out_min;       /* 输出下限                                       */
    float out_max;       /* 输出上限                                       */
    float output;        /* 当前控制器输出                                 */
} PID_t;

/* ========================================================================
 * 公共函数
 * ======================================================================== */

/**
 * @brief  初始化 PID 控制器
 * @param  pid   PID 结构体指针
 * @param  kp    比例增益
 * @param  ki    积分增益
 * @param  kd    微分增益（0 表示仅使用 PI）
 * @param  min   输出最小值
 * @param  max   输出最大值
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd, float min, float max);

/**
 * @brief  设置目标值（设定值）
 * @param  pid       PID 结构体指针
 * @param  setpoint  期望目标值
 */
void PID_SetSetpoint(PID_t *pid, float setpoint);

/**
 * @brief  执行一次 PID 控制计算
 * @param  pid       PID 结构体指针
 * @param  measured  当前测量值（反馈值）
 * @param  dt        距上次调用的时间间隔（秒）
 * @return           限幅后的控制器输出
 *
 * @note   应以固定频率调用（例如 200Hz 对应 dt=0.005）
 */
float PID_Compute(PID_t *pid, float measured, float dt);

/**
 * @brief  重置 PID 状态（清除积分、误差和输出）
 * @param  pid  PID 结构体指针
 */
void PID_Reset(PID_t *pid);

/**
 * @brief  在运行时调整增益
 * @param  pid  PID 结构体指针
 * @param  kp   新的比例增益
 * @param  ki   新的积分增益
 * @param  kd   新的微分增益
 */
void PID_SetGains(PID_t *pid, float kp, float ki, float kd);

#ifdef __cplusplus
}
#endif

#endif /* _PID_H_ */
