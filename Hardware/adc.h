/**
 * @file    adc.h
 * @brief   电池电压 ADC 驱动
 *
 * ADC1 CH0（PA15），12 位分辨率，VDDA 参考电压（3.3V）。
 * 分压比为 11:1（最大可测 36.3V，典型 2S 锂电池为约 8.4V）。
 */

#ifndef _ADC_H_
#define _ADC_H_

#include "ti_msp_dl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * 公共函数
 * ======================================================================== */

/**
 * @brief  读取电池电压
 * @return 电池电压（伏特；例如满电 2S 锂电池为 8.4）
 *
 * 计算公式：Vbat = ADC_reading * 3.3V * voltage_divider / 4096
 */
float Get_battery_volt(void);

/** 新接口命名约定的别名 */
#define Batt_GetVoltage()  Get_battery_volt()

#ifdef __cplusplus
}
#endif

#endif /* _ADC_H_ */
