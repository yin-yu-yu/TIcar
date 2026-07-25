/**
 * @file    adc.h
 * @brief   Battery voltage ADC driver
 *
 * ADC1 CH0 (PA15), 12-bit resolution, VDDA reference (3.3V)
 * Voltage divider: 11:1 (measures up to 36.3V, typically 2S LiPo ~8.4V)
 */

#ifndef _ADC_H_
#define _ADC_H_

#include "ti_msp_dl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Public Functions
 * ======================================================================== */

/**
 * @brief  Read battery voltage
 * @return Battery voltage in volts (e.g., 8.4 for fully charged 2S LiPo)
 *
 * Calculation: Vbat = ADC_reading * 3.3V * voltage_divider / 4096
 */
float Get_battery_volt(void);

/** Alias for new API naming convention */
#define Batt_GetVoltage()  Get_battery_volt()

#ifdef __cplusplus
}
#endif

#endif /* _ADC_H_ */
