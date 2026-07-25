/**
 * @file    clock.h
 * @brief   System clock placeholder (SysConfig handles clock init)
 *
 * This file exists as a placeholder for the original WheelTec clock.h
 * dependency. All clock configuration is now handled by SysConfig
 * (SYSCFG_DL_SYSCTL_init in ti_msp_dl_config.c).
 *
 * If MPU6050 DMP driver requires specific clock functions, add them here
 * during MPU6050 porting.
 */

#ifndef _CLOCK_H_
#define _CLOCK_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Reserved for MPU6050 DMP timestamp functions if needed */

#ifdef __cplusplus
}
#endif

#endif /* _CLOCK_H_ */
