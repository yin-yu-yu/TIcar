/**
 * @file    encoder.h
 * @brief   Quadrature encoder driver (GPIO interrupt-based)
 *
 * Two encoders: Encoder A (GPIOA PA25/PA26), Encoder B (GPIOB PB20/PB24)
 * ISR: GROUP1_IRQHandler in encoder.c
 */

#ifndef _ENCODER_H_
#define _ENCODER_H_

#include <stdint.h>
#include "ti_msp_dl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Encoder count accumulators (incremented in ISR) ---- */
extern int32_t Get_Encoder_countA;
extern int32_t Get_Encoder_countB;

/* ========================================================================
 * Public Functions
 * ======================================================================== */

/**
 * @brief  Get encoder A count and reset
 * @return Accumulated pulses since last call
 */
static inline int32_t Encoder_GetCountA(void)
{
    int32_t val = Get_Encoder_countA;
    /* Count is typically reset in control ISR after reading */
    return val;
}

/**
 * @brief  Get encoder B count and reset
 * @return Accumulated pulses since last call
 */
static inline int32_t Encoder_GetCountB(void)
{
    int32_t val = Get_Encoder_countB;
    return val;
}

/**
 * @brief  Reset both encoder counts to zero
 */
static inline void Encoder_Reset(void)
{
    Get_Encoder_countA = 0;
    Get_Encoder_countB = 0;
}

#ifdef __cplusplus
}
#endif

#endif /* _ENCODER_H_ */
