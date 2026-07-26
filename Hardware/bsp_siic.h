/**
 * @file    bsp_siic.h
 * @brief   Software I2C bus abstraction interface
 *
 * Provides a generic I2C interface (pIICInterface_t) that can be
 * backed by either software bit-banged I2C or hardware I2C peripheral.
 *
 * Ported from: WHEELTEC_C07A_BalanceCar/BSP/Inc/bsp_siic.h
 */

#ifndef __BSP_SIIC_H
#define __BSP_SIIC_H

#include "ti_msp_dl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Type Definitions
 * ======================================================================== */

typedef enum {
    IIC_OK       = 0x00U,
    IIC_ERR      = 0x01U,
    IIC_BUSY     = 0x02U,
    IIC_TIMEOUT  = 0x03U,
    I2C_BUS_ERROR,
    I2C_ARBITRATION_LOST,
    I2C_ADDR_NACK
} IIC_Status_t;

typedef struct {
    void (*init)(void);

    /* Raw write: DevAddress + data */
    IIC_Status_t (*write)(uint16_t DevAddress, uint8_t *pData,
                          uint16_t Size, uint32_t Timeout);

    /* Raw read: DevAddress | 0x01 + read data */
    IIC_Status_t (*read)(uint16_t DevAddress, uint8_t *pData,
                         uint16_t Size, uint32_t Timeout);

    /* Register write: DevAddress + RegAddress + data */
    IIC_Status_t (*write_reg)(uint16_t DevAddress, uint16_t MemAddress,
                              uint8_t *pData, uint16_t Size, uint32_t Timeout);

    /* Register read: DevAddress(write) + RegAddress + restart + DevAddress(read) + data */
    IIC_Status_t (*read_reg)(uint16_t DevAddress, uint16_t MemAddress,
                             uint8_t *pData, uint16_t Size, uint32_t Timeout);

    void (*delay_ms)(uint16_t ms);
} IICInterface_t, *pIICInterface_t;

/* ========================================================================
 * Global I2C Device
 * ======================================================================== */
extern IICInterface_t User_sIICDev;

/**
 * @brief  Unlock stuck I2C bus (release SDA if held low by device)
 */
void mpu6050_i2c_sda_unlock(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SIIC_H */
