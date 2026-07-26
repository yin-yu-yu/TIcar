/**
 * @file    bsp_siic.h
 * @brief   软件 I2C 总线抽象接口
 *
 * 提供通用 I2C 接口（pIICInterface_t），可由软件模拟 I2C
 * 或硬件 I2C 外设实现。
 *
 * 移植自：WHEELTEC_C07A_BalanceCar/BSP/Inc/bsp_siic.h
 */

#ifndef __BSP_SIIC_H
#define __BSP_SIIC_H

#include "ti_msp_dl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * 类型定义
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

    /* 原始写：设备地址 + 数据 */
    IIC_Status_t (*write)(uint16_t DevAddress, uint8_t *pData,
                          uint16_t Size, uint32_t Timeout);

    /* 原始读：设备地址 | 0x01 + 读取数据 */
    IIC_Status_t (*read)(uint16_t DevAddress, uint8_t *pData,
                         uint16_t Size, uint32_t Timeout);

    /* 寄存器写：设备地址 + 寄存器地址 + 数据 */
    IIC_Status_t (*write_reg)(uint16_t DevAddress, uint16_t MemAddress,
                              uint8_t *pData, uint16_t Size, uint32_t Timeout);

    /* 寄存器读：设备地址（写）+ 寄存器地址 + 重启 + 设备地址（读）+ 数据 */
    IIC_Status_t (*read_reg)(uint16_t DevAddress, uint16_t MemAddress,
                             uint8_t *pData, uint16_t Size, uint32_t Timeout);

    void (*delay_ms)(uint16_t ms);
} IICInterface_t, *pIICInterface_t;

/* ========================================================================
 * 全局 I2C 设备
 * ======================================================================== */
extern IICInterface_t User_sIICDev;

/**
 * @brief  解锁挂起的 I2C 总线（设备拉低 SDA 时释放 SDA）
 */
void mpu6050_i2c_sda_unlock(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SIIC_H */
