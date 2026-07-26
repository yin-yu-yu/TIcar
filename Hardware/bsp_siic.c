/**
 * @file    bsp_siic.c
 * @brief   Software I2C bus driver (bit-banged GPIO)
 *
 * Provides IICInterface_t User_sIICDev for MPU6050 communication.
 * Uses software bit-banging — no hardware I2C peripheral needed.
 *
 * Ported from: WHEELTEC_C07A_BalanceCar/BSP/bsp_siic.c
 * Adapted for TIcar: uses board.h for delays, GPIO PA0(SDA)/PA1(SCL)
 *
 * I2C pins (configurable here — also add to empty.syscfg if desired):
 *   - SDA: PA0 (IOMUX_PINCM1)
 *   - SCL: PA1 (IOMUX_PINCM2)
 *
 * All I2C transactions guarantee: STOP on every exit (success or failure),
 * SDA/SCL restored to idle (both HIGH, both OUTPUT).
 *
 * Estimated timing at 80MHz Cortex-M0+:
 *   14-byte register read (MPU6050 full sensor burst):
 *     Start + addr + reg + restart + read 14 + Stop ≈ 350–500 µs
 */

#include "bsp_siic.h"
#include "board.h"

/* ========================================================================
 * Pin Configuration — change these if using different GPIOs
 * ======================================================================== */
#define SIIC_PORT       GPIOA
#define SIIC_SDA_PIN    DL_GPIO_PIN_0
#define SIIC_SCL_PIN    DL_GPIO_PIN_1
#define SIIC_SDA_IOMUX  IOMUX_PINCM1   /* PA0 IOMUX index */
#define SIIC_SCL_IOMUX  IOMUX_PINCM2   /* PA1 IOMUX index */

/* Delay tuning: increase if communication is unreliable at high CPU freq */
#define USERCONFIG_DELAYUS     1

/* ---- GPIO shorthand macros ---- */
#define SDA_H   DL_GPIO_setPins(SIIC_PORT, SIIC_SDA_PIN)
#define SDA_L   DL_GPIO_clearPins(SIIC_PORT, SIIC_SDA_PIN)
#define SCL_H   DL_GPIO_setPins(SIIC_PORT, SIIC_SCL_PIN)
#define SCL_L   DL_GPIO_clearPins(SIIC_PORT, SIIC_SCL_PIN)

/* ========================================================================
 * Static Helpers
 * ======================================================================== */

static void SDA_IN(void)
{
    DL_GPIO_initDigitalInput(SIIC_SDA_IOMUX);
}

static void SDA_OUT(void)
{
    DL_GPIO_initDigitalOutput(SIIC_SDA_IOMUX);
    DL_GPIO_setPins(SIIC_PORT, SIIC_SDA_PIN);
    DL_GPIO_enableOutput(SIIC_PORT, SIIC_SDA_PIN);
}

static void SCL_OUT(void)
{
    DL_GPIO_initDigitalOutput(SIIC_SCL_IOMUX);
    DL_GPIO_setPins(SIIC_PORT, SIIC_SCL_PIN);
    DL_GPIO_enableOutput(SIIC_PORT, SIIC_SCL_PIN);
}

static uint8_t READ_SDA(void)
{
    return (DL_GPIO_readPins(SIIC_PORT, SIIC_SDA_PIN) & SIIC_SDA_PIN) ? 1 : 0;
}

/**
 * @brief  Initialize both I2C pins explicitly as outputs, idle HIGH
 */
static void siic_init(void)
{
    /* Explicitly configure SDA (PA0) as digital output */
    SDA_OUT();

    /* Explicitly configure SCL (PA1) as digital output */
    SCL_OUT();

    /* Bus idle: both lines HIGH, both outputs */
    SCL_H;
    SDA_H;
}

/**
 * @brief  Restore bus to idle state after an error.
 *         Issues STOP then re-init both pins to output-high.
 */
static void sIIC_Stop(void);  /* fwd decl */

static void sIIC_RecoverBus(void)
{
    sIIC_Stop();
    /* Ensure both pins are output-driving HIGH */
    SDA_OUT();
    SCL_OUT();
    SDA_H;
    SCL_H;
}

/* ---- I2C bus primitives ---- */

static void sIIC_Start(void)
{
    SDA_OUT();
    SCL_L;
    SDA_H;
    SCL_H;
    delay_us(USERCONFIG_DELAYUS);
    SDA_L;
    delay_us(USERCONFIG_DELAYUS);
    SCL_L;
    delay_us(USERCONFIG_DELAYUS);
}

/**
 * @brief  Issue STOP condition.
 *         Final state: SCL=H, SDA=H, SDA=OUTPUT (idle).
 */
static void sIIC_Stop(void)
{
    SDA_OUT();
    SCL_L;
    SDA_L;
    SCL_H;
    delay_us(USERCONFIG_DELAYUS);
    SDA_H;
    delay_us(USERCONFIG_DELAYUS);
}

/**
 * @brief  Wait for ACK from slave. Issues STOP on timeout.
 * @return 1 = ACK received, 0 = timeout (STOP already issued)
 */
static uint8_t sIIC_WaitAck(uint32_t timeout)
{
    uint32_t time = 0;

    SDA_IN();
    SDA_H;
    delay_us(USERCONFIG_DELAYUS);
    SCL_H;
    delay_us(USERCONFIG_DELAYUS);

    while (READ_SDA()) {
        time++;
        delay_us(USERCONFIG_DELAYUS);
        if (time > timeout) {
            sIIC_Stop();        /* STOP on timeout */
            SDA_OUT();          /* Restore output mode */
            SCL_OUT();
            SDA_H;
            SCL_H;
            return 0;
        }
    }

    SCL_L;
    SDA_OUT();
    return 1;
}

static void sIIC_Ack(void)
{
    SDA_OUT();
    SCL_L;
    SDA_L;
    delay_us(USERCONFIG_DELAYUS);
    SCL_H;
    delay_us(USERCONFIG_DELAYUS);
    SCL_L;
    SDA_H;
}

static void sIIC_NAck(void)
{
    SDA_OUT();
    SCL_L;
    SDA_L;
    delay_us(USERCONFIG_DELAYUS);
    SDA_H;
    SCL_H;
    delay_us(USERCONFIG_DELAYUS);
    SCL_L;
    SDA_H;
}

static void sIIC_SendByte(uint8_t byte)
{
    SDA_OUT();
    SCL_L;

    for (uint8_t i = 0; i < 8; i++) {
        if (byte & 0x80) SDA_H; else SDA_L;
        delay_us(1);
        byte <<= 1;
        SCL_H;
        delay_us(1);
        SCL_L;
    }
}

static uint8_t sIIC_ReadByte(uint8_t ack)
{
    uint8_t byte = 0;

    SDA_IN();

    for (uint8_t i = 0; i < 8; i++) {
        SCL_L;
        delay_us(1);
        SCL_H;
        delay_us(1);
        byte <<= 1;
        if (READ_SDA()) byte |= 0x01;
        delay_us(1);
    }

    if (!ack) sIIC_NAck(); else sIIC_Ack();
    return byte;
}

/* ========================================================================
 * I2C Transaction Functions
 *
 * EVERY exit path — success or failure — guarantees:
 *   1. STOP condition issued
 *   2. SDA + SCL restored to OUTPUT, HIGH (idle)
 * ======================================================================== */

/* ---- Raw write: DevAddr + data bytes ---- */

static IIC_Status_t IIC_Master_Transmit(uint16_t dev_addr, uint8_t *data,
                                        uint16_t size, uint32_t timeout)
{
    sIIC_Start();
    sIIC_SendByte((uint8_t)dev_addr);
    if (!sIIC_WaitAck(timeout)) { return IIC_TIMEOUT; }
    /*                           ^ STOP + idle already done in WaitAck timeout */

    for (uint16_t i = 0; i < size; i++) {
        sIIC_SendByte(data[i]);
        if (!sIIC_WaitAck(timeout)) { return IIC_TIMEOUT; }
    }

    sIIC_Stop();
    return IIC_OK;
}

/* ---- Raw read: DevAddr|0x01 + read bytes ---- */

static IIC_Status_t IIC_Master_Receive(uint16_t dev_addr, uint8_t *data,
                                       uint16_t size, uint32_t timeout)
{
    sIIC_Start();
    sIIC_SendByte((uint8_t)(dev_addr | 0x01));
    if (!sIIC_WaitAck(timeout)) { return IIC_TIMEOUT; }

    for (uint16_t i = 0; i < size; i++) {
        data[i] = sIIC_ReadByte((i == (size - 1)) ? 0 : 1);
    }

    sIIC_Stop();
    return IIC_OK;
}

/* ---- Register write: DevAddr + RegAddr + data bytes ---- */

static IIC_Status_t IIC_Mem_Write(uint16_t dev_addr, uint16_t mem_addr,
                                  uint8_t *data, uint16_t size, uint32_t timeout)
{
    sIIC_Start();
    sIIC_SendByte((uint8_t)dev_addr);
    if (!sIIC_WaitAck(timeout)) { return IIC_TIMEOUT; }

    sIIC_SendByte((uint8_t)mem_addr);
    if (!sIIC_WaitAck(timeout)) { sIIC_RecoverBus(); return IIC_TIMEOUT; }

    for (uint16_t i = 0; i < size; i++) {
        sIIC_SendByte(data[i]);
        if (!sIIC_WaitAck(timeout)) { sIIC_RecoverBus(); return IIC_TIMEOUT; }
    }

    sIIC_Stop();
    return IIC_OK;
}

/* ---- Register read: DevAddr(wr) + RegAddr + restart + DevAddr(rd) + read ---- */

static IIC_Status_t IIC_Mem_Read(uint16_t dev_addr, uint16_t mem_addr,
                                 uint8_t *data, uint16_t size, uint32_t timeout)
{
    sIIC_Start();
    sIIC_SendByte((uint8_t)dev_addr);
    if (!sIIC_WaitAck(timeout)) { return IIC_TIMEOUT; }

    sIIC_SendByte((uint8_t)mem_addr);
    if (!sIIC_WaitAck(timeout)) { sIIC_RecoverBus(); return IIC_TIMEOUT; }

    /* Repeated start: switch to read */
    sIIC_Start();
    sIIC_SendByte((uint8_t)(dev_addr | 0x01));
    if (!sIIC_WaitAck(timeout)) { sIIC_RecoverBus(); return IIC_TIMEOUT; }

    for (uint16_t i = 0; i < size; i++) {
        data[i] = sIIC_ReadByte((i == (size - 1)) ? 0 : 1);
    }

    sIIC_Stop();
    return IIC_OK;
}

static void iic_delayms(uint16_t ms)
{
    delay_ms(ms);
}

/* ========================================================================
 * Public I2C Device Instance
 * ======================================================================== */
IICInterface_t User_sIICDev = {
    .init      = siic_init,
    .write     = IIC_Master_Transmit,
    .read      = IIC_Master_Receive,
    .write_reg = IIC_Mem_Write,
    .read_reg  = IIC_Mem_Read,
    .delay_ms  = iic_delayms
};

/* ========================================================================
 * Bus Recovery
 * ======================================================================== */

/**
 * @brief  Unlock stuck I2C bus — toggle SCL up to 9 times to free SDA
 *
 * If a slave holds SDA low (clock stretch or crash), the master can
 * pulse SCL to let the slave finish its pending bit. After 9 pulses
 * without SDA release the bus is considered hung.
 */
void mpu6050_i2c_sda_unlock(void)
{
    SDA_IN();
    for (uint8_t i = 0; i < 9; i++) {
        SCL_L;
        delay_us(10);
        SCL_H;
        delay_us(10);
        if (READ_SDA()) break;  /* SDA released — bus recovered */
    }
    SDA_OUT();
    SCL_OUT();
    SDA_H;
    SCL_H;
}
