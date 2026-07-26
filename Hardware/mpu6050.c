/**
 * @file    mpu6050.c
 * @brief   MPU6050 6-axis IMU driver with Mahony AHRS attitude estimation
 *
 * Ported from: WHEELTEC_C07A_BalanceCar/BSP/bsp_mpu6050.c
 *
 * Hardware: MPU6050 (3-axis gyro + 3-axis accelerometer)
 * Interface: Software I2C via bsp_siic.h (SDA=PA0, SCL=PA1)
 *
 * Register configuration:
 *   - Gyro:    ±2000 dps  (scale 16.4 LSB/dps)
 *   - Accel:   ±2g        (scale 16384 LSB/g, output in g units per API)
 *   - DLPF:    CFG=3, ~44 Hz bandwidth (closest to target 42/44 Hz)
 *   - Sample:  SMPLRT_DIV=4 → 1kHz / (1+4) = 200 Hz
 *
 * Timing budget (80 MHz Cortex-M0+, no FPU):
 *   - Software I2C 14-byte read:         ~350–500 µs
 *   - Mahony AHRS (quaternion + 3 trig): ~600–1100 µs
 *   - Total worst-case per iteration:    ~1600 µs < 5000 µs ✓
 *
 * API (see mpu6050.h):
 *   MPU6050_Init()        — Initialize I2C + configure MPU6050 registers
 *   MPU6050_DataReady()   — Check if new data available since last read
 *   MPU6050_Read()        — Read gyro(deg/s), accel(g), angle(deg) all at once
 *   MPU6050_GetYaw()      — Get Z-axis yaw angle (deg)
 *   MPU6050_GetPitch()    — Get pitch angle (deg)
 *   MPU6050_GetRoll()     — Get roll angle (deg)
 *   MPU6050_GetGyroZ()    — Get Z-axis angular velocity (deg/s)
 *
 * Call flow (200 Hz, from TIMER ISR or main loop):
 *   1. mpu6050_read_raw()     — Read 14 bytes via I2C
 *   2. mpu6050_update_ahrs()  — Mahony AHRS → roll/pitch/yaw
 *   3. MPU6050_GetYaw() etc.  — Read the latest angles
 */

#include "mpu6050.h"
#include "bsp_siic.h"
#include "board.h"
#include <math.h>

/* ========================================================================
 * MPU6050 Register Addresses (from reference bsp_mpu6050.h)
 * ======================================================================== */
#define MPU6050_DEV              0x68
#define MPU6050_RA_SMPLRT_DIV    0x19
#define MPU6050_RA_CONFIG        0x1A
#define MPU6050_RA_GYRO_CONFIG   0x1B
#define MPU6050_RA_ACCEL_CONFIG  0x1C
#define MPU6050_RA_INT_PIN_CFG   0x37
#define MPU6050_RA_INT_ENABLE    0x38
#define MPU6050_RA_ACCEL_XOUT_H  0x3B
#define MPU6050_RA_USER_CTRL     0x6A
#define MPU6050_RA_PWR_MGMT_1    0x6B
#define MPU6050_RA_PWR_MGMT_2    0x6C
#define MPU6050_RA_FIFO_EN       0x23
#define MPU6050_RA_WHO_AM_I      0x75

/* ========================================================================
 * Internal Data Structures
 * ======================================================================== */

typedef struct {
    float x, y, z;
} imu_val_t;

typedef struct {
    imu_val_t gyro;    /* Angular velocity (rad/s) */
    imu_val_t accel;   /* Linear acceleration (m/s^2) */
    float     roll;    /* Roll angle (deg) */
    float     pitch;   /* Pitch angle (deg) */
    float     yaw;     /* Yaw angle (deg) */
} Imu_t;

/* ========================================================================
 * Module State
 * ======================================================================== */
static Imu_t g_mpu6050;
static bool  g_initialized = false;
static bool  g_data_ready  = false;   /* Set after each successful read */

/* ---- Conversion constants ---- */
#define GYRO_SCALE_FACTOR    (0.060975609756f)   /* 1/16.4: LSB → °/s for ±2000dps     */
#define ACCEL_SCALE_FACTOR   (1.0f / 16384.0f)   /* 1/16384: LSB → g for ±2g           */
#define DEG_TO_RAD           (0.01745329252f)     /* π/180                               */

/* ---- Mahony AHRS constants ---- */
#define AHRS_KP              (0.8f)      /* Proportional gain              */
#define AHRS_KI              (0.0001f)   /* Integral gain                  */
#define AHRS_HALF_T          (0.0025f)   /* 0.5 * dt (dt = 0.005s @200Hz) */

/* ========================================================================
 * Static Helpers
 * ======================================================================== */

/**
 * @brief  Fast inverse square root (Quake 3 algorithm)
 */
static float Q_rsqrt(float number)
{
    long i;
    float x2, y;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    y  = number;
    i  = *(long *)&y;
    i  = 0x5f3759df - (i >> 1);
    y  = *(float *)&i;
    y  = y * (threehalfs - (x2 * y * y));
    return y;
}

/**
 * @brief  Mahony AHRS — quaternion-based attitude update
 *
 * Fuses gyroscope and accelerometer data into roll/pitch/yaw.
 * Uses proportional-integral feedback to correct gyro drift
 * from the gravity vector measured by the accelerometer.
 *
 * @param imu  Pointer to Imu_t (reads gyro/accel, writes roll/pitch/yaw)
 */
static void mpu6050_update_ahrs(Imu_t *imu)
{
    static float eInt[3] = {0.0f};
    static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    /* ---- Get sensor readings ---- */
    float gx = imu->gyro.x;
    float gy = imu->gyro.y;
    float gz = imu->gyro.z;
    float ax = imu->accel.x;
    float ay = imu->accel.y;
    float az = imu->accel.z;

    /* ---- Pre-compute quaternion products ---- */
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    /* ---- Extract gravity vector from quaternion ---- */
    vx = 2.0f * (q1q3 - q0q2);
    vy = 2.0f * (q0q1 + q2q3);
    vz = 1.0f - 2.0f * (q1q1 + q2q2);

    /* ---- Normalize accelerometer ---- */
    norm = Q_rsqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    /* ---- Cross product: accelerometer × gravity = orientation error ---- */
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    /* ---- PI correction on gyro bias ---- */
    eInt[0] += ex;
    eInt[1] += ey;
    eInt[2] += ez;

    gx += AHRS_KP * ex + AHRS_KI * eInt[0];
    gy += AHRS_KP * ey + AHRS_KI * eInt[1];
    gz += AHRS_KP * ez + AHRS_KI * eInt[2];

    /* ---- Quaternion integration (first-order) ---- */
    q0 += (-q1 * gx - q2 * gy - q3 * gz) * AHRS_HALF_T;
    q1 += ( q0 * gx + q2 * gz - q3 * gy) * AHRS_HALF_T;
    q2 += ( q0 * gy - q1 * gz + q3 * gx) * AHRS_HALF_T;
    q3 += ( q0 * gz + q1 * gy - q2 * gx) * AHRS_HALF_T;

    /* ---- Normalize quaternion ---- */
    norm = Q_rsqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;

    /* ---- Quaternion → Euler angles (degrees) ---- */
    imu->roll  = atan2f(2.0f * (q2 * q3 + q0 * q1),
                        -2.0f * q1 * q1 - 2.0f * q2 * q2 + 1.0f) * 57.295779513f;
    imu->pitch = asinf(-2.0f * q1 * q3 + 2.0f * q0 * q2) * 57.295779513f;
    imu->yaw   = atan2f(2.0f * (q1 * q2 + q0 * q3),
                         q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.295779513f;
}

/**
 * @brief  Read raw accelerometer + gyro data from MPU6050 via I2C
 * @return true on success, false on I2C error
 *
 * Reads 14 bytes starting from ACCEL_XOUT_H:
 *   [0-1] ACCEL_X,  [2-3] ACCEL_Y,  [4-5] ACCEL_Z
 *   [6-7] TEMP,     [8-9] GYRO_X,   [10-11] GYRO_Y,  [12-13] GYRO_Z
 */
static bool mpu6050_read_raw(void)
{
    pIICInterface_t iic = &User_sIICDev;
    uint8_t buf[14] = {0};

    IIC_Status_t status = iic->read_reg((uint16_t)(MPU6050_DEV << 1),
                                        MPU6050_RA_ACCEL_XOUT_H,
                                        buf, 14, 100);

    if (status != IIC_OK) {
        g_data_ready = false;
        return false;
    }

    /* ---- Parse raw data (big-endian) ---- */
    g_mpu6050.accel.x = (float)((int16_t)((buf[0] << 8) | buf[1]));
    g_mpu6050.accel.y = (float)((int16_t)((buf[2] << 8) | buf[3]));
    g_mpu6050.accel.z = (float)((int16_t)((buf[4] << 8) | buf[5]));

    g_mpu6050.gyro.x = (float)((int16_t)((buf[8]  << 8) | buf[9]));
    g_mpu6050.gyro.y = (float)((int16_t)((buf[10] << 8) | buf[11]));
    g_mpu6050.gyro.z = (float)((int16_t)((buf[12] << 8) | buf[13]));

    /* ---- Scale: LSB → physical units ----
     * Gyro:  LSB * 1/16.4 → °/s, then → rad/s
     * Accel: LSB * 1/16384 → g   */
    g_mpu6050.gyro.x *= GYRO_SCALE_FACTOR * DEG_TO_RAD;
    g_mpu6050.gyro.y *= GYRO_SCALE_FACTOR * DEG_TO_RAD;
    g_mpu6050.gyro.z *= GYRO_SCALE_FACTOR * DEG_TO_RAD;

    g_mpu6050.accel.x *= ACCEL_SCALE_FACTOR;
    g_mpu6050.accel.y *= ACCEL_SCALE_FACTOR;
    g_mpu6050.accel.z *= ACCEL_SCALE_FACTOR;

    g_data_ready = true;
    return true;
}

/* ========================================================================
 * Public Functions — API matching mpu6050.h
 * ======================================================================== */

/**
 * @brief  Initialize MPU6050 and configure for 200Hz operation
 *
 * Steps:
 *   1. Verify device presence (WHO_AM_I == 0x68)
 *   2. Reset MPU6050
 *   3. Wake up, set gyro X-axis as PLL clock source
 *   4. Configure gyro ±2000dps, accel ±2g
 *   5. Set DLPF bandwidth 42Hz, sample rate 200Hz
 *   6. Disable interrupts and FIFO
 */
void MPU6050_Init(void)
{
    /* Idempotent guard — only run once per boot */
    if (g_initialized) return;

    pIICInterface_t iic = &User_sIICDev;
    uint8_t iobuf;
    IIC_Status_t check_state = IIC_OK;

    /* Init software I2C */
    iic->init();

    /* ---- Check device presence ---- */
    iobuf = 0;
    check_state += iic->read_reg((uint16_t)(MPU6050_DEV << 1),
                                 MPU6050_RA_WHO_AM_I, &iobuf, 1, 500);
    if (iobuf != MPU6050_DEV) {
        /* MPU6050 not found — leave uninitialized */
        return;
    }

    /* ---- Reset MPU6050 ---- */
    iobuf = (uint8_t)(1 << 7);
    check_state += iic->write_reg((uint16_t)(MPU6050_DEV << 1),
                                  MPU6050_RA_PWR_MGMT_1, &iobuf, 1, 500);

    /* Wait for reset to complete */
    iic->delay_ms(200);

    /* ---- Wake up, set gyro X as PLL clock ---- */
    iobuf = (uint8_t)(1 << 0);
    check_state += iic->write_reg((uint16_t)(MPU6050_DEV << 1),
                                  MPU6050_RA_PWR_MGMT_1, &iobuf, 1, 500);

    /* ---- Enable both accel and gyro (disable low-power) ---- */
    iobuf = 0;
    check_state += iic->write_reg((uint16_t)(MPU6050_DEV << 1),
                                  MPU6050_RA_PWR_MGMT_2, &iobuf, 1, 500);

    /* ---- Gyro ±2000dps ---- */
    iobuf = (uint8_t)(3 << 3);
    check_state += iic->write_reg((uint16_t)(MPU6050_DEV << 1),
                                  MPU6050_RA_GYRO_CONFIG, &iobuf, 1, 500);

    /* ---- Accel ±2g ---- */
    iobuf = (uint8_t)(0 << 4);
    check_state += iic->write_reg((uint16_t)(MPU6050_DEV << 1),
                                  MPU6050_RA_ACCEL_CONFIG, &iobuf, 1, 500);

    /* ---- DLPF bandwidth ~44 Hz (DLPF_CFG=3, ~4.9ms lag, 1kHz gyro output) ---- */
    iobuf = (uint8_t)(3 << 0);
    check_state += iic->write_reg((uint16_t)(MPU6050_DEV << 1),
                                  MPU6050_RA_CONFIG, &iobuf, 1, 500);

    /* ---- Sample rate divider: 1kHz / (1 + 4) = 200Hz ---- */
    iobuf = 4;  /* SMPLRT_DIV = 4 → 1000/(1+4) = 200Hz */
    check_state += iic->write_reg((uint16_t)(MPU6050_DEV << 1),
                                  MPU6050_RA_SMPLRT_DIV, &iobuf, 1, 500);

    /* ---- Disable interrupts ---- */
    iobuf = 0;
    check_state += iic->write_reg((uint16_t)(MPU6050_DEV << 1),
                                  MPU6050_RA_INT_ENABLE, &iobuf, 1, 500);

    /* ---- Disable FIFO ---- */
    iobuf = 0;
    check_state += iic->write_reg((uint16_t)(MPU6050_DEV << 1),
                                  MPU6050_RA_FIFO_EN, &iobuf, 1, 500);

    /* ---- Disable I2C master, FIFO; enable I2C interface ---- */
    iobuf = 0;
    check_state += iic->write_reg((uint16_t)(MPU6050_DEV << 1),
                                  MPU6050_RA_USER_CTRL, &iobuf, 1, 500);

    /* ---- Enable I2C bypass mode ---- */
    iobuf = (uint8_t)(1 << 1);
    check_state += iic->write_reg((uint16_t)(MPU6050_DEV << 1),
                                  MPU6050_RA_INT_PIN_CFG, &iobuf, 1, 500);

    if (check_state == IIC_OK) {
        g_initialized = true;
    }
}

/**
 * @brief  Check if new MPU6050 data is available
 * @return true after a successful mpu6050_read_raw() call
 */
bool MPU6050_DataReady(void)
{
    return g_data_ready;
}

/**
 * @brief  Read all MPU6050 data: gyro, accel, and Euler angles
 *
 * Performs a complete read cycle:
 *   1. Read raw sensor registers via I2C
 *   2. Run Mahony AHRS to update roll/pitch/yaw
 *   3. Copy results to output arrays
 *
 * @param  gyro   [out] Gyroscope (deg/s): [gx, gy, gz] or NULL
 * @param  accel  [out] Accelerometer (g): [ax, ay, az] or NULL
 * @param  angle  [out] Euler angles (deg): [roll, pitch, yaw] or NULL
 *
 * @note   Call at CONTROL_FREQ_HZ (200Hz) for best results
 */
void MPU6050_Read(float gyro[3], float accel[3], float angle[3])
{
    if (!g_initialized) {
        if (gyro)  { gyro[0] = 0.0f;  gyro[1] = 0.0f;  gyro[2] = 0.0f; }
        if (accel) { accel[0] = 0.0f; accel[1] = 0.0f; accel[2] = 0.0f; }
        if (angle) { angle[0] = 0.0f; angle[1] = 0.0f; angle[2] = 0.0f; }
        return;
    }

    /* Read raw sensor data via I2C */
    if (!mpu6050_read_raw()) {
        g_data_ready = false;
        if (gyro)  { gyro[0] = 0.0f;  gyro[1] = 0.0f;  gyro[2] = 0.0f; }
        if (accel) { accel[0] = 0.0f; accel[1] = 0.0f; accel[2] = 0.0f; }
        if (angle) { angle[0] = 0.0f; angle[1] = 0.0f; angle[2] = 0.0f; }
        return;
    }

    /* Update attitude via Mahony AHRS */
    mpu6050_update_ahrs(&g_mpu6050);

    /* Copy to output arrays */
    if (gyro) {
        gyro[0] = g_mpu6050.gyro.x / DEG_TO_RAD;    /* rad/s → deg/s */
        gyro[1] = g_mpu6050.gyro.y / DEG_TO_RAD;
        gyro[2] = g_mpu6050.gyro.z / DEG_TO_RAD;
    }
    if (accel) {
        accel[0] = g_mpu6050.accel.x;
        accel[1] = g_mpu6050.accel.y;
        accel[2] = g_mpu6050.accel.z;
    }
    if (angle) {
        angle[0] = g_mpu6050.roll;
        angle[1] = g_mpu6050.pitch;
        angle[2] = g_mpu6050.yaw;
    }

    g_data_ready = true;
}

/* ---- Individual accessors ---- */

float MPU6050_GetYaw(void)
{
    return g_mpu6050.yaw;
}

float MPU6050_GetPitch(void)
{
    return g_mpu6050.pitch;
}

float MPU6050_GetRoll(void)
{
    return g_mpu6050.roll;
}

float MPU6050_GetGyroZ(void)
{
    /* Return Z-axis gyro in deg/s */
    return g_mpu6050.gyro.z / DEG_TO_RAD;
}
