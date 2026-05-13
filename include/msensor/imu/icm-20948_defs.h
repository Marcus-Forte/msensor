#pragma once

#include <cmath>

// Non comprehensive register list.
#define ICM20948_ADDR0 (0x69)

// Registers BANK0
#define REG_REG_BANK_SEL (0x7F)  // always the same from any bank.

#define REG_WHO_AM_I (0x00)
#define REG_USER_CTRL (0x03)
#define REG_LP_CONFIG (0x05)
#define REG_PWR_MGMT_1 (0x06)
#define REG_PWR_MGMT_2 (0x07)

#define REG_ACCEL_XOUT_H (0x2D)  // Try using block read.
#define REG_ACCEL_XOUT_L (0x2E)
#define REG_ACCEL_YOUT_H (0x2F)
#define REG_ACCEL_YOUT_L (0x30)
#define REG_ACCEL_ZOUT_H (0x31)
#define REG_ACCEL_ZOUT_L (0x32)

#define REG_GYRO_XOUT_H (0x33)
#define REG_GYRO_XOUT_L (0x34)
#define REG_GYRO_YOUT_H (0x35)
#define REG_GYRO_YOUT_L (0x36)
#define REG_GYRO_ZOUT_H (0x37)
#define REG_GYRO_ZOUT_L (0x38)

#define REG_DATA_RDY_STATUS (0x74)
#define REG_FIFO_MODE (0x69)

// Registers BANK1
#define REG_GYRO_CONFIG_1 (0x01)
#define REG_GYRO_CONFIG_2 (0x02)
#define REG_ACCEL_CONFIG_1 (0x14)
#define REG_ACCEL_CONFIG_2 (0x15)

#define REG_XA_OFFS_H (0x14)
#define REG_XA_OFFS_L (0x15)
#define REG_YA_OFFS_H (0x17)
#define REG_YA_OFFS_L (0x18)
#define REG_ZA_OFFS_H (0x1A)
#define REG_ZA_OFFS_L (0x1B)

// Registers Bank 2
#define REG_XG_OFFS_USRH (0x03)
#define REG_XG_OFFS_USRL (0x04)
#define REG_YG_OFFS_USRH (0x05)
#define REG_YG_OFFS_USRL (0x06)
#define REG_ZG_OFFS_USRH (0x07)
#define REG_ZG_OFFS_USRL (0x08)

// Constants
#define CONST_WHO_AM_I_VALUE (0xEA)

constexpr double GRAVITY = 9.80665;
// Conversion factors. Divide by int16 max, multiply by sensor full scale.
constexpr double FACTOR_GYRO_500DPS = 0.015259255f;  // 500 / 32767. Unit: degree / sec
constexpr double FACTOR_GYRO_500DPS_RADS =
    FACTOR_GYRO_500DPS * M_PI / 180.0f;  // Unit: radians / sec

constexpr double FACTOR_ACC_2G = 0.000061037f;                  // 2 / 32767. Unit: G's
constexpr double FACTOR_ACC_2G_MS = FACTOR_ACC_2G * GRAVITY;    // Unit: m/s²
constexpr double FACTOR_ACC_16G = 0.000488296f;                 // 16 / 32767. Unit: G's
constexpr double FACTOR_ACC_16G_MS = FACTOR_ACC_16G * GRAVITY;  // Unit: m/s²