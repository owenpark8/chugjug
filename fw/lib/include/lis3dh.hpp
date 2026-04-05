#pragma once

#include <cstdint>

#include "stm32u5xx_hal.h"

#define LIS3DH_ADDR (0x18 << 1)

namespace lis3dh {

    enum class FullScale {
        FS_2G = 0b00,
        FS_4G = 0b01,
        FS_8G = 0b10,
        FS_16G = 0b11
    };

    enum class Resolution {
        LOW_POWER_8_BIT = 0,
        NORMAL_10_BIT = 1,
        HIGH_RESOLUTION_12_BIT = 2
    };

    void set_i2c_timeout(int timeout_ms);
    int init(I2C_HandleTypeDef* hi2c);
    int read_accel_mg(I2C_HandleTypeDef* hi2c, int* x_mg, int* y_mg, int* z_mg, Resolution res = Resolution::NORMAL_10_BIT, FullScale fs = FullScale::FS_2G);

    int who_am_i_check(I2C_HandleTypeDef* hi2c);

    // CTRL_REG1
    int write_ctrl_reg1(I2C_HandleTypeDef* hi2c, uint8_t ctrl_reg1);
    uint8_t ctrl_reg1_odr(int data_rate_hz = 0);
    uint8_t ctrl_reg1_low_power_mode(bool enabled = false);
    uint8_t ctrl_reg1_z_axis(bool enabled = true);
    uint8_t ctrl_reg1_y_axis(bool enabled = true);
    uint8_t ctrl_reg1_x_axis(bool enabled = true);

    // INT1_CFG
    int write_int1_cfg(I2C_HandleTypeDef* hi2c, uint8_t int1_cfg);
    uint8_t int1_cfg_aoi(bool enabled = false);
    uint8_t int1_cfg_6d(bool enabled = false);
    uint8_t int1_cfg_zhie(bool enabled = false);
    uint8_t int1_cfg_zlie(bool enabled = false);
    uint8_t int1_cfg_yhie(bool enabled = false);
    uint8_t int1_cfg_ylie(bool enabled = false);
    uint8_t int1_cfg_xhie(bool enabled = false);
    uint8_t int1_cfg_xlie(bool enabled = false);

    // INT2_CFG
    int write_int2_cfg(I2C_HandleTypeDef* hi2c, uint8_t int2_cfg);
    uint8_t int2_cfg_aoi(bool enabled = false);
    uint8_t int2_cfg_6d(bool enabled = false);
    uint8_t int2_cfg_zhie(bool enabled = false);
    uint8_t int2_cfg_zlie(bool enabled = false);
    uint8_t int2_cfg_yhie(bool enabled = false);
    uint8_t int2_cfg_ylie(bool enabled = false);
    uint8_t int2_cfg_xhie(bool enabled = false);
    uint8_t int2_cfg_xlie(bool enabled = false);

    // INT2_THS
    int write_int2_ths(I2C_HandleTypeDef* hi2c, uint8_t int2_ths);
    uint8_t int2_ths_threshold_mg(int threshold_g, FullScale fs = FullScale::FS_2G);

} // namespace lis3dh
