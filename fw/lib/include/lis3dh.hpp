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

    enum class DataRate {
        POWER_DOWN = 0x00,
        ODR_1_HZ = 0x10,
        ODR_10_HZ = 0x20,
        ODR_25_HZ = 0x30,
        ODR_50_HZ = 0x40,
        ODR_100_HZ = 0x50,
        ODR_200_HZ = 0x60,
        ODR_400_HZ = 0x70,
        ODR_1600_HZ = 0x80,
    };

    constexpr int data_rate_to_hz(DataRate data_rate) {
        switch (data_rate) {
            case DataRate::POWER_DOWN:
                return 0;
            case DataRate::ODR_1_HZ:
                return 1;
            case DataRate::ODR_10_HZ:
                return 10;
            case DataRate::ODR_25_HZ:
                return 25;
            case DataRate::ODR_50_HZ:
                return 50;
            case DataRate::ODR_100_HZ:
                return 100;
            case DataRate::ODR_200_HZ:
                return 200;
            case DataRate::ODR_400_HZ:
                return 400;
            case DataRate::ODR_1600_HZ:
                return 1600;
            default:
                return 0;
        }
    }

    void set_i2c_timeout(int timeout_ms);
    int init(I2C_HandleTypeDef* hi2c);
    int read_accel_mg(I2C_HandleTypeDef* hi2c, int* x_mg, int* y_mg, int* z_mg, Resolution res = Resolution::NORMAL_10_BIT, FullScale fs = FullScale::FS_2G);

    int who_am_i_check(I2C_HandleTypeDef* hi2c);

    // CTRL_REG1
    int write_ctrl_reg1(I2C_HandleTypeDef* hi2c, uint8_t ctrl_reg1);
    uint8_t ctrl_reg1_odr(DataRate data_rate = DataRate::POWER_DOWN);
    uint8_t ctrl_reg1_low_power_mode(bool enabled = false);
    uint8_t ctrl_reg1_z_axis(bool enabled = true);
    uint8_t ctrl_reg1_y_axis(bool enabled = true);
    uint8_t ctrl_reg1_x_axis(bool enabled = true);

    // CTRL_REG6
    int write_ctrl_reg6(I2C_HandleTypeDef* hi2c, uint8_t ctrl_reg6);
    uint8_t ctrl_reg6_i2_ia2(bool enabled = false);

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

    // INT1_THS
    int write_int1_ths(I2C_HandleTypeDef* hi2c, uint8_t int1_ths);
    uint8_t int1_ths_threshold_mg(int threshold_g, FullScale fs = FullScale::FS_2G);

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

    // INT2_DURATION
    int write_int2_duration(I2C_HandleTypeDef* hi2c, uint8_t int2_duration);
    uint8_t int2_duration_num_samples(int num_samples);

} // namespace lis3dh
