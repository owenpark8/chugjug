#pragma once

#include <cstdint>

#include "stm32u5xx_hal.h"

#define LIS3DH_ADDR (0x18 << 1)

namespace lis3dh {
    void set_i2c_timeout(int timeout_ms);
    int init(I2C_HandleTypeDef* hi2c);
    int read_accel(I2C_HandleTypeDef* hi2c, int16_t* x, int16_t* y,
                   int16_t* z);

    int who_am_i_check(I2C_HandleTypeDef* hi2c);
    int write_ctrl_reg1(I2C_HandleTypeDef* hi2c, int data_rate_hz = 0,
                        bool low_power_mode_enabled = false,
                        bool z_axis_enabled = true, bool y_axis_enabled = true,
                        bool x_axis_enabled = true);
} // namespace lis3dh
