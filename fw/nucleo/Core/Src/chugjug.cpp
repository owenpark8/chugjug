#include "lis3dh.hpp"

#include <cstdio>

#include "main.h"

extern I2C_HandleTypeDef hi2c1;

#define LIS3DH_I2C_HANDLE hi2c1

void init();
void loop();
void error_handler();

namespace {
    constexpr int ODR_HZ = 100;
}

void init() {
    lis3dh::set_i2c_timeout(100);
    if (lis3dh::init(&LIS3DH_I2C_HANDLE) != 0) {
        error_handler();
    }

    // 100Hz data rate, all axes enabled
    uint8_t const ctrl_reg1 = lis3dh::ctrl_reg1_odr(ODR_HZ) |
                              lis3dh::ctrl_reg1_low_power_mode(false) |
                              lis3dh::ctrl_reg1_z_axis(true) |
                              lis3dh::ctrl_reg1_y_axis(true) |
                              lis3dh::ctrl_reg1_x_axis(true);
    if (lis3dh::write_ctrl_reg1(&LIS3DH_I2C_HANDLE, ctrl_reg1) != 0) {
        error_handler();
    }

    // Enable interrupt on INT2 for z-high event of 1.104g
    uint8_t const int2_cfg = lis3dh::int2_cfg_zhie(true);
    if (lis3dh::write_int2_cfg(&LIS3DH_I2C_HANDLE, int2_cfg) != 0) {
        error_handler();
    }
    uint8_t const int2_ths = lis3dh::int2_ths_threshold_mg(1104, lis3dh::FullScale::FS_2G);
    if (lis3dh::write_int2_ths(&LIS3DH_I2C_HANDLE, int2_ths) != 0) {
        error_handler();
    }
}

void loop() {
    while (true) {
        int x_mg, y_mg, z_mg;
        if (lis3dh::read_accel_mg(&LIS3DH_I2C_HANDLE, &x_mg, &y_mg, &z_mg) != 0) {
            error_handler();
        }
        printf(">x:%d,y:%d,z:%d\r\n", x_mg, y_mg, z_mg);
        HAL_Delay(1000 / (ODR_HZ / 2));
    }
}

void error_handler() {

    Error_Handler();
}


extern "C" {
void chugjug_init(void) {
    init();
}
void chugjug_loop(void) {
    loop();
}
}
