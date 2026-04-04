#include "lis3dh.hpp"

#include "main.h"

extern I2C_HandleTypeDef hi2c1;

#define LIS3DH_I2C_HANDLE hi2c1

void init();
void loop();
void error_handler();


void init() {
    lis3dh::set_i2c_timeout(100);
    if (lis3dh::init(&LIS3DH_I2C_HANDLE) != 0) {
        error_handler();
    }
    // 100Hz data rate, all axes enabled
    if (lis3dh::write_ctrl_reg1(&LIS3DH_I2C_HANDLE, 100) != 0) {
        error_handler();
    }
}

void loop() {
    while (true) {
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
