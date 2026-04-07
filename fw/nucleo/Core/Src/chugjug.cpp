#include "lis3dh.hpp"

#include <cstdio>

#include "main.h"

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;

#define LIS3DH_I2C_HANDLE hi2c1

void init();
void loop();
void error_handler();
void read_accel_callback();
void handle_int2_interrupt();
void enter_standby_mode();

void init() {
    HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);

    // ===== STANDBY MODE =====
    if (__HAL_PWR_GET_FLAG(PWR_FLAG_SBF) != RESET) {
        __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SBF);
        if (__HAL_PWR_GET_FLAG(PWR_WAKEUP_FLAG1) != RESET) {
            __HAL_PWR_CLEAR_FLAG(PWR_WAKEUP_FLAG1);
        }
    }

    // ===== LIS3DH =====
    lis3dh::set_i2c_timeout(100);
    if (lis3dh::init(&LIS3DH_I2C_HANDLE) != 0) {
        error_handler();
    }
    // 100Hz data rate, all axes enabled
    uint8_t const ctrl_reg1 = lis3dh::ctrl_reg1_odr(lis3dh::DataRate::ODR_100_HZ) |
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
    uint8_t const int2_duration = lis3dh::int2_duration_num_samples(1);
    if (lis3dh::write_int2_duration(&LIS3DH_I2C_HANDLE, int2_duration) != 0) {
        error_handler();
    }
    uint8_t const ctrl_reg6 = lis3dh::ctrl_reg6_i2_ia2(true);
    if (lis3dh::write_ctrl_reg6(&LIS3DH_I2C_HANDLE, ctrl_reg6) != 0) {
        error_handler();
    }

    // ===== UPDATE TIMER =====
    HAL_TIM_Base_Start_IT(&htim1);
}

void loop() {
    while (true) {
    }
}

void error_handler() {
    Error_Handler();
}

void read_accel_callback() {
    int x_mg, y_mg, z_mg;
    if (lis3dh::read_accel_mg(&LIS3DH_I2C_HANDLE, &x_mg, &y_mg, &z_mg) != 0) {
        error_handler();
    }
    printf(">x:%d,y:%d,z:%d\r\n", x_mg, y_mg, z_mg);
}

void handle_int2_interrupt() {
    printf(">int2:1000\r\n");
    printf("INT2 interrupt triggered!\r\n");
    printf(">int2:0\r\n");
}

void enter_standby_mode() {
    printf("Entering standby mode...\r\n");
    __HAL_PWR_CLEAR_FLAG(PWR_WAKEUP_FLAG1);
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1_HIGH_0);
    HAL_PWR_EnterSTANDBYMode();
}


extern "C" {
void chugjug_init(void) {
    init();
}
void chugjug_loop(void) {
    loop();
}

void chugjug_gpio_exti_callback(uint16_t gpio_pin) {
    if (gpio_pin == LIS3DH_INT2_Pin) {
        handle_int2_interrupt();
    } else if (gpio_pin == USER_BUTTON_Pin) {
        enter_standby_mode();
    }
}

void chugjug_timer_period_elapsed_callback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == TIM1) {
        read_accel_callback();
    }
}
}
