#include <cstdarg>
#include <cstdio>

#include "filter.hpp"
#include "lis3dh.hpp"

#include "main.h"

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;

#define LIS3DH_I2C_HANDLE hi2c1
#define UPDATE_TIMER_HANDLE htim1

struct AccelData {
    int x_mg;
    int y_mg;
    int z_mg;
};

constexpr uint32_t UPDATE_TIMER_FREQUENCY_HZ = 50;
constexpr uint32_t UPDATE_TIMER_PERIOD_MS = 1000 / UPDATE_TIMER_FREQUENCY_HZ;

constexpr size_t num_updates_in_n_ms(size_t ms) {
    return (ms + UPDATE_TIMER_PERIOD_MS - 1) / UPDATE_TIMER_PERIOD_MS;
}
constexpr size_t NUM_UPDATES_FOR_TILT_DETECTED = num_updates_in_n_ms(400);
constexpr int TILT_LOW_THRESHOLD_Z_MG = 980;

constexpr size_t NUM_UPDATES_FOR_IDLE_DETECTED = num_updates_in_n_ms(10000);
constexpr int IDLE_LOW_THRESHOLD_Z_MG = 935;
constexpr int IDLE_HIGH_THRESHOLD_Z_MG = 1045;
constexpr int IDLE_LOW_THRESHOLD_X_MG = -10;
constexpr int IDLE_HIGH_THRESHOLD_X_MG = 120;
constexpr int IDLE_LOW_THRESHOLD_Y_MG = -70;
constexpr int IDLE_HIGH_THRESHOLD_Y_MG = 60;

constexpr size_t ACCEL_FILTER_SIZE = 8;

RunningMeanFilter<int, ACCEL_FILTER_SIZE> x_filter;
RunningMeanFilter<int, ACCEL_FILTER_SIZE> y_filter;
RunningMeanFilter<int, ACCEL_FILTER_SIZE> z_filter;
static bool volatile is_idle = false;
static bool volatile is_tilted = false;
static size_t tilt_detected_count = 0;
static size_t idle_detected_count = 0;

#ifdef DEBUG
constexpr int16_t FIRST_EIGHT_AUDIO_SAMPLES[] = {0, 0, 1, 0, 1, 10, -6, 19};
constexpr int16_t LAST_EIGHT_AUDIO_SAMPLES[] = {0, 0, 0, 0, 0, 0, 0, 0};
#endif
extern uint8_t const __flash_audio_start[];
constexpr size_t AUDIO_SIZE_BYTES = 1473500;
constexpr size_t AUDIO_SIZE_SAMPLES = AUDIO_SIZE_BYTES / sizeof(int16_t);

void init();
void loop();
void error_handler();
void read_accel();
void update_timer_callback();
void enter_standby_mode();

void log(char const* format, ...) {
#ifdef DEBUG
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
#endif
}

void init() {
    HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);

#ifdef DEBUG
    const auto* audio_ptr = reinterpret_cast<const int16_t*>(__flash_audio_start);
    for (size_t i = 0; i < 8; i++) {
        if (audio_ptr[i] != FIRST_EIGHT_AUDIO_SAMPLES[i]) {
            log(">audio sample %d mismatch: expected 0x%04x, got 0x%04x\r\n", i, FIRST_EIGHT_AUDIO_SAMPLES[i], audio_ptr[i]);
            return;
        } else {
            log(">audio_sample_%d:%d\r\n", i, audio_ptr[i]);
        }
        size_t sample_index = AUDIO_SIZE_SAMPLES - 8 + i;
        if (audio_ptr[sample_index] != LAST_EIGHT_AUDIO_SAMPLES[i]) {
            log(">audio sample %d mismatch: expected 0x%04x, got 0x%04x\r\n", sample_index, LAST_EIGHT_AUDIO_SAMPLES[i], audio_ptr[sample_index]);
            return;
        } else {
            log(">audio_sample_%d:%d\r\n", sample_index, audio_ptr[sample_index]);
        }
    }
#endif

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
    // Enable interrupt on INT2 for z-high event of 1.104g to trigger wake up from standby mode
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

    // Pre-fill accel filters with initial readings to avoid startup spikes
    for (size_t i = 0; i < ACCEL_FILTER_SIZE; i++) {
        read_accel();
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

void read_accel() {
    int x_mg, y_mg, z_mg;
    if (lis3dh::read_accel_mg(&LIS3DH_I2C_HANDLE, &x_mg, &y_mg, &z_mg) != 0) {
        error_handler();
    }
    x_filter.add(x_mg);
    y_filter.add(y_mg);
    z_filter.add(z_mg);
}

void update_timer_callback() {
    read_accel();
    int const x_mean = x_filter.mean();
    int const y_mean = y_filter.mean();
    int const z_mean = z_filter.mean();
    log(">x:%d,y:%d,z:%d\r\n", x_mean, y_mean, z_mean);

    if (z_mean < TILT_LOW_THRESHOLD_Z_MG) {
        tilt_detected_count++;
        if (tilt_detected_count >= NUM_UPDATES_FOR_TILT_DETECTED) {
            if (!is_tilted) {
                log(">tilt:%d\r\n", TILT_LOW_THRESHOLD_Z_MG);
            }
            is_tilted = true;
        }
    } else {
        tilt_detected_count = 0;
        if (is_tilted) {
            log(">tilt:0\r\n");
        }
        is_tilted = false;
    }

    if (z_mean > IDLE_LOW_THRESHOLD_Z_MG && z_mean < IDLE_HIGH_THRESHOLD_Z_MG &&
        x_mean > IDLE_LOW_THRESHOLD_X_MG && x_mean < IDLE_HIGH_THRESHOLD_X_MG &&
        y_mean > IDLE_LOW_THRESHOLD_Y_MG && y_mean < IDLE_HIGH_THRESHOLD_Y_MG) {
        idle_detected_count++;
        if (idle_detected_count >= NUM_UPDATES_FOR_IDLE_DETECTED) {
            is_idle = true;
        }
    } else {
        idle_detected_count = 0;
        is_idle = false;
    }

    if (is_idle) {
        enter_standby_mode();
        return;
    }
}

void enter_standby_mode() {
    log("Entering standby mode...\r\n");
    is_idle = false;
    idle_detected_count = 0;
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
    // if (gpio_pin == USER_BUTTON_Pin) {
    //     is_idle = true;
    // }
}

void chugjug_timer_period_elapsed_callback(TIM_HandleTypeDef* htim) {
    if (htim == &UPDATE_TIMER_HANDLE) {
        update_timer_callback();
    }
}
}
