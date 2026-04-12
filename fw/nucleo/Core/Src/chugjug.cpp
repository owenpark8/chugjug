#include <cstdarg>
#include <cstdio>

#include "filter.hpp"
#include "lis3dh.hpp"
#include "sound_effect.hpp"

#include "main.h"
#include "stm32u5xx_hal_dma_ex.h"
#include "stm32u5xx_hal_sai.h"

extern I2C_HandleTypeDef hi2c1;
extern SAI_HandleTypeDef hsai_BlockA1;
extern DMA_HandleTypeDef handle_GPDMA1_Channel0;
extern TIM_HandleTypeDef htim1;

#define LIS3DH_I2C_HANDLE hi2c1
#define SAI_HANDLE hsai_BlockA1
#define DMA_HANDLE handle_GPDMA1_Channel0
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

constexpr size_t NUM_UPDATES_FOR_IDLE_DETECTED = num_updates_in_n_ms(4000);
constexpr int IDLE_LOW_THRESHOLD_Z_MG = 935;
constexpr int IDLE_HIGH_THRESHOLD_Z_MG = 1045;
constexpr int IDLE_LOW_THRESHOLD_X_MG = -10;
constexpr int IDLE_HIGH_THRESHOLD_X_MG = 120;
constexpr int IDLE_LOW_THRESHOLD_Y_MG = -70;
constexpr int IDLE_HIGH_THRESHOLD_Y_MG = 60;

constexpr size_t ACCEL_FILTER_SIZE = 8;

extern int16_t const __flash_use_audio_start[];
namespace use_audio {
#ifdef DEBUG
    constexpr int16_t FIRST_EIGHT_AUDIO_SAMPLES[] = {0, 0, 1, 0, 1, 10, -6, 19};
    constexpr int16_t LAST_EIGHT_AUDIO_SAMPLES[] = {0, 0, 0, 0, 0, 0, 0, 0};
#endif
    constexpr size_t AUDIO_SIZE_BYTES = 1473500;
    constexpr size_t AUDIO_SAMPLE_SIZE_BYTES = sizeof(int16_t);
    constexpr size_t AUDIO_SIZE_SAMPLES = AUDIO_SIZE_BYTES / AUDIO_SAMPLE_SIZE_BYTES;
} // namespace use_audio
extern int16_t const __flash_pickup_audio_start[];
namespace pickup_audio {
    constexpr size_t AUDIO_SIZE_BYTES = 95978;
    constexpr size_t AUDIO_SAMPLE_SIZE_BYTES = sizeof(int16_t);
    constexpr size_t AUDIO_SIZE_SAMPLES = AUDIO_SIZE_BYTES / AUDIO_SAMPLE_SIZE_BYTES;
} // namespace pickup_audio

static bool volatile update_timer_flag = false;
static RunningMeanFilter<int, ACCEL_FILTER_SIZE> x_filter;
static RunningMeanFilter<int, ACCEL_FILTER_SIZE> y_filter;
static RunningMeanFilter<int, ACCEL_FILTER_SIZE> z_filter;
static bool volatile is_idle = false;
static bool volatile is_tilted = false;
static size_t tilt_detected_count = 0;
static size_t idle_detected_count = 0;
static SoundEffect<use_audio::AUDIO_SIZE_SAMPLES> use_sound_effect(__flash_use_audio_start, &DMA_HANDLE, &SAI_HANDLE);
static SoundEffect<pickup_audio::AUDIO_SIZE_SAMPLES> pickup_sound_effect(__flash_pickup_audio_start, &DMA_HANDLE, &SAI_HANDLE);
static bool volatile is_playing_use_sound_effect = false;
static bool volatile is_playing_pickup_sound_effect = false;
static bool volatile should_play_use_sound_effect = false;
static bool volatile should_play_pickup_sound_effect = false;

void init_standby_mode();
void init_lis3dh();
void init_audio();
void init();
void loop();
void error_handler();
void read_accel();
void update_timer_callback();
void sai_dma_complete_callback(DMA_HandleTypeDef* hdma);
void enter_standby_mode();

void log(char const* format, ...) {
#ifdef DEBUG
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
#endif
}

void init_standby_mode() {
    if (__HAL_PWR_GET_FLAG(PWR_FLAG_SBF) != RESET) {
        __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SBF);
        if (__HAL_PWR_GET_FLAG(PWR_WAKEUP_FLAG1) != RESET) {
            __HAL_PWR_CLEAR_FLAG(PWR_WAKEUP_FLAG1);
        }
        if (!is_playing_use_sound_effect && !is_playing_pickup_sound_effect) {
            should_play_pickup_sound_effect = true;
        }
    }
}

void init_lis3dh() {
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
}

void init_audio() {
#ifdef DEBUG
    auto use_audio_ptr = reinterpret_cast<const int16_t*>(__flash_use_audio_start);
    for (size_t i = 0; i < 8; i++) {
        if (use_audio_ptr[i] != use_audio::FIRST_EIGHT_AUDIO_SAMPLES[i]) {
            log(">audio_ptr:%d", (int) use_audio_ptr);
            log(">audio sample %d mismatch: expected 0x%04x, got 0x%04x\r\n", i, use_audio::FIRST_EIGHT_AUDIO_SAMPLES[i], use_audio_ptr[i]);
            error_handler();
        }
        size_t sample_index = use_audio::AUDIO_SIZE_SAMPLES - 8 + i;
        if (use_audio_ptr[sample_index] != use_audio::LAST_EIGHT_AUDIO_SAMPLES[i]) {
            log(">audio sample %d mismatch: expected 0x%04x, got 0x%04x\r\n", sample_index, use_audio::LAST_EIGHT_AUDIO_SAMPLES[i], use_audio_ptr[sample_index]);
            error_handler();
        }
    }
#endif

    if (use_sound_effect.init() == false) {
        error_handler();
    }
    if (pickup_sound_effect.init() == false) {
        error_handler();
    }

    if (HAL_DMA_RegisterCallback(&DMA_HANDLE, HAL_DMA_XFER_CPLT_CB_ID, sai_dma_complete_callback) != HAL_OK) {
        error_handler();
    }
}

void init() {
    HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);

    init_standby_mode();

    init_lis3dh();

    init_audio();

    if (should_play_pickup_sound_effect) {
        if (pickup_sound_effect.start_audio() == false) {
            error_handler();
        }
        is_playing_pickup_sound_effect = true;
        should_play_pickup_sound_effect = false;
    }

    // Pre-fill accel filters with initial readings to avoid startup spikes
    for (size_t i = 0; i < ACCEL_FILTER_SIZE; i++) {
        read_accel();
    }

    if (HAL_TIM_Base_Start_IT(&UPDATE_TIMER_HANDLE) != HAL_OK) {
        error_handler();
    }
}

void loop() {
    while (true) {
        if (update_timer_flag) {
            update_timer_flag = false;
            update_timer_callback();
        }
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
                should_play_use_sound_effect = true;
            }
            is_tilted = true;
        }
    } else {
        tilt_detected_count = 0;
        if (is_tilted) {
            log(">tilt:0\r\n");
            if (is_playing_use_sound_effect) {
                if (use_sound_effect.stop_audio() == false) {
                    error_handler();
                }
                is_playing_use_sound_effect = false;
            }
            should_play_use_sound_effect = false;
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
    if (should_play_use_sound_effect) {
        if (is_playing_pickup_sound_effect) {
            if (pickup_sound_effect.stop_audio() == false) {
                error_handler();
            }
        }
        if (use_sound_effect.start_audio() == false) {
            error_handler();
        }
        is_playing_use_sound_effect = true;
        should_play_use_sound_effect = false;
    }
}

void sai_dma_complete_callback(DMA_HandleTypeDef* hdma) {
    if (HAL_DMAEx_List_UnLinkQ(&DMA_HANDLE) != HAL_OK) {
        error_handler();
    }
    if (HAL_DMAEx_List_DeInit(&DMA_HANDLE) != HAL_OK) {
        error_handler();
    }
    if (is_playing_use_sound_effect && !is_playing_pickup_sound_effect) {
        is_playing_use_sound_effect = false;
    } else if (!is_playing_use_sound_effect && is_playing_pickup_sound_effect) {
        is_playing_pickup_sound_effect = false;
    } else {
        error_handler();
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
        update_timer_flag = true;
    }
}
}
