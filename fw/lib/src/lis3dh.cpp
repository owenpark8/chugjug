#include "lis3dh.hpp"

namespace lis3dh {

    static constexpr uint8_t LIS3DH_REG_WHO_AM_I = 0x0F;
    static constexpr uint8_t LIS3DH_REG_CTRL_REG1 = 0x20;
    static constexpr uint8_t LIS3DH_REG_OUT_X_L = 0x28;
    static constexpr uint8_t LIS3DH_INT1_CFG = 0x30;
    static constexpr uint8_t LIS3DH_INT2_CFG = 0x34;

    static int I2C_TIMEOUT_MS = 100;

    // Sensitivity table in mg/digit [FS][Resolution]
    // Indexed as [FullScale][LOW_POWER, NORMAL, HIGH_RES]
    static constexpr int SENSITIVITY[4][3] = {
            {16, 4, 1},    // FS = 00 (±2g)
            {32, 8, 2},    // FS = 01 (±4g)
            {64, 16, 4},   // FS = 10 (±8g)
            {192, 48, 12}, // FS = 11 (±16g)
    };

    static int write_register(I2C_HandleTypeDef* hi2c, uint8_t reg, uint8_t value) {
        if (HAL_I2C_Mem_Write(hi2c, LIS3DH_ADDR, reg, 1,
                              &value, 1, I2C_TIMEOUT_MS) != HAL_OK) {
            return -1;
        }
        return 0;
    }

    void set_i2c_timeout(int timeout_ms) { I2C_TIMEOUT_MS = timeout_ms; }

    int init(I2C_HandleTypeDef* hi2c) {
        if (who_am_i_check(hi2c) != 0) {
            return -1;
        }

        return 0;
    }

    int read_accel_mg(I2C_HandleTypeDef* hi2c, int* x_mg, int* y_mg, int* z_mg, Resolution res, FullScale fs) {
        uint8_t buf[6];
        // Auto-increment read (0x80 | addr)
        if (HAL_I2C_Mem_Read(hi2c, LIS3DH_ADDR, LIS3DH_REG_OUT_X_L | 0x80, 1,
                             buf, 6, I2C_TIMEOUT_MS) != HAL_OK) {
            return -1;
        }

        auto raw_x = static_cast<int16_t>(buf[1] << 8 | buf[0]);
        auto raw_y = static_cast<int16_t>(buf[3] << 8 | buf[2]);
        auto raw_z = (int16_t) (buf[5] << 8 | buf[4]);

        // move left-justified data to right-justified of proper resolution
        int shift;
        switch (res) {
            case Resolution::LOW_POWER_8_BIT:
                shift = 8;
                break;
            case Resolution::NORMAL_10_BIT:
                shift = 6; // 10-bit data in high 10 bits
                break;
            case Resolution::HIGH_RESOLUTION_12_BIT:
                shift = 4; // 12-bit data in high 12 bits
                break;
            default:
                shift = 6;
        }
        raw_x >>= shift;
        raw_y >>= shift;
        raw_z >>= shift;

        int sensitivity = SENSITIVITY[static_cast<int>(fs)][static_cast<int>(res)];
        *x_mg = raw_x * sensitivity;
        *y_mg = raw_y * sensitivity;
        *z_mg = raw_z * sensitivity;

        return 0;
    }

    int who_am_i_check(I2C_HandleTypeDef* hi2c) {
        uint8_t who_am_i;
        if (HAL_I2C_Mem_Read(hi2c, LIS3DH_ADDR, LIS3DH_REG_WHO_AM_I, 1,
                             &who_am_i, 1, I2C_TIMEOUT_MS) == HAL_OK) {
            if (who_am_i == 0x33) {
                return 0;
            }
        }
        return -1;
    }

    // CTRL_REG1
    int write_ctrl_reg1(I2C_HandleTypeDef* hi2c, uint8_t ctrl_reg1) {
        return write_register(hi2c, LIS3DH_REG_CTRL_REG1, ctrl_reg1);
    }
    uint8_t ctrl_reg1_odr(int data_rate_hz) {
        switch (data_rate_hz) {
            case 1:
                return 0x10; // 1Hz
            case 10:
                return 0x20; // 10Hz
            case 25:
                return 0x30; // 25Hz
            case 50:
                return 0x40; // 50Hz
            case 100:
                return 0x50; // 100Hz
            case 200:
                return 0x60; // 200Hz
            case 400:
                return 0x70; // 400Hz
            case 1600:
                return 0x80; // 1600Hz
            case 1344:       // 1344Hz (low power mode)
            case 5476:       // 5.376kHz (high-resolution/normal mode)
                return 0x90;
            default:
                return 0x00; // power-down mode
        }
    }
    uint8_t ctrl_reg1_low_power_mode(bool enabled) {
        return enabled ? (1 << 3) : 0;
    }
    uint8_t ctrl_reg1_z_axis(bool enabled) {
        return enabled ? (1 << 2) : 0;
    }
    uint8_t ctrl_reg1_y_axis(bool enabled) {
        return enabled ? (1 << 1) : 0;
    }
    uint8_t ctrl_reg1_x_axis(bool enabled) {
        return enabled ? (1 << 0) : 0;
    }

    // INT1_CFG
    int write_int1_cfg(I2C_HandleTypeDef* hi2c, uint8_t int1_cfg) {
        return write_register(hi2c, LIS3DH_INT1_CFG, int1_cfg);
    }
    uint8_t int1_cfg_aoi(bool enabled) {
        return enabled ? (1 << 7) : 0;
    }
    uint8_t int1_cfg_6d(bool enabled) {
        return enabled ? (1 << 6) : 0;
    }
    uint8_t int1_cfg_zhie(bool enabled) {
        return enabled ? (1 << 5) : 0;
    }
    uint8_t int1_cfg_zlie(bool enabled) {
        return enabled ? (1 << 4) : 0;
    }
    uint8_t int1_cfg_yhie(bool enabled) {
        return enabled ? (1 << 3) : 0;
    }
    uint8_t int1_cfg_ylie(bool enabled) {
        return enabled ? (1 << 2) : 0;
    }
    uint8_t int1_cfg_xhie(bool enabled) {
        return enabled ? (1 << 1) : 0;
    }
    uint8_t int1_cfg_xlie(bool enabled) {
        return enabled ? (1 << 0) : 0;
    }

    // INT2_CFG
    int write_int2_cfg(I2C_HandleTypeDef* hi2c, uint8_t int2_cfg) {
        return write_register(hi2c, LIS3DH_INT2_CFG, int2_cfg);
    }
    uint8_t int2_cfg_aoi(bool enabled) {
        return enabled ? (1 << 7) : 0;
    }
    uint8_t int2_cfg_6d(bool enabled) {
        return enabled ? (1 << 6) : 0;
    }
    uint8_t int2_cfg_zhie(bool enabled) {
        return enabled ? (1 << 5) : 0;
    }
    uint8_t int2_cfg_zlie(bool enabled) {
        return enabled ? (1 << 4) : 0;
    }
    uint8_t int2_cfg_yhie(bool enabled) {
        return enabled ? (1 << 3) : 0;
    }
    uint8_t int2_cfg_ylie(bool enabled) {
        return enabled ? (1 << 2) : 0;
    }
    uint8_t int2_cfg_xhie(bool enabled) {
        return enabled ? (1 << 1) : 0;
    }
    uint8_t int2_cfg_xlie(bool enabled) {
        return enabled ? (1 << 0) : 0;
    }

    // INT2_THS
    int write_int2_ths(I2C_HandleTypeDef* hi2c, uint8_t int2_ths) {
        return write_register(hi2c, 0x36, int2_ths);
    }
    uint8_t int2_ths_threshold_mg(int threshold_mg, FullScale fs) {
        int mg_per_lsb;
        switch (fs) {
            case FullScale::FS_2G:
                mg_per_lsb = 16;
                break;
            case FullScale::FS_4G:
                mg_per_lsb = 32;
                break;
            case FullScale::FS_8G:
                mg_per_lsb = 62;
                break;
            case FullScale::FS_16G:
                mg_per_lsb = 186;
                break;
            default:
                mg_per_lsb = 16;
        }
        int threshold_lsb = threshold_mg / mg_per_lsb;
        if (threshold_lsb < 0) threshold_lsb = 0;
        if (threshold_lsb > 127) threshold_lsb = 127; // 7-bit value
        return (uint8_t) threshold_lsb;
    }

} // namespace lis3dh
