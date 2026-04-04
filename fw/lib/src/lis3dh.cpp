#include "lis3dh.hpp"

namespace lis3dh {

    static constexpr uint8_t LIS3DH_REG_WHO_AM_I = 0x0F;
    static constexpr uint8_t LIS3DH_REG_CTRL_REG1 = 0x20;
    static constexpr uint8_t LIS3DH_REG_OUT_X_L = 0x28;
    static constexpr uint8_t LIS3DH_INT1_SRC = 0x31;

    static int I2C_TIMEOUT_MS = 100;

    void set_i2c_timeout(int timeout_ms) { I2C_TIMEOUT_MS = timeout_ms; }

    int init(I2C_HandleTypeDef* hi2c) {
        if (who_am_i_check(hi2c) != 0) {
            return -1;
        }

        return 0;
    }

    int read_accel(I2C_HandleTypeDef* hi2c, int16_t* x, int16_t* y, int16_t* z) {
        uint8_t buf[6];
        // Auto-increment read (0x80 | addr)
        if (HAL_I2C_Mem_Read(hi2c, LIS3DH_ADDR, LIS3DH_REG_OUT_X_L | 0x80, 1,
                             buf, 6, I2C_TIMEOUT_MS) != HAL_OK) {
            return -1;
        }

        *x = (int16_t) (buf[1] << 8 | buf[0]);
        *y = (int16_t) (buf[3] << 8 | buf[2]);
        *z = (int16_t) (buf[5] << 8 | buf[4]);

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

    int write_ctrl_reg1(I2C_HandleTypeDef* hi2c, int data_rate_hz,
                        bool low_power_mode_enabled, bool z_axis_enabled,
                        bool y_axis_enabled, bool x_axis_enabled) {
        uint8_t ctrl1 = 0x00;

        switch (data_rate_hz) {
            case 0:
                ctrl1 |= 0x00;
                break; // power-down mode
            case 1:
                ctrl1 |= 0x10;
                break; // 1Hz
            case 10:
                ctrl1 |= 0x20;
                break; // 10Hz
            case 25:
                ctrl1 |= 0x30;
                break; // 25Hz
            case 50:
                ctrl1 |= 0x40;
                break; // 50Hz
            case 100:
                ctrl1 |= 0x50;
                break; // 100Hz
            case 200:
                ctrl1 |= 0x60;
                break; // 200Hz
            case 400:
                ctrl1 |= 0x70;
                break; // 400Hz
            case 1600:
                ctrl1 |= 0x80;
                break; // 1600Hz
            case 1344: // 1344Hz (low power mode)
            case 5476: // 5.376kHz (high-resolution/normal mode)
                ctrl1 |= 0x90;
                break;
            default:
                return -1; // unsupported data rate
        }

        if (low_power_mode_enabled) {
            ctrl1 |= (1 << 3);
        }
        if (z_axis_enabled) {
            ctrl1 |= (1 << 2);
        }
        if (y_axis_enabled) {
            ctrl1 |= (1 << 1);
        }
        if (x_axis_enabled) {
            ctrl1 |= (1 << 0);
        }

        if (HAL_I2C_Mem_Write(hi2c, LIS3DH_ADDR, LIS3DH_REG_CTRL_REG1, 1,
                              &ctrl1, 1, I2C_TIMEOUT_MS) != HAL_OK) {
            return -1;
        }

        return 0;
    }

    int write_int1_src(I2C_HandleTypeDef* hi2c, uint8_t int1_src) {
        if (HAL_I2C_Mem_Write(hi2c, LIS3DH_ADDR, LIS3DH_INT1_SRC, 1,
                              &int1_src, 1, I2C_TIMEOUT_MS) != HAL_OK) {
            return -1;
        }
        return 0;
    }
} // namespace lis3dh
