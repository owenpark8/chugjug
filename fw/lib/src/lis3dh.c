#include "lis3dh.h"

#define LIS3DH_REG_WHO_AM_I   0x0F
#define LIS3DH_REG_CTRL_REG1  0x20
#define LIS3DH_REG_OUT_X_L    0x28

int lis3dh_init(I2C_HandleTypeDef *hi2c)
{
    uint8_t who_am_i;
    if (HAL_I2C_Mem_Read(hi2c, LIS3DH_ADDR, LIS3DH_REG_WHO_AM_I, 1, &who_am_i, 1, 100) != HAL_OK) {
        return -1;
    }
    if (who_am_i != 0x33) {
        return -1;
    }

    // Enable all axes, 100Hz
    uint8_t ctrl1 = 0x57;
    if (HAL_I2C_Mem_Write(hi2c, LIS3DH_ADDR, LIS3DH_REG_CTRL_REG1, 1, &ctrl1, 1, 100) != HAL_OK) {
        return -1;
    }

    return 0;
}

int lis3dh_read_accel(I2C_HandleTypeDef *hi2c, int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buf[6];
    // Auto-increment read (0x80 | addr)
    if (HAL_I2C_Mem_Read(hi2c, LIS3DH_ADDR, LIS3DH_REG_OUT_X_L | 0x80, 1, buf, 6, 100) != HAL_OK) {
        return -1;
    }

    *x = (int16_t)(buf[1] << 8 | buf[0]);
    *y = (int16_t)(buf[3] << 8 | buf[2]);
    *z = (int16_t)(buf[5] << 8 | buf[4]);

    return 0;
}
