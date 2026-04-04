#ifndef LIS3DH_H
#define LIS3DH_H

#include "stm32u5xx_hal.h"

#define LIS3DH_ADDR (0x18 << 1)

int lis3dh_init(I2C_HandleTypeDef *hi2c);
int lis3dh_read_accel(I2C_HandleTypeDef *hi2c, int16_t *x, int16_t *y, int16_t *z);

#endif
