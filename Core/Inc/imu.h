#ifndef __IMU_H
#define __IMU_H

#include "main.h"

#define LSM9DS1_AG_ADDR_READ  0xD7
#define LSM9DS1_M_ADDR_READ   0x3D
#define LSM9DS1_AG_ADDR_WRITE  0xD6
#define LSM9DS1_M_ADDR_WRITE   0x3C
#define CTRL_REG1_G   0x10
#define OUT_G 0x18
#define OUT_A 0x28
#define WHO_AM_I 0x0F
#define OUT_M 0x28
#define CTRL_REG1_M   0x20
#define CTRL_REG2_M   0x21
#define CTRL_REG3_M   0x22

#define IMU_BUFF_LEN 6

typedef struct IMU {
	I2C_HandleTypeDef *hi2c;
	uint8_t *acc_buff;
	uint8_t *gyro_buff;
	uint8_t *mag_buff;
} IMU_t;


void init_imu(IMU_t *imu);
HAL_StatusTypeDef read_imu(IMU_t *imu);
HAL_StatusTypeDef acc_gyro_read(IMU_t *imu, uint8_t reg, uint8_t *buf, uint16_t len);
HAL_StatusTypeDef mag_read(IMU_t *imu, uint8_t reg, uint8_t *buf, uint16_t len);


#endif
