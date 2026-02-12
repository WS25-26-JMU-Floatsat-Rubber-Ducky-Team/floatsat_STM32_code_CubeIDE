#ifndef __IMU_H
#define __IMU_H

#include "main.h"
#include "attitude_types.h"

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
#define GYRO_SAMPLES 10

typedef struct IMU {
	I2C_HandleTypeDef *hi2c;
	uint8_t *acc_buff;
	uint8_t *gyro_buff;
	uint8_t *mag_buff;
    vec3_t gyro;
    vec3_t acc;
    vec3_t mag;
	float gyro_x_bias;
	float gyro_y_bias;
	float gyro_z_bias;
} IMU_t;

typedef struct {
	float dt;
	float df; // = 1/dt
	float alpha; // between 0 and 1!
	float beta;  // between 0 and 1!
	float x;
	float v;
} FILTERS_AB;

void filter_ab(FILTERS_AB *ab, float in);


void init_imu(IMU_t *imu);
void parse_imu(IMU_t *imu);
HAL_StatusTypeDef calib_imu(IMU_t *imu);
HAL_StatusTypeDef read_imu(IMU_t *imu);
HAL_StatusTypeDef acc_gyro_read(IMU_t *imu, uint8_t reg, uint8_t *buf, uint16_t len);
HAL_StatusTypeDef mag_read(IMU_t *imu, uint8_t reg, uint8_t *buf, uint16_t len);


#endif
