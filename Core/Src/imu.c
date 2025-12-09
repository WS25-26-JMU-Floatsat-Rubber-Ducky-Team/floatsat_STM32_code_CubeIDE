#include "imu.h"

HAL_StatusTypeDef acc_gyro_write(IMU_t *imu, uint8_t reg, uint8_t value) {
	return HAL_I2C_Mem_Write_IT(imu->hi2c, LSM9DS1_AG_ADDR_WRITE, reg, I2C_MEMADD_SIZE_8BIT, &value, 1);
}

HAL_StatusTypeDef mag_write(IMU_t *imu, uint8_t reg, uint8_t value) {
	return HAL_I2C_Mem_Write_IT(imu->hi2c, LSM9DS1_M_ADDR_WRITE, reg, I2C_MEMADD_SIZE_8BIT, &value, 1);
}

HAL_StatusTypeDef acc_gyro_read(IMU_t *imu, uint8_t reg, uint8_t *buf, uint16_t len) {
	return HAL_I2C_Mem_Read_IT(imu->hi2c, LSM9DS1_AG_ADDR_READ, reg, I2C_MEMADD_SIZE_8BIT, buf, len);
}

HAL_StatusTypeDef mag_read(IMU_t *imu, uint8_t reg, uint8_t *buf, uint16_t len) {
	return HAL_I2C_Mem_Read_IT(imu->hi2c, LSM9DS1_M_ADDR_READ, reg, I2C_MEMADD_SIZE_8BIT, buf, len);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	// TODO probably needs to be in main and call a function here. -> pass reference to imu
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	// TODO probably needs to be in main and call a function here -> pass reference to imu
}

void init_imu(IMU_t *imu) {
	acc_gyro_write(imu, CTRL_REG1_G, 0xcb);
	HAL_Delay(10);
	mag_write(imu, CTRL_REG1_M, 0xfc);
	HAL_Delay(10);
	mag_write(imu, CTRL_REG3_M, 0x00);
}

