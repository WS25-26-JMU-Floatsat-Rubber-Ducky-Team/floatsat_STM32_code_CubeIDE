#include "imu.h"

HAL_StatusTypeDef acc_gyro_write(IMU_t *imu, uint8_t reg, uint8_t value) {
	return HAL_I2C_Mem_Write(imu->hi2c, LSM9DS1_AG_ADDR_WRITE, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 10);
}

HAL_StatusTypeDef mag_write(IMU_t *imu, uint8_t reg, uint8_t value) {
	return HAL_I2C_Mem_Write(imu->hi2c, LSM9DS1_M_ADDR_WRITE, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 10);
}

HAL_StatusTypeDef acc_gyro_read(IMU_t *imu, uint8_t reg, uint8_t *buf, uint16_t len) {
	return HAL_I2C_Mem_Read(imu->hi2c, LSM9DS1_AG_ADDR_READ, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 10);
}

HAL_StatusTypeDef mag_read(IMU_t *imu, uint8_t reg, uint8_t *buf, uint16_t len) {
	return HAL_I2C_Mem_Read(imu->hi2c, LSM9DS1_M_ADDR_READ, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 10);
}

int imu_counter = 0;
HAL_StatusTypeDef read_imu(IMU_t *imu){
	imu_counter += 1;
	if (imu_counter > 3) imu_counter = 0;
	switch (imu_counter) {
		case 1:
			return acc_gyro_read(imu, OUT_A, imu->acc_buff, IMU_BUFF_LEN);
		case 2:
			return acc_gyro_read(imu, OUT_G, imu->gyro_buff, IMU_BUFF_LEN);
		case 3:
			//mag_read(imu, OUT_M, imu->mag_buff, IMU_BUFF_LEN);
			break;
		case 0:
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			break;
		default:
			break;
	}
	return HAL_OK;
}


//void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
//	// TODO probably needs to be in main and call a function here. -> pass reference to imu
//}


void init_imu(IMU_t *imu) {
	acc_gyro_write(imu, CTRL_REG1_G, 0xcb);
	mag_write(imu, CTRL_REG2_M, 0x04);  // SOFT_RST
	mag_write(imu, CTRL_REG1_M, 0xfc);
	mag_write(imu, CTRL_REG3_M, 0x00);
}
