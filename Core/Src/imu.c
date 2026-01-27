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


void parse_imu(IMU_t *imu) {
	int16_t raw_accel_x = (((uint16_t)imu->acc_buff[0]) << 8) + (uint16_t)imu->acc_buff[1];
	int16_t raw_accel_y = (((uint16_t)imu->acc_buff[2]) << 8) + (uint16_t)imu->acc_buff[3];
	int16_t raw_accel_z = (((uint16_t)imu->acc_buff[4]) << 8) + (uint16_t)imu->acc_buff[5];

	int16_t raw_gyro_x = (((uint16_t)imu->gyro_buff[0]) << 8) + (uint16_t)imu->gyro_buff[1];
	int16_t raw_gyro_y = (((uint16_t)imu->gyro_buff[2]) << 8) + (uint16_t)imu->gyro_buff[3];
	int16_t raw_gyro_z = (((uint16_t)imu->gyro_buff[4]) << 8) + (uint16_t)imu->gyro_buff[5];

	float accel_x = ((float)raw_accel_x) * 0.000061; // mg/LSB to g
	float accel_y = ((float)raw_accel_y) * 0.000061; // mg/LSB to g
	float accel_z = ((float)raw_accel_z) * 0.000061; // mg/LSB to g
	float gyro_x = ((float)raw_gyro_x) * 0.01750; // mdps / LSB to dps
	float gyro_y = ((float)raw_gyro_y) * 0.01750; // mdps / LSB to dps
	float gyro_z = ((float)raw_gyro_z) * 0.01750; // mdps / LSB to dps

	// TODO put values in some struct!

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
