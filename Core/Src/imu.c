#include "imu.h"


FILTERS_AB x_rate_filter = {0.002, 1/0.002, 0.9f, 0.5f, 0.0f, 0.0f};
FILTERS_AB y_rate_filter = {0.002, 1/0.002, 0.9f, 0.5f, 0.0f, 0.0f};
FILTERS_AB z_rate_filter = {0.002, 1/0.002, 0.9f, 0.5f, 0.0f, 0.0f};


// see https://en.wikipedia.org/wiki/Alpha_beta_filter
void filter_ab(FILTERS_AB *ab, float in) {
	float x = ab->x + (ab->v * ab->dt);
	float v = ab->v;
	float r = in - x;

	x += ab->alpha * r;
	v += ab->beta * r * ab->df;

	ab->x = x;
	ab->v = v;
}



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
			return mag_read(imu, OUT_M, imu->mag_buff, IMU_BUFF_LEN);
		case 0:
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			break;
		default:
			break;
	}
	return HAL_OK;
}


void parse_imu(IMU_t *imu) {
	int16_t raw_accel_x = (uint16_t)imu->acc_buff[0] + (((uint16_t)imu->acc_buff[1]) << 8);
	int16_t raw_accel_y = (uint16_t)imu->acc_buff[2] + (((uint16_t)imu->acc_buff[3]) << 8);
	int16_t raw_accel_z = (uint16_t)imu->acc_buff[4] + (((uint16_t)imu->acc_buff[5]) << 8);

	int16_t raw_gyro_x = (uint16_t)imu->gyro_buff[0] + (((uint16_t)imu->gyro_buff[1]) << 8);
	int16_t raw_gyro_y = (uint16_t)imu->gyro_buff[2] + (((uint16_t)imu->gyro_buff[3]) << 8);
	int16_t raw_gyro_z = (uint16_t)imu->gyro_buff[4] + (((uint16_t)imu->gyro_buff[5]) << 8);

	int16_t raw_mag_x = (uint16_t)imu->mag_buff[0] + (((uint16_t)imu->mag_buff[1]) << 8);
	int16_t raw_mag_y = (uint16_t)imu->mag_buff[2] + (((uint16_t)imu->mag_buff[3]) << 8);
	int16_t raw_mag_z = (uint16_t)imu->mag_buff[4] + (((uint16_t)imu->mag_buff[5]) << 8);

	imu->acc.v[1] = ((float)raw_accel_x) * 0.000061; // mg/LSB to g
	imu->acc.v[0] = ((float)raw_accel_y) * 0.000061; // mg/LSB to g
	imu->acc.v[2] = ((float)raw_accel_z) * 0.000061; // mg/LSB to g

	float gyro_x = ((float)raw_gyro_x) * 0.0175 * 0.1; // mdps / LSB to dps to radps
	float gyro_y = ((float)raw_gyro_y) * 0.0175 * 0.1; // mdps / LSB to dps to radps
	float gyro_z = ((float)raw_gyro_z) * 0.0175 * 0.1; // mdps / LSB to dps to radps

	//filter_ab(&x_rate_filter, gyro_x - imu->gyro_x_bias);
	//filter_ab(&y_rate_filter, gyro_y - imu->gyro_y_bias);
	//filter_ab(&z_rate_filter, gyro_z - imu->gyro_z_bias);
	imu->gyro.v[1] = gyro_x - imu->gyro_x_bias; //-x_rate_filter.x;
	imu->gyro.v[0] = gyro_y - imu->gyro_y_bias; //y_rate_filter.x;
	imu->gyro.v[2] = gyro_z - imu->gyro_z_bias; //z_rate_filter.x;

	imu->mag.v[0] = ((float)raw_mag_x) * 0.014;  // mgauss/LSB to uT
	imu->mag.v[1] = ((float)raw_mag_y) * 0.014;  // mgauss/LSB to uT
	imu->mag.v[2] = ((float)raw_mag_z) * 0.014;  // mgauss/LSB to uT

	imu->mag.v[0] -= -3.689;
	imu->mag.v[1] -= 13.566;
	imu->mag.v[2] -= 0.0;
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


HAL_StatusTypeDef calib_imu(IMU_t *imu) {
	imu->gyro_x_bias = 0.0;
	imu->gyro_y_bias = 0.0;
	imu->gyro_z_bias = 0.0;
	float gyro_x_bias_agreg = 0.0;
	float gyro_y_bias_agreg = 0.0;
	float gyro_z_bias_agreg = 0.0;

	// warm filter
	for (int i = 0; i < 10; i++) {
		if (acc_gyro_read(imu, OUT_G, imu->gyro_buff, IMU_BUFF_LEN) != HAL_OK){
			return HAL_ERROR; // imu faild stop calib
		} else {
			parse_imu(imu);
		}
		HAL_Delay(10);
	}

	// agreg values
	for (int i = 0; i < GYRO_SAMPLES; i++) {
		if (acc_gyro_read(imu, OUT_G, imu->gyro_buff, IMU_BUFF_LEN) != HAL_OK){
			return HAL_ERROR; // imu faild stop calib
		} else {
			parse_imu(imu);
			gyro_x_bias_agreg += imu->gyro.v[0];
			gyro_y_bias_agreg += imu->gyro.v[1];
			gyro_z_bias_agreg += imu->gyro.v[2];
		}
		HAL_Delay(10);
	}
	imu->gyro_x_bias = gyro_x_bias_agreg / GYRO_SAMPLES;
	imu->gyro_y_bias = gyro_y_bias_agreg / GYRO_SAMPLES;
	imu->gyro_z_bias = gyro_z_bias_agreg / GYRO_SAMPLES;
	return HAL_OK;
}


