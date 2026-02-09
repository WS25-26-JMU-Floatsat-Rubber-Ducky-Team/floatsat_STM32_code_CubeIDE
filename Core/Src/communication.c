#include "communication.h"
#include "imu.h"

void comunicate(COM_t *com, IMU_t *imu, measurement_t *meas) {
	switch ((int)com->spi_rx_buf[0]) {
	case READ_RAW_ACC:
		for (int i = 0; i < IMU_BUFF_LEN; i++) com->spi_tx_buf[i] = imu->acc_buff[i];
		break;
	case READ_RAW_GYRO:
		for (int i = 0; i < IMU_BUFF_LEN; i++) com->spi_tx_buf[i] = imu->gyro_buff[i];
		break;
	case READ_RAW_MAG:
		for (int i = 0; i < IMU_BUFF_LEN; i++) com->spi_tx_buf[i] = imu->mag_buff[i];
		break;
	case READ_FILT_ACC:
		*((int16_t *)(&(com->spi_tx_buf[0]))) = (int16_t)(imu->acc.v[0]*1000);  // g
		*((int16_t *)(&(com->spi_tx_buf[2]))) = (int16_t)(imu->acc.v[1]*1000);
		*((int16_t *)(&(com->spi_tx_buf[4]))) = (int16_t)(imu->acc.v[2]*1000);
		break;
	case READ_FILT_GYRO:
		*((int16_t *)(&(com->spi_tx_buf[0]))) = (int16_t)(imu->gyro.v[0]*100);  // dps
		*((int16_t *)(&(com->spi_tx_buf[2]))) = (int16_t)(imu->gyro.v[1]*100);
		*((int16_t *)(&(com->spi_tx_buf[4]))) = (int16_t)(imu->gyro.v[2]*100);
		break;
	case READ_FILT_MAG:
		*((int16_t *)(&(com->spi_tx_buf[0]))) = (int16_t)(imu->mag.v[0]*100);  // uT
		*((int16_t *)(&(com->spi_tx_buf[2]))) = (int16_t)(imu->mag.v[1]*100);
		*((int16_t *)(&(com->spi_tx_buf[4]))) = (int16_t)(imu->mag.v[2]*100);
		break;
	case READ_ACTUAL_ORIENTATION:
		com->spi_tx_buf[0] = (uint8_t)(meas->q.w*128+128);
		com->spi_tx_buf[1] = (uint8_t)(meas->q.x*128+128);
		com->spi_tx_buf[2] = (uint8_t)(meas->q.y*128+128);
		com->spi_tx_buf[3] = (uint8_t)(meas->q.z*128+128);
		com->spi_tx_buf[4] = 0;
		com->spi_tx_buf[5] = 0;
		break;
	default:
		break;
	}

	HAL_SPI_TransmitReceive_IT(com->hspi, com->spi_tx_buf, com->spi_rx_buf, SPI_FRAME_LEN);
}
