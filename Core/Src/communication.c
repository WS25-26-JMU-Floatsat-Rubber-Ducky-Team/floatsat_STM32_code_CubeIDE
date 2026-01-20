#include "communication.h"
#include "imu.h"

void comunicate(COM_t *com, IMU_t *imu) {
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
	default:
		break;
	}

	HAL_SPI_TransmitReceive_IT(com->hspi, com->spi_tx_buf, com->spi_rx_buf, SPI_FRAME_LEN);
}
