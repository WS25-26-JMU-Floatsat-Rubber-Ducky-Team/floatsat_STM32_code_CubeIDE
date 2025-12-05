#include "communication.h"
#include "imu.h"


void comunicate(COM_t *com, IMU_t *imu) {
	switch ((int)com->spi_rx_buf[0]) {
	case READ_RAW_ACC:
		mag_read(imu, OUT_A, com->spi_tx_buf, 6);
		break;
	case READ_RAW_GYRO:
		mag_read(imu, OUT_G, com->spi_tx_buf, 6);
		break;
	case READ_RAW_MAG:
		mag_read(imu, OUT_M, com->spi_tx_buf, 6);
		break;
	default:
		break;
	}


	HAL_SPI_TransmitReceive_IT(com->hspi, com->spi_tx_buf, com->spi_rx_buf, SPI_FRAME_LEN);
}
