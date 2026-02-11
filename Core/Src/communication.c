#include "communication.h"
#include "imu.h"

void comunicate(COM_t *com, IMU_t *imu, measurement_t *meas, control_params_t *params, float *omega_body_z_cmd) {
	switch ((int)com->spi_rx_buf[0]) {
	case READ_RAW:
		for (int i = 0; i < IMU_BUFF_LEN; i++) com->spi_tx_buf[i] = imu->acc_buff[i];
		for (int i = 0; i < IMU_BUFF_LEN; i++) com->spi_tx_buf[i+IMU_BUFF_LEN] = imu->gyro_buff[i];
		for (int i = 0; i < IMU_BUFF_LEN; i++) com->spi_tx_buf[i+(2*IMU_BUFF_LEN)] = imu->mag_buff[i];
		break;
	case READ_FILT:
		*((int16_t *)(&(com->spi_tx_buf[0]))) = (int16_t)(imu->acc.v[0]*1000);  // g
		*((int16_t *)(&(com->spi_tx_buf[2]))) = (int16_t)(imu->acc.v[1]*1000);
		*((int16_t *)(&(com->spi_tx_buf[4]))) = (int16_t)(imu->acc.v[2]*1000);

		*((int16_t *)(&(com->spi_tx_buf[6]))) = (int16_t)(imu->gyro.v[0]*100);  // dps
		*((int16_t *)(&(com->spi_tx_buf[8]))) = (int16_t)(imu->gyro.v[1]*100);
		*((int16_t *)(&(com->spi_tx_buf[10]))) = (int16_t)(imu->gyro.v[2]*100);

		*((int16_t *)(&(com->spi_tx_buf[12]))) = (int16_t)(imu->mag.v[0]*100);  // uT
		*((int16_t *)(&(com->spi_tx_buf[14]))) = (int16_t)(imu->mag.v[1]*100);
		*((int16_t *)(&(com->spi_tx_buf[16]))) = (int16_t)(imu->mag.v[2]*100);
		break;
	case WRITE_SETTINGS:
		params->rate_kp[0] = ((float)com->spi_rx_buf[1]) / 100.0f; // 'pitchP'
		params->rate_ki[0] = ((float)com->spi_rx_buf[2]) / 1000.0f; // pitchI
		params->rate_kd[0] = ((float)com->spi_rx_buf[3]) / 100.0f; // pitchD
		params->rate_kp[1] = ((float)com->spi_rx_buf[4]) / 100.0f; // rollP
		params->rate_ki[1] = ((float)com->spi_rx_buf[5]) / 1000.0f; // rollI
		params->rate_kd[1] = ((float)com->spi_rx_buf[6]) / 100.0f; // rollD
		params->rate_kp[2] = ((float)com->spi_rx_buf[7]) / 100.0f; // yawP
		params->rate_ki[2] = ((float)com->spi_rx_buf[8]) / 1000.0f; // yawI
		params->rate_kd[2] = ((float)com->spi_rx_buf[9]) / 100.0f; // yawD

		/*
		= ((float)com->spi_rx_buf[10]) / 100.0f; // magnetometerWeight
		= ((float)com->spi_rx_buf[11]) / 100.0f; // gyroscopeWeight
		= ((float)com->spi_rx_buf[12]) / 100.0f; // accelerometerWeight
		= ((float)com->spi_rx_buf[13]) / 10.0f; // maxTiltAngle
		*/
		*omega_body_z_cmd = (((float)com->spi_rx_buf[14])-128.0f) * (6.28f / 128.0f);
		// break; // fall through to answer with useful data.
	case READ_ACTUAL_ORIENTATION:
		*((int16_t *)(&(com->spi_tx_buf[0]))) = (int16_t)(meas->q.w*10000);
		*((int16_t *)(&(com->spi_tx_buf[2]))) = (int16_t)(meas->q.x*10000);
		*((int16_t *)(&(com->spi_tx_buf[4]))) = (int16_t)(meas->q.y*10000);
		*((int16_t *)(&(com->spi_tx_buf[6]))) = (int16_t)(meas->q.z*10000);
		break;
	default:
		break;
	}

	HAL_SPI_TransmitReceive_IT(com->hspi, com->spi_tx_buf, com->spi_rx_buf, SPI_FRAME_LEN);
}
