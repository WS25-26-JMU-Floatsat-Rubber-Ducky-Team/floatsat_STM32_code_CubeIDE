#include "main.h"
#include "imu.h"
#include "attitude_estimator.h"

#define SPI_FRAME_LEN 6   // 10 bytes (80 bits)

#define READ_RAW_ACC 1
#define READ_RAW_GYRO 2
#define READ_RAW_MAG 3
#define READ_FILT_ACC 4
#define READ_FILT_GYRO 5
#define READ_FILT_MAG 6
#define READ_ACTUAL_ORIENTATION 7


typedef struct COM {
	SPI_HandleTypeDef *hspi;
	uint8_t *spi_rx_buf;
	uint8_t *spi_tx_buf;
} COM_t;


void comunicate(COM_t *com, IMU_t *imu, measurement_t *meas);

