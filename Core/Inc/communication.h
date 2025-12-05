#include "main.h"
#include "imu.h"

#define SPI_FRAME_LEN 6   // 10 bytes (80 bits)

#define READ_RAW_ACC 1
#define READ_RAW_GYRO 2
#define READ_RAW_MAG 3


typedef struct COM {
	SPI_HandleTypeDef *hspi;
	uint8_t *spi_rx_buf[SPI_FRAME_LEN];
	uint8_t *spi_tx_buf[SPI_FRAME_LEN];
} COM_t;


void comunicate(COM_t *com, IMU_t *imu);

