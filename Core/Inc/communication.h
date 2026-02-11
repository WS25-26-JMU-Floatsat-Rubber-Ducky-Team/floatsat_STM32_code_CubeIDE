#include "main.h"
#include "imu.h"
#include "attitude_estimator.h"
#include "attitude_control.h"

#define SPI_FRAME_LEN 18   // 10 bytes (80 bits)

#define READ_RAW 1
#define READ_FILT 2
#define READ_ACTUAL_ORIENTATION 3
#define WRITE_SETTINGS 4


typedef struct COM {
	SPI_HandleTypeDef *hspi;
	uint8_t *spi_rx_buf;
	uint8_t *spi_tx_buf;
} COM_t;


void comunicate(COM_t *com, IMU_t *imu, measurement_t *meas, control_params_t *params, float *omega_body_z_cmd);

