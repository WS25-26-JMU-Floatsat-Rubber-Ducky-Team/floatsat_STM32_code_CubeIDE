/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lsm9ds1_reg.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
TIM_HandleTypeDef htim2;   // step timer

#define SPI_FRAME_LEN 6   // 10 bytes (80 bits)
#define LSM9DS1_AG_ADDR_READ  0xD7
#define LSM9DS1_M_ADDR_READ   0x3D
#define LSM9DS1_AG_ADDR_WRITE  0xD6
#define LSM9DS1_M_ADDR_WRITE   0x3C
#define CTRL_REG1_G   0x10
#define OUT_G 0x18
#define OUT_A 0x28
#define WHO_AM_I 0x0F
#define OUT_M 0x28
#define CTRL_REG1_M   0x20
#define CTRL_REG3_M   0x22

uint8_t spi_rx_buf[SPI_FRAME_LEN];
uint8_t spi_tx_buf[SPI_FRAME_LEN];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
HAL_StatusTypeDef imu_write_reg(uint16_t devAddr, uint8_t reg, uint8_t value);
HAL_StatusTypeDef imu_read_regs(uint16_t devAddr, uint8_t reg, uint8_t *buf,
		uint16_t len);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static volatile uint8_t g_step = 1;  // 1..6
static volatile uint32_t g_stepCounter = 0;
volatile uint8_t rx_byte = 0;

static inline void set_abc(uint8_t A, uint8_t B, uint8_t C) {
	// Build BSRR: lower 16 bits = set, upper 16 bits = reset
	uint32_t set = 0;
	uint32_t rst = 0;

	(A ? (set |= GPIO_PIN_12) : (rst |= GPIO_PIN_12));
	(B ? (set |= GPIO_PIN_13) : (rst |= GPIO_PIN_13));
	(C ? (set |= GPIO_PIN_14) : (rst |= GPIO_PIN_14));

	GPIOB->BSRR = (set) | ((uint32_t) rst << 16);
}

// ------------ Required function ------------
void Commutation_Step(uint8_t step) {
	switch (step) {
	case 1:
		set_abc(1, 0, 0);
		break; // HLL
	case 2:
		set_abc(1, 1, 0);
		break; // HHL
	case 3:
		set_abc(0, 1, 0);
		break; // LHL
	case 4:
		set_abc(0, 1, 1);
		break; // LHH
	case 5:
		set_abc(0, 0, 1);
		break; // LLH
	case 6:
		set_abc(1, 0, 1);
		break; // HLH
	default: // step 7
	case 7:
		set_abc(0, 0, 0);
		break; // LLL
	}
}

// ------------ Public API ------------
void Commutation_Start(uint32_t step_hz); // start fixed-frequency commutation
void Commutation_Stop(void);              // stop stepping (keeps last state)

static void MX_GPIOB_Phases_Init(void) {
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef gi = { 0 };
	gi.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
	gi.Mode = GPIO_MODE_OUTPUT_PP;
	gi.Pull = GPIO_NOPULL;
	gi.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &gi);

	// default to all low
	set_abc(0, 0, 0);
}

static void MX_TIM3_PWM_CH2_Init_5pct(void) {
	/* 1) Enable clocks */
	__HAL_RCC_TIM3_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* 2) GPIO config: PB5 -> TIM3_CH2 (AF2) */
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;   // check AF for your MCU
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* 3) TIM3 base config */
	// 16 MHz / (PSC+1) / (ARR+1) = 20 kHz
	// PSC = 0 -> 16 MHz
	// ARR = 799 -> 16 MHz / 800 = 20 kHz
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 799;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_PWM_Init(&htim3);

	/* 4) PWM channel config: 5% duty on CH2 */
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = (htim3.Init.Period + 1) * 5 / 100; // 5% of 800 = 40 set duty
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);

	/* 5) Start PWM */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

static void MX_TIM2_StepTimer_Init(uint32_t step_hz) {
	__HAL_RCC_TIM2_CLK_ENABLE();

	const uint32_t timer_clk_hz = 16000000UL;
	uint32_t psc = 1599;                           // 16 MHz / (1599+1) = 10 kHz
	uint32_t tick = timer_clk_hz / (psc + 1);              // = 10 000

	if (step_hz == 0)
		step_hz = 1;                         // avoid div-by-zero

	uint32_t arr = (tick / step_hz);
	if (arr == 0)
		arr = 1;                                 // limit max frequency
	else
		arr -= 1;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = psc;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = arr;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&htim2);

	HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/*static void MX_LED_Init(void) {
 __HAL_RCC_GPIOC_CLK_ENABLE();

 GPIO_InitTypeDef gi = { 0 };
 gi.Pin = GPIO_PIN_13;
 gi.Mode = GPIO_MODE_OUTPUT_PP;
 gi.Pull = GPIO_NOPULL;
 gi.Speed = GPIO_SPEED_FREQ_LOW;
 HAL_GPIO_Init(GPIOC, &gi);

 // Default LED OFF (PC13 = HIGH)
 //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
 }*/

void Commutation_Start(uint32_t step_hz) {
	MX_GPIOB_Phases_Init();
	MX_TIM3_PWM_CH2_Init_5pct();   // enable driver at 5% duty
	MX_TIM2_StepTimer_Init(step_hz);
	//MX_LED_Init();

	g_step = 1;
	Commutation_Step(g_step);

	__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
	HAL_TIM_Base_Start_IT(&htim2);
}

void Commutation_Stop(void) {
	HAL_TIM_Base_Stop_IT(&htim2);
	// keep last commutation state; if you want to de-energize:
	set_abc(0, 0, 0);
}

void TIM2_IRQHandler(void) {
	if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET) {
		if (__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_UPDATE) != RESET) {
			__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);

			// advance commutation step
			g_step++;
			if (g_step > 6)
				g_step = 1;
			Commutation_Step(g_step);

			// --- LED toggle every 600 steps ---
			/*g_stepCounter++;
			 if (g_stepCounter >= 600) {
			 g_stepCounter = 0;
			 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // toggle LED (active-low)
			 }*/
		}
	}
}

void Commutation_SetFrequency(uint32_t step_hz) {
	if (step_hz < 1)
		step_hz = 1;

	uint32_t timer_clk = 16000000UL;     // 16 MHz
	uint32_t psc = 1599;           // same as in MX_TIM2_StepTimer_Init
	uint32_t tick = timer_clk / (psc + 1); // 10 000 Hz

	uint32_t arr = tick / step_hz;
	if (arr == 0)
		arr = 1;
	else
		arr -= 1;

	__HAL_TIM_SET_AUTORELOAD(&htim2, arr);
	__HAL_TIM_SET_COUNTER(&htim2, 0);

}

void SetDuty_TIM3_CH2(uint8_t duty_percent) {
	if (duty_percent > 95)
		duty_percent = 95;
	if (duty_percent < 50)
		duty_percent = 50;

	uint32_t arr = htim3.Init.Period + 1;     // ARR+1 is timer resolution
	uint32_t pulse = (arr * duty_percent) / 100;

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse);
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM3_Init();
	MX_SPI1_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	for (int i = 0; i < SPI_FRAME_LEN; i++) {
		spi_rx_buf[i] = 0;
	}
	for (int i = 0; i < SPI_FRAME_LEN; i++) {
		spi_tx_buf[i] = i + 1;
	}

	float rps = 1;         // start speed
	float target_rps = 17;   // end speed
	float step = 0.05;      // how much to increase per loop
	uint32_t delay_ms_motor1 = 50; // how fast to ramp
	uint32_t delay_ms_imu = 50;
	Commutation_Start(rps);
	uint32_t last_tick_motor = HAL_GetTick();
	uint32_t last_tick_imu = HAL_GetTick();
	uint32_t now = HAL_GetTick();

	//HAL_SPI_Receive_IT(&hspi1, spi_rx_buf, SPI_FRAME_LEN); // wait for first 10 bytes
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	HAL_SPI_TransmitReceive_IT(&hspi1, spi_tx_buf, spi_rx_buf, SPI_FRAME_LEN);

	/*
	 Commutation_Start(1 * 7 * 6);  // 1 RPS
	 HAL_Delay(1000);
	 Commutation_Start(2 * 7 * 6);  // 2 RPS
	 HAL_Delay(1000);
	 Commutation_Start(3 * 7 * 6);  // 3 RPS
	 HAL_Delay(1000);
	 Commutation_Start(4 * 7 * 6);  // 4 RPS
	 HAL_Delay(1000);
	 Commutation_Start(5 * 7 * 6);  // 5 RPS
	 */

	imu_write_reg(LSM9DS1_AG_ADDR_WRITE, CTRL_REG1_G, 0xcb);
	imu_write_reg(LSM9DS1_M_ADDR_WRITE, CTRL_REG1_M, 0xfc);
	imu_write_reg(LSM9DS1_M_ADDR_WRITE, CTRL_REG3_M, 0x00);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (rps < target_rps) {
			now = HAL_GetTick(); // current time in ms

			if ((now - last_tick_motor) >= delay_ms_motor1) {
				last_tick_motor = now;  // move to next slot

				rps += step;

				float comm_freq_f = rps * 7.0f * 6.0f; // rps → electrical steps per sec
				//float duty = rps * 3.90625 + 70; // rps * 60 to get rpm, then /128 because of kv rating, then /12 to get duty and then *100%

				uint32_t comm_freq = (uint32_t) comm_freq_f;

				if (comm_freq < 1)
					comm_freq = 1;

				Commutation_SetFrequency(comm_freq);

				float duty = 98.0f;
				SetDuty_TIM3_CH2((uint8_t) duty);
			}

		}

		if ((HAL_GetTick() - last_tick_imu) >= delay_ms_imu) {
			last_tick_imu = HAL_GetTick();
			//imu_read_regs(LSM9DS1_AG_ADDR_READ, OUT_A, spi_tx_buf, 6);
			imu_read_regs(LSM9DS1_M_ADDR_READ, OUT_M, spi_tx_buf, 6);
			//imu_read_regs(LSM9DS1_M_ADDR_READ, CTRL_REG1_M, spi_tx_buf, 6);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */
	/*__HAL_RCC_GPIOB_CLK_ENABLE();

	 GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	 GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;  // example SCL/SDA
	 GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	 GPIO_InitStruct.Pull = GPIO_PULLUP;
	 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	 GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	 HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	 __HAL_RCC_I2C1_CLK_ENABLE();*/
	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_SLAVE;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */
	HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);   // priority 5 is fine
	HAL_NVIC_EnableIRQ(SPI1_IRQn);           // <<< CRITICAL
	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_8 | GPIO_PIN_9,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
	GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_RESET);

	/*Configure GPIO pins : PC13 PC8 PC9 */
	GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PB12 PB13 PB14 */
	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PA8 PA9 PA10 PA11 */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	// Configure PA5 (SCK), PA6 (MISO), PA7 (MOSI) for SPI1 (AF5)
	GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;            // or GPIO_PULLDOWN if needed
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
 if (hspi->Instance == SPI1) {
 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
 // We just received 10 bytes in spi_rx_buf[]

 // Example: echo them back → copy RX to TX buffer
 for (uint8_t i = 0; i < SPI_FRAME_LEN; i++) {
 spi_tx_buf[i] = spi_rx_buf[i];
 //spi_tx_buf[i] = 1;
 }
 // Now send those 10 bytes back to the master
 // (master must start a new SPI transaction and provide clock)
 HAL_StatusTypeDef status = HAL_SPI_Transmit_IT(&hspi1, spi_tx_buf,
 SPI_FRAME_LEN);
 (void) status; // optionally check for HAL_OK
 }
 }

 void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
 if (hspi->Instance == SPI1) {
 //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
 // Finished sending 10 bytes back.
 // Arm the SPI again to wait for the next 10-byte frame.
 HAL_SPI_Receive_IT(&hspi1, spi_rx_buf, SPI_FRAME_LEN);
 }
 }*/

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI1) {
		// Indicate transfer completed

		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		/*for (int i = 0; i < SPI_FRAME_LEN; i++) {
		 spi_tx_buf[i] = spi_rx_buf[i];
		 }*/
		// Start the next transaction
		HAL_SPI_TransmitReceive_IT(&hspi1, spi_tx_buf, spi_rx_buf,
		SPI_FRAME_LEN);
	}
}

HAL_StatusTypeDef imu_write_reg(uint16_t devAddr, uint8_t reg, uint8_t value) {
	return HAL_I2C_Mem_Write(&hi2c1, devAddr, reg,
	I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

HAL_StatusTypeDef imu_read_regs(uint16_t devAddr, uint8_t reg, uint8_t *buf,
		uint16_t len) {
	return HAL_I2C_Mem_Read(&hi2c1, devAddr, reg,
	I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
