#include "motor.h"
#include "lsm9ds1_reg.h"

static volatile uint8_t g_step = 1;  // 1..6
static volatile uint32_t g_stepCounter = 0;
volatile uint8_t rx_byte = 0;


void set_abc(Motor_t *motor, uint8_t A, uint8_t B, uint8_t C) {
	HAL_GPIO_WritePin(motor->GPIOx_A, motor->GPIO_Pin_A, A ? GPIO_PIN_SET: GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor->GPIOx_B, motor->GPIO_Pin_B, B ? GPIO_PIN_SET: GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor->GPIOx_C, motor->GPIO_Pin_C, C ? GPIO_PIN_SET: GPIO_PIN_RESET);
}

static void MX_GPIOB_Phases_Init(Motor_t *motor) {
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef gi = { 0 };
	gi.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
	gi.Mode = GPIO_MODE_OUTPUT_PP;
	gi.Pull = GPIO_NOPULL;
	gi.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &gi);

	// default to all low
	set_abc(motor, 0, 0, 0);
}

void Commutation_Step(Motor_t *motor, uint8_t step) {
	switch (step) {
	case 1:
		set_abc(motor, 1, 0, 0);
		break; // HLL
	case 2:
		set_abc(motor, 1, 1, 0);
		break; // HHL
	case 3:
		set_abc(motor, 0, 1, 0);
		break; // LHL
	case 4:
		set_abc(motor, 0, 1, 1);
		break; // LHH
	case 5:
		set_abc(motor, 0, 0, 1);
		break; // LLH
	case 6:
		set_abc(motor, 1, 0, 1);
		break; // HLH
	default: // step 7
	case 7:
		set_abc(motor, 0, 0, 0);
		break; // LLL
	}
}

static void MX_TIM3_PWM_CH2_Init_5pct(Motor_t *motor) {
	/* 1) Enable clocks */
	__HAL_RCC_TIM3_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* 2) GPIO config: PB5 -> TIM3_CH2 (AF2) */
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	Commutation_Step(motor, g_step);
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;   // check AF for your MCU
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* 3) TIM3 base config */
	// 16 MHz / (PSC+1) / (ARR+1) = 20 kHz
	// PSC = 0 -> 16 MHz
	// ARR = 799 -> 16 MHz / 800 = 20 kHz
	// ARR = 399 -> 16 MHz / 400 = 40 kHz
	motor->pwm_timer->Instance = motor->pwm_timer_instance;
	motor->pwm_timer->Init.Prescaler = 0;
	motor->pwm_timer->Init.CounterMode = TIM_COUNTERMODE_UP;
	motor->pwm_timer->Init.Period = 399;
	motor->pwm_timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	motor->pwm_timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_PWM_Init(motor->pwm_timer);

	/* 4) PWM channel config: 5% duty on CH2 */
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = (motor->pwm_timer->Init.Period + 1) * 5 / 100; // 5% of 800 = 40 set duty
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(motor->pwm_timer, &sConfigOC, TIM_CHANNEL_2);

	/* 5) Start PWM */
	HAL_TIM_PWM_Start(motor->pwm_timer, TIM_CHANNEL_2);
}

static void MX_TIM2_StepTimer_Init(Motor_t *motor, float step_hz) {
	__HAL_RCC_TIM2_CLK_ENABLE();

	const uint32_t timer_clk_hz = 16000000UL;
	uint32_t psc = 15;                           // 16 MHz / (15+1) = 1 MHz
	float tick = timer_clk_hz / (psc + 1);              // = 1 000 000

	if (step_hz == 0)
		step_hz = 1;                         // avoid div-by-zero

	uint32_t arr = (uint32_t)(tick / step_hz);
	if (arr == 0)
		arr = 1;                                 // limit max frequency
	else
		arr -= 1;

	motor->commutation_timer->Instance = motor->commutation_timer_instance;
	motor->commutation_timer->Init.Prescaler = psc;
	motor->commutation_timer->Init.CounterMode = TIM_COUNTERMODE_UP;
	motor->commutation_timer->Init.Period = arr;
	motor->commutation_timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	motor->commutation_timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(motor->commutation_timer);

	HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
}


void Commutation_Start(Motor_t *motor, float step_hz) {
	MX_GPIOB_Phases_Init(motor);
	MX_TIM3_PWM_CH2_Init_5pct(motor);   // enable driver at 5% duty
	MX_TIM2_StepTimer_Init(motor, step_hz);
	//MX_LED_Init();

	g_step = 1;
	Commutation_Step(motor, g_step);

	__HAL_TIM_CLEAR_FLAG(motor->commutation_timer, TIM_FLAG_UPDATE);
	HAL_TIM_Base_Start_IT(motor->commutation_timer);
	Commutation_Step(motor, g_step);
}

void Commutation_Stop(Motor_t *motor) {
	HAL_TIM_Base_Stop_IT(motor->commutation_timer);
	// keep last commutation state; if you want to de-energize:
	set_abc(motor, 0, 0, 0);
}


void motor_irq(Motor_t *motor) {
	g_step++;
	if (g_step > 6)
		g_step = 1;
	Commutation_Step(motor, g_step);
}

void Commutation_SetFrequency(Motor_t *motor, float step_hz) {
	if (step_hz < 1.1)
		step_hz = 1.1;

	float timer_clk = 16000000UL;     // 16 MHz
	float psc = 15;           // same as in MX_TIM2_StepTimer_Init
	float tick = timer_clk / (psc + 1); // 1MHz

	uint32_t arr = (uint32_t)(tick / (step_hz - 1));

	__HAL_TIM_SET_AUTORELOAD(motor->commutation_timer, arr);

}

void SetDuty_TIM3_CH2(Motor_t *motor, uint8_t duty_percent) {
	if (duty_percent > 95)
		duty_percent = 95;
	if (duty_percent < 50)
		duty_percent = 50;

	uint32_t arr = motor->pwm_timer->Init.Period + 1;     // ARR+1 is timer resolution
	uint32_t pulse = (arr * duty_percent) / 100;

	__HAL_TIM_SET_COMPARE(motor->pwm_timer, TIM_CHANNEL_2, pulse);
}

