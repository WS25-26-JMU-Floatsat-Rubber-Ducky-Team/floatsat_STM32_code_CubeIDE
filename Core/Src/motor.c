#include "motor.h"
#include "lsm9ds1_reg.h"

volatile uint8_t rx_byte = 0;

/*static void EnableTimerClock(TIM_TypeDef *tim)
{
    if (tim == TIM1)      __HAL_RCC_TIM1_CLK_ENABLE();
    else if (tim == TIM2) __HAL_RCC_TIM2_CLK_ENABLE();
    else if (tim == TIM3) __HAL_RCC_TIM3_CLK_ENABLE();
    else if (tim == TIM4) __HAL_RCC_TIM4_CLK_ENABLE();
    else if (tim == TIM5) __HAL_RCC_TIM5_CLK_ENABLE();
    else if (tim == TIM9) __HAL_RCC_TIM9_CLK_ENABLE();
    else if (tim == TIM10) __HAL_RCC_TIM10_CLK_ENABLE();
    else if (tim == TIM11) __HAL_RCC_TIM11_CLK_ENABLE();
    // add others if you use them
}*/

void setTarget(Motor_t *m, float target_rps)
{
    m->target_rps = target_rps;
}

void setStep(Motor_t *m, float rps_step)
{
    if (rps_step < 0.0f) rps_step = 0.0f;
    m->rps_step = rps_step;
}

void Motor_ControlTick(Motor_t *m)
{
    /* 1. Ramp rps toward target_rps (can be negative) */
    if (m->rps < m->target_rps) {
        m->rps += m->rps_step;
        if (m->rps > m->target_rps)
            m->rps = m->target_rps;
    }
    else if (m->rps > m->target_rps) {
        m->rps -= m->rps_step;
        if (m->rps < m->target_rps)
            m->rps = m->target_rps;
    }

    /* 2. Update direction from sign of rps */
    if (m->rps >= 0.0f) {
        m->cw = true;
    } else {
        m->cw = false;
    }

    /* 3. Use absolute speed for commutation frequency */
    float abs_rps = (m->rps >= 0.0f) ? m->rps : -m->rps;

    if (abs_rps < 0.37f) {
        Commutation_SetFrequency(m, 0.37f*42.0f);   // idle minimum
        return;
    }

    /* 4. Mechanical RPS → electrical commutation frequency */
    Commutation_SetFrequency(m, abs_rps * 7.0f * 6.0f);
}

void set_abc(Motor_t *motor, uint8_t A, uint8_t B, uint8_t C) {
	HAL_GPIO_WritePin(motor->GPIOx_A, motor->GPIO_Pin_A, A ? GPIO_PIN_SET: GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor->GPIOx_B, motor->GPIO_Pin_B, B ? GPIO_PIN_SET: GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor->GPIOx_C, motor->GPIO_Pin_C, C ? GPIO_PIN_SET: GPIO_PIN_RESET);
}

void Commutation_Step(Motor_t *motor)
{
    uint8_t s = motor->step;   // 1..6

    // Reverse mapping for CCW: 1<->6, 2<->5, 3<->4
    if (!motor->cw) {
        s = 7 - s;
    }

    switch (s) {
    case 1: set_abc(motor, 1, 0, 0); break; // HLL
    case 2: set_abc(motor, 1, 1, 0); break; // HHL
    case 3: set_abc(motor, 0, 1, 0); break; // LHL
    case 4: set_abc(motor, 0, 1, 1); break; // LHH
    case 5: set_abc(motor, 0, 0, 1); break; // LLH
    case 6: set_abc(motor, 1, 0, 1); break; // HLH
    default: set_abc(motor, 0, 0, 0); break;
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
	Commutation_Step(motor);
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
	motor->commutation_timer->Init.CounterMode = TIM_COUNTERMODE_DOWN;
	motor->commutation_timer->Init.Period = arr;
	motor->commutation_timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	motor->commutation_timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(motor->commutation_timer);
}

void Commutation_Start(Motor_t *motor, float step_hz) {
	MX_TIM3_PWM_CH2_Init_5pct(motor);   // enable driver at 5% duty
	MX_TIM2_StepTimer_Init(motor, step_hz);

	motor -> step = 1;
	Commutation_Step(motor);

	__HAL_TIM_CLEAR_FLAG(motor->commutation_timer, TIM_FLAG_UPDATE);
	HAL_TIM_Base_Start_IT(motor->commutation_timer);
	Commutation_Step(motor);
}

void Commutation_Stop(Motor_t *motor) {
	HAL_TIM_Base_Stop_IT(motor->commutation_timer);
	// keep last commutation state; if you want to de-energize:
	set_abc(motor, 0, 0, 0);
}

void motor_irq(Motor_t *motor) {
	motor -> step++;
	if (motor -> step > 6)
		motor -> step = 1;
	Commutation_Step(motor);
}

void Commutation_SetFrequency(Motor_t *motor, float step_hz) {
	if (step_hz < 1)
		step_hz = 1;

	float timer_clk = 16000000UL;     // 16 MHz
	float psc = 15;           // same as in MX_TIM2_StepTimer_Init
	float tick = timer_clk / (psc + 1); // 1MHz

	motor->commutation_timer->Instance->ARR = (uint32_t)(tick / step_hz) - 1;
}

void SetDuty_TIM3_CH2(Motor_t *motor, uint8_t duty_percent) {
	if (duty_percent > 98)
		duty_percent = 98;
	if (duty_percent < 50)
		duty_percent = 50;

	uint32_t arr = motor->pwm_timer->Init.Period + 1;     // ARR+1 is timer resolution
	uint32_t pulse = (arr * duty_percent) / 100;

	__HAL_TIM_SET_COMPARE(motor->pwm_timer, TIM_CHANNEL_2, pulse);
}
