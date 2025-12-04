#include "main.h"

typedef struct Motor {
	TIM_HandleTypeDef *commutation_timer;
	TIM_TypeDef *commutation_timer_instance;
	TIM_HandleTypeDef *pwm_timer;
	TIM_TypeDef *pwm_timer_instance;
} Motor_t;


void Commutation_Start(Motor_t *motor, uint32_t step_hz);
void Commutation_Stop(Motor_t *motor);
void motor_irq(Motor_t *motor);
void Commutation_SetFrequency(Motor_t *motor, uint32_t step_hz);
void SetDuty_TIM3_CH2(Motor_t *motor, uint8_t duty_percent);




