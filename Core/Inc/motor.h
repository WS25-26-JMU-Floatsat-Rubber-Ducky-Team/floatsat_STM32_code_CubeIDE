#include "main.h"
#include "stdbool.h"
typedef struct Motor {
	TIM_HandleTypeDef *commutation_timer;
	TIM_TypeDef *commutation_timer_instance;
	TIM_HandleTypeDef *pwm_timer;
	TIM_TypeDef *pwm_timer_instance;
	GPIO_TypeDef* GPIOx_A;
	uint16_t GPIO_Pin_A;
	GPIO_TypeDef* GPIOx_B;
	uint16_t GPIO_Pin_B;
	GPIO_TypeDef* GPIOx_C;
	uint16_t GPIO_Pin_C;
	uint8_t step;
	float rps;             // current commanded speed (mechanical)
	float target_rps;      // target speed (mechanical)
	float rps_step;        // ramp per control tick (rps/tick)
	bool cw;
} Motor_t;


void Commutation_Start(Motor_t *motor, float step_hz);
void Commutation_Stop(Motor_t *motor);
void motor_irq(Motor_t *motor);
void Commutation_SetFrequency(Motor_t *motor, float step_hz);
void SetDuty_TIM3_CH2(Motor_t *motor, uint8_t duty_percent);
void setTarget(Motor_t *m, float target_rps);
void setStep(Motor_t *m, float rps_step);
void Motor_ControlTick(Motor_t *m);
