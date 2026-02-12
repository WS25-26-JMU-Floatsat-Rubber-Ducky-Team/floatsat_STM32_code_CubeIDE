/*
 * pid.h
 *
 *  Created on: Mar 11, 2025
 *      Author: Luis
 */

#ifndef INC_PID_H_
#define INC_PID_H_


// see: https://github.com/pms67/PID/blob/master/PID.h
typedef struct {
	float dt; // Sample time / time between samples (in seconds)

	float Kp;
	float Ki;
	float Kd;
	float Kt; // anti-windup constant. To small -> anti-winup ineffective. To large -> sensitive to measurement error. Pick something like: Kt = Ki or Kt = 1/sqrt((1/ki)(1/kd))

	float d_tau; // Derivative low-pass filter time constant

	float lim; // Output limit
	float saturation; // Output saturation due to limit

	float integrator;
	float previous_error;

	float differentiator;
	float previous_measurement;

	float out; // limited Controller output
} PID_t;

float PID_init(PID_t *pid, float dt, float Kp, float Ki, float Kd, float d_tau, float lim);
float PID_controller(PID_t *pid, float setpoint, float measurement);


#endif /* INC_PID_H_ */
