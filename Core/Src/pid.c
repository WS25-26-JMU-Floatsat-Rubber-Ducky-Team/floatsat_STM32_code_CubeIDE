/*
 * pid.c
 *
 *  Created on: Mar 11, 2025
 *      Author: Luis
 */


#include "pid.h"

// see: https://github.com/pms67/PID/blob/master/PID.c
// for anti-windup see: https://www.youtube.com/watch?v=QaffgVgcRr0
float PID_controller(PID_t *pid, float setpoint, float measurement) {
	// Error
	float error = setpoint - measurement;

	// Proportional
	float proportional = pid->Kp * error;

	// Integral
	pid->integrator = pid->integrator
			+ 0.5f * pid->Ki * pid->dt * (error + pid->previous_error) // Integral
			- pid->Kt * pid->saturation; // Anti-wind-up

	// Derivative on measurement with band-limit
	pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->previous_measurement)
						+ (2.0f * pid->d_tau - pid->dt) * pid->differentiator)
						/ (2.0f * pid->d_tau + pid->dt);

	// Output
	float out = proportional + pid->integrator + pid->differentiator;

	// Limit
	if (out > pid->lim) pid->out = pid->lim; // Positive limit
	else if (out < -pid->lim) pid->out = -pid->lim; // Negative limit
	else pid->out = out; // no limit

	/* Store error and measurement for later use */
	pid->saturation = out - pid->out;
	pid->previous_error = error;
	pid->previous_measurement = measurement;

	/* Return controller output */
	return pid->out;
}
