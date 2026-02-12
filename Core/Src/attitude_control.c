#include "attitude_control.h"
#include <math.h>

float pitch_mem = 0.0;
float roll_mem = 0.0;
float yaw_mem = 0.0;

PID_t pitch_speed_pid = {
		.dt = 0.002,
		.Kp = 0.1,
		.Ki = 1.0,
		.Kd = 0.0,
		.Kt = 1.0,  // eq to Ki
		.d_tau = 0.001,
		.lim = 10,

		.saturation = 0,
		.integrator = 0,
		.previous_error = 0,
		.differentiator = 0,
		.previous_measurement = 0,
		.out = 0,
};

PID_t roll_speed_pid = {
		.dt = 0.002,
		.Kp = 0.1,
		.Ki = 1.0,
		.Kd = 0.0,
		.Kt = 1.0,  // eq to Ki
		.d_tau = 0.001,
		.lim = 10,

		.saturation = 0,
		.integrator = 0,
		.previous_error = 0,
		.differentiator = 0,
		.previous_measurement = 0,
		.out = 0,
};

PID_t yaw_speed_pid = {
		.dt = 0.002,
		.Kp = 0.05,
		.Ki = 5.0,
		.Kd = 0.0,
		.Kt = 5.0,  // eq to Ki
		.d_tau = 0.001,
		.lim = 20,

		.saturation = 0,
		.integrator = 0,
		.previous_error = 0,
		.differentiator = 0,
		.previous_measurement = 0,
		.out = 0,
};

PID_t pitch_position_pid = {
		.dt = 0.002,
		.Kp = 1.0,
		.Ki = 0.1,
		.Kd = 0.0,
		.Kt = 0.1,  // eq to Ki
		.d_tau = 0.001,
		.lim = 2,

		.saturation = 0,
		.integrator = 0,
		.previous_error = 0,
		.differentiator = 0,
		.previous_measurement = 0,
		.out = 0,
};

PID_t roll_position_pid = {
		.dt = 0.002,
		.Kp = 1.0,
		.Ki = 0.1,
		.Kd = 0.0,
		.Kt = 0.1,  // eq to Ki
		.d_tau = 0.001,
		.lim = 2,

		.saturation = 0,
		.integrator = 0,
		.previous_error = 0,
		.differentiator = 0,
		.previous_measurement = 0,
		.out = 0,
};

PID_t yaw_position_pid = {
		.dt = 0.002,
		.Kp = 1.5,
		.Ki = 0.01,
		.Kd = 0.0,
		.Kt = 0.1,  // eq to Ki
		.d_tau = 0.001,
		.lim = 2,

		.saturation = 0,
		.integrator = 0,
		.previous_error = 0,
		.differentiator = 0,
		.previous_measurement = 0,
		.out = 0,
};

void control_step(IMU_t *imu, measurement_t *meas, control_params_t *params, float *omega_body_z_cmd, vec3_t *motor_speed, quat_t *q_setpoint) {
	float yaw_target = *omega_body_z_cmd;

	float c_pitch_est = pitch_from_quat(&(meas->q));
	float t_pitch = pitch_from_quat(q_setpoint);
	PID_controller(&pitch_position_pid, t_pitch, c_pitch_est);

	float c_roll_est = roll_from_quat(&(meas->q));
	float t_roll = roll_from_quat(q_setpoint);
	PID_controller(&roll_position_pid, t_roll, c_roll_est);

	if (*omega_body_z_cmd > -0.1f && *omega_body_z_cmd < 0.1f) { // if the rate setpoint is zero activate position control
		float c_yaw_est = yaw_from_quat(&(meas->q));
		float t_yaw = yaw_from_quat(q_setpoint);
		PID_controller(&yaw_position_pid, t_yaw, c_yaw_est);
		yaw_target = yaw_speed_pid.out;
	}

	PID_controller(&pitch_speed_pid, pitch_position_pid.out, imu->gyro.v[0]);
	PID_controller(&roll_speed_pid, -roll_position_pid.out, -imu->gyro.v[1]);
	PID_controller(&yaw_speed_pid, yaw_position_pid.out, imu->gyro.v[2]);

	motor_mix(motor_speed, pitch_speed_pid.out, roll_speed_pid.out, yaw_target);
}

void motor_mix(vec3_t *motor_speed, float pitch, float roll, float yaw) {
	motor_speed->v[0] =  0.8165f * roll + 0.0f    * pitch + 0.5773f * yaw;
	motor_speed->v[1] = -0.4083f * roll + 0.7071f * pitch + 0.5773f * yaw;
	motor_speed->v[2] = -0.4083f * roll - 0.7071f * pitch + 0.5773f * yaw;
}
