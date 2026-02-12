#ifndef ATTITUDE_CONTROL_H
#define ATTITUDE_CONTROL_H

#include "attitude_types.h"
#include "attitude_estimator.h"
#include "imu.h"
#include <stdint.h>
#include "pid.h"

typedef struct {
    /* Outer (angle) loop */
    float angle_kp[3];
    float angle_ki[3];
    float angle_kd[3];
    float max_angvel_cmd[3];     // rad/s clamp

    /* Inner (rate) loop */
    float rate_kp[3];
    float rate_ki[3];
    float rate_kd[3];

    /* Actuator limits */
    float max_motor_rpm;
    float max_motor_torque;

    /* Geometry */
    float wheel_axis[3][3];  // [wheel][xyz]

    /* Timing */
    float dt;

    /* Virtual actuator mapping */
    float torque_to_rpm;     // [RPM / Nm]
} control_params_t;

void control_step(IMU_t *imu, measurement_t *meas, control_params_t *params, float *omega_body_z_cmd, vec3_t *motor_speed, quat_t *q_setpoint);
void motor_mix(vec3_t *motor_speed, float pitch, float roll, float yaw);

#endif /* ATTITUDE_CONTROL_H */
