#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include "attitude_types.h"
#include "imu.h"

typedef struct {
    quat_t q;      // estimated attitude
    vec3_t omega;  // angular velocity (rad/s)
} measurement_t;

typedef struct {
    quat_t q;      // filter state
} meas_state_t;

void measurement_init(meas_state_t *s, vec3_t acc, vec3_t mag);

measurement_t measurement_update(
    meas_state_t *s,
    const IMU_t *raw,
    float dt
);

float yaw_from_quat(quat_t *q);
float pitch_from_quat(quat_t *q);
float roll_from_quat(quat_t *q);

#endif /* ATTITUDE_ESTIMATOR_H */
