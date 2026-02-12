#include "attitude_estimator.h"
#include <math.h>

#define EPS 1e-6f // Tiny safety constant so you don’t divide by zero/

// Quaternion helpers
static quat_t quat_mul(quat_t a, quat_t b)
{
    quat_t r;
    r.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    r.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    r.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    r.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
    return r;
}

static quat_t quat_norm(quat_t q)
{
    float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z) + EPS;
    q.w/=n; q.x/=n; q.y/=n; q.z/=n;
    return q;
}

// Euler → quaternion
static quat_t eul2quat(float r, float p, float y)
{
    float cr = cosf(r*0.5f), sr = sinf(r*0.5f);
    float cp = cosf(p*0.5f), sp = sinf(p*0.5f);
    float cy = cosf(y*0.5f), sy = sinf(y*0.5f);

    quat_t q;
    q.w = cy*cp*cr + sy*sp*sr;
    q.x = cy*cp*sr - sy*sp*cr;
    q.y = sy*cp*sr + cy*sp*cr;
    q.z = sy*cp*cr - cy*sp*sr;
    return quat_norm(q);
}

// Acc+Mag → quaternion (absolute attitude)
static quat_t quat_from_acc(vec3_t acc, float yaw_est)
{
    float ax=acc.v[0], ay=acc.v[1], az=acc.v[2];

    // Normalize
    float n = sqrtf(ax*ax + ay*ay + az*az) + EPS;
    ax/=n; ay/=n; az/=n;

    float roll  = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));

    return eul2quat(roll, pitch, yaw_est);
}

void measurement_init(meas_state_t *s, vec3_t acc, vec3_t mag)
{
    s->q.w = 1;
    s->q.x = s->q.y = s->q.z = 0;
}

float yaw_from_quat(quat_t *q) {
	// Extract current yaw from q
	float ys = 2.0f*(q->w*q->z + q->x*q->y);
	float yc = 1.0f - 2.0f*(q->y*q->y + q->z*q->z);
	float yaw_est = atan2f(ys, yc);
}

measurement_t measurement_update(
    meas_state_t *s,
    const IMU_t *raw,
    float dt
)
{
    measurement_t out;

    // Gyro integrate
    float wx=raw->gyro.v[0], wy=raw->gyro.v[1], wz=raw->gyro.v[2];

    quat_t q = s->q;
    quat_t q_dot = {0,
        0.5f*( wx*q.w + wy*q.z - wz*q.y),
        0.5f*(-wx*q.z + wy*q.w + wz*q.x),
        0.5f*( wx*q.y - wy*q.x + wz*q.w)
    };

    // --- Gyro integration ---
    quat_t q_gyro = {
        q.w + q_dot.w*dt,
        q.x + q_dot.x*dt,
        q.y + q_dot.y*dt,
        q.z + q_dot.z*dt
    };
    q_gyro = quat_norm(q_gyro);

    // Extract current yaw from q_gyro
    float yaw_est = yaw_from_quat(&q_gyro);

    // --- Tilt correction from ACC ONLY ---
    quat_t q_tilt = quat_from_acc(raw->acc, yaw_est);

    // Blend roll/pitch only by nudging gyro orientation toward tilt
    const float alpha_rp = 0.02f;

    quat_t q_rp_blend;
    q_rp_blend.w = (1-alpha_rp)*q_gyro.w + alpha_rp*q_tilt.w;
    q_rp_blend.x = (1-alpha_rp)*q_gyro.x + alpha_rp*q_tilt.x;
    q_rp_blend.y = (1-alpha_rp)*q_gyro.y + alpha_rp*q_tilt.y;
    q_rp_blend.z = (1-alpha_rp)*q_gyro.z + alpha_rp*q_tilt.z;
    q_rp_blend = quat_norm(q_rp_blend);

    s->q = q_rp_blend;

    // Outputs
    out.q = s->q;
    out.omega = raw->gyro;

    return out;
}
