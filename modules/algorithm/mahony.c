#include "mahony.h"

#include <math.h>

void MahonyInit(MahonyFilter_t *filter, float kp, float ki)
{
    filter->q[0] = 1.0f;
    filter->q[1] = 0.0f;
    filter->q[2] = 0.0f;
    filter->q[3] = 0.0f;
    filter->kp = kp;
    filter->ki = ki;
    filter->integral[0] = filter->integral[1] = filter->integral[2] = 0.0f;
}

void MahonyUpdate(MahonyFilter_t *filter, float gx, float gy, float gz,
                  float ax, float ay, float az, float dt)
{
    float q0 = filter->q[0], q1 = filter->q[1], q2 = filter->q[2], q3 = filter->q[3];
    const float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm > 1.0e-6f)
    {
        ax /= norm;
        ay /= norm;
        az /= norm;
        const float vx = 2.0f * (q1 * q3 - q0 * q2);
        const float vy = 2.0f * (q0 * q1 + q2 * q3);
        const float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
        const float ex = ay * vz - az * vy;
        const float ey = az * vx - ax * vz;
        const float ez = ax * vy - ay * vx;
        filter->integral[0] += filter->ki * ex * dt;
        filter->integral[1] += filter->ki * ey * dt;
        filter->integral[2] += filter->ki * ez * dt;
        gx += filter->kp * ex + filter->integral[0];
        gy += filter->kp * ey + filter->integral[1];
        gz += filter->kp * ez + filter->integral[2];
    }
    const float dq0 = (-q1 * gx - q2 * gy - q3 * gz) * 0.5f * dt;
    const float dq1 = (q0 * gx + q2 * gz - q3 * gy) * 0.5f * dt;
    const float dq2 = (q0 * gy - q1 * gz + q3 * gx) * 0.5f * dt;
    const float dq3 = (q0 * gz + q1 * gy - q2 * gx) * 0.5f * dt;
    q0 += dq0;
    q1 += dq1;
    q2 += dq2;
    q3 += dq3;
    const float qnorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    filter->q[0] = q0 * qnorm;
    filter->q[1] = q1 * qnorm;
    filter->q[2] = q2 * qnorm;
    filter->q[3] = q3 * qnorm;
}

float MahonyYaw(const MahonyFilter_t *filter)
{
    return atan2f(2.0f * (filter->q[0] * filter->q[3] + filter->q[1] * filter->q[2]),
                  1.0f - 2.0f * (filter->q[2] * filter->q[2] + filter->q[3] * filter->q[3]));
}
