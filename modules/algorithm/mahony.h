#ifndef MAHONY_H
#define MAHONY_H

typedef struct
{
    float q[4];
    float kp;
    float ki;
    float integral[3];
} MahonyFilter_t;

void MahonyInit(MahonyFilter_t *filter, float kp, float ki);
void MahonyUpdate(MahonyFilter_t *filter, float gx, float gy, float gz,
                  float ax, float ay, float az, float dt);
float MahonyYaw(const MahonyFilter_t *filter);

#endif
