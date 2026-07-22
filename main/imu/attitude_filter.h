#ifndef _ATTITUDE_FILTER_H_
#define _ATTITUDE_FILTER_H_

#include <stdbool.h>

typedef struct attitude {
    float roll;
    float pitch;
    float yaw;
} attitude_t;

typedef struct attitude_filter {
    attitude_t attitude;
    bool initialized;
} attitude_filter_t;


void attitudeFilterReset(attitude_filter_t *filter);
attitude_t attitudeFilterUpdate(attitude_filter_t *filter,
                                float accel_x_g,
                                float accel_y_g,
                                float accel_z_g,
                                float gyro_x_rad_s,
                                float gyro_y_rad_s,
                                float gyro_z_rad_s,
                                float dt_s,
                                float correction_time_s);

#endif /* _ATTITUDE_FILTER_H_ */
