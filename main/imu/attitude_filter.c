#include "attitude_filter.h"

#include <math.h>
#include <string.h>

#define ACCEL_FULL_TRUST_DEVIATION_G  0.10f
#define ACCEL_ZERO_TRUST_DEVIATION_G  0.35f


static float accelTrust(float accel_norm_g)
{
    const float deviation_g = fabsf(accel_norm_g - 1.0f);

    if (deviation_g <= ACCEL_FULL_TRUST_DEVIATION_G) {
        return 1.0f;
    }
    if (deviation_g >= ACCEL_ZERO_TRUST_DEVIATION_G) {
        return 0.0f;
    }

    return (ACCEL_ZERO_TRUST_DEVIATION_G - deviation_g) /
           (ACCEL_ZERO_TRUST_DEVIATION_G - ACCEL_FULL_TRUST_DEVIATION_G);
}

void attitudeFilterReset(attitude_filter_t *filter)
{
    memset(filter, 0, sizeof(*filter));
}

attitude_t attitudeFilterUpdate(attitude_filter_t *filter,
                                float accel_x_g,
                                float accel_y_g,
                                float accel_z_g,
                                float gyro_x_rad_s,
                                float gyro_y_rad_s,
                                float gyro_z_rad_s,
                                float dt_s,
                                float correction_time_s)
{
    const float yz_length = sqrtf((accel_y_g * accel_y_g) +
                                  (accel_z_g * accel_z_g));
    const float xz_length = sqrtf((accel_x_g * accel_x_g) +
                                  (accel_z_g * accel_z_g));
    const float accel_pitch = atan2f(-accel_x_g, yz_length);
    const float accel_roll = atan2f(accel_y_g, xz_length);

    if (!filter->initialized) {
        filter->attitude.pitch = accel_pitch;
        filter->attitude.roll = accel_roll;
        filter->initialized = true;
        return filter->attitude;
    }

    const float predicted_pitch = filter->attitude.pitch + (gyro_y_rad_s * dt_s);
    const float predicted_roll = filter->attitude.roll + (gyro_x_rad_s * dt_s);
    const float accel_norm_g = sqrtf((accel_x_g * accel_x_g) +
                                     (accel_y_g * accel_y_g) +
                                     (accel_z_g * accel_z_g));
    const float base_correction_weight = dt_s / (correction_time_s + dt_s);
    const float correction_weight = base_correction_weight * accelTrust(accel_norm_g);

    filter->attitude.pitch = predicted_pitch +
                             (correction_weight * (accel_pitch - predicted_pitch));
    filter->attitude.roll = predicted_roll +
                            (correction_weight * (accel_roll - predicted_roll));
    filter->attitude.yaw += gyro_z_rad_s * dt_s;

    return filter->attitude;
}
