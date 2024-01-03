#ifndef _IMU_H_
#define _IMU_H_

#include "esp_err.h"

typedef struct attitude{
    float roll;
    float pitch;
    float yaw;
} attitude_t;

esp_err_t imuInit(void);

#endif /* _IMU_H_ */