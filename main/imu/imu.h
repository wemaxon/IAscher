#ifndef _IMU_H_
#define _IMU_H_

#include "esp_err.h"


esp_err_t imuInit(void);
esp_err_t imuDumpData(void);

#endif /* _IMU_H_ */