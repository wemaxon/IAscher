#ifndef _LED_H_
#define _LED_H_

#include "esp_err.h"


#define LED_STATUS_PIN      (8)


esp_err_t ledInit(void);
esp_err_t ledStatusSet(uint8_t level);


#endif /* _LED_H_ */