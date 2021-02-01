#ifndef _BLESCAN_H_
#define _BLESCAN_H_

#include <stdint.h>
#include <time.h>
#include "esp_bt_defs.h"

static const char remote_device_name[2][20] = {"PUMP-EAST", "PUMP-WEST"};

void ble_task(void * pvParameters);

#endif
