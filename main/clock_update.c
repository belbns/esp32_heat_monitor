/****************************************************************************
* Nikolay Belov
* 
* Task clock_task
* Sync system and BLE devices time
* 
****************************************************************************/

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"

#include "esp_sntp.h"

#include "main.h"

#define RU_NTP_POOL	"ru.pool.ntp.org"
#define CLOCK_TAG "CLOCK"

uint8_t fl_time = false;

void time_sync_notification_cb(struct timeval *tv)
{
	ESP_LOGI(CLOCK_TAG, "Notification of a time sync. event");

	// Sync controllers time after init (fl_time == true)
	if (fl_time) {
    	for (uint8_t i = 0; i < 2; i++) {
    		if (put_to_cmd_queue(i, SET_TIME, 0)) {
        		ESP_LOGI(CLOCK_TAG, "=== SET_TIME -> pump_cmd_queue[%d]", i);
    		} else {
        		ESP_LOGI(CLOCK_TAG, "*** Cannot send SET_TIME to pump_cmd_queue[%d]", i);
    		}
    	}
    	// Save parameters on sd card
    	save_params();
    }
}


void clock_task(void * pvParameters)
{
    ( void ) pvParameters;

    ESP_LOGI(CLOCK_TAG, "== clock_task started");

	// initialize the SNTP service
	sntp_setoperatingmode(SNTP_OPMODE_POLL);
	sntp_setservername(0, RU_NTP_POOL);
	sntp_set_time_sync_notification_cb(time_sync_notification_cb);

	sntp_init();
	
	// wait for the service to set the time
	time_t now;
	struct tm timeinfo;
	time(&now);
	localtime_r(&now, &timeinfo);
	while(timeinfo.tm_year < (2016 - 1900)) {
		
		printf("Time not set, waiting...\n");
		vTaskDelay(5000 / portTICK_PERIOD_MS);
		time(&now);
        localtime_r(&now, &timeinfo);
	}

	// change the timezone
	setenv("TZ", "MSK-3", 1);
	tzset();
   	
	localtime_r(&now, &timeinfo);

	// first time sending 2 commands to each device
	for (uint8_t i = 0; i < 4; i++) {
    	if (put_to_cmd_queue(i / 2, SET_TIME, 0)) {
        	ESP_LOGI(CLOCK_TAG, "=== SET_TIME -> pump_cmd_queue[%d]", i / 2);
    	} else {
        	ESP_LOGI(CLOCK_TAG, "*** Cannot send SET_TIME to pump_cmd_queue[%d]", i / 2);
    	}
    }
    fl_time = true;

	char *buffer = (char *)malloc(100);
	strftime(buffer, 100, "%d/%m/%Y %H:%M:%S", &timeinfo);
	ESP_LOGI(CLOCK_TAG, "Actual time: %s\n", buffer);
	free(buffer);

	while(1) {
		vTaskDelay((4 * 60 * 60 * 1000) / portTICK_RATE_MS); // 4 hours
		sntp_stop();
		sntp_init();
	}	
}