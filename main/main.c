/* 
 * Nikolay Belov
 *
 * Tools: Espressif ESP-IDF v4.3-dev-2136-gb0150615d.
 * Based on Espressif examples.
 * Use nopnop2002 ili9340 library (https://github.com/nopnop2002/esp-idf-ili9340)
 *
 * Scanning and control two BLE devices (STM32F030F4P6, YDY-16),
 * display main parameters via TFT display (ILI9341),
 * display and set parameters with web server,
 * use sd card for web server files and save parameters and statistics. 
**/

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include <esp_http_server.h>
#include <cJSON.h>
#include "esp_timer.h"
#include "esp_spiffs.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"

#include "main.h"
#include "ble_scan.h"
#include "clock_update.h"

#include "ili9340.h"
#include "fontx.h"
#include "bmpfile.h"
#include "decode_image.h"
#include "pngle.h"

#define ESP_WIFI_SSID      CONFIG_WIFI_SSID
#define ESP_WIFI_PASS      CONFIG_WIFI_PASSWORD
#define ESP_MAXIMUM_RETRY  10

#define WEB_SERVER_PORT     80

#define CHECK_FILE_EXTENSION(filename, ext) (strcasecmp(&filename[strlen(filename) - strlen(ext)], ext) == 0)

extern uint8_t fl_time;  // true - synchronized

xQueueHandle pump_cmd_queue[2];

pump_struct heat_ctrl[2] = {
    {   .mode_set = 1, .mode_rec = 1,
        .t_main = 37.50, .t_aux = 25.25, .t_air = 20.0,
        .err_main = 0, .err_aux = 0, .err_air = 0,
        .delta_on_set = 8, .delta_on_rec = 8, .delta_off_set = 1, .delta_off_rec = 1,
        .night_begin_set.hour = 23, .night_begin_set.min = 0, .night_begin_set.sec = 0,
        .night_begin_rec.hour = 23, .night_begin_rec.min = 0, .night_begin_rec.sec = 0,
        .night_end_set.hour = 8, .night_end_set.min = 0, .night_end_set.sec = 0,
        .night_end_rec.hour = 8, .night_end_rec.min = 0, .night_end_rec.sec = 0,
        .period_set.hour = 2, .period_set.min = 20, .period_set.sec = 0,
        .period_rec.hour = 2, .period_rec.min = 20, .period_rec.sec = 0,
        .ctrl_time.hour = 0, .ctrl_time.min = 0, .ctrl_time.sec = 0,
        .last_start.hour = 0, .last_start.min = 0, .last_start.sec = 0,
        .sclk_set = 399, .sclk_rec = 399,
        .lock_set = 0, .lock_rec = 0, .state = 0, .night = 0, .pump_on = 0,
        .pressure = 0.0, .press_bar = 0.0, .vref_cal = 0, .vref_data = 0,
        .rec_flags = 0, .updated = true, .tm_updated = 0, .upd_cnt = 0 },
        
    {   .mode_set = 1, .mode_rec = 1,
        .t_main = 39.75, .t_aux = 28.0, .t_air = 21.50,
        .err_main = 0, .err_aux = 0, .err_air = 0,
        .delta_on_set = 6, .delta_on_rec = 8, .delta_off_set = 1, .delta_off_rec = 1,
        .night_begin_set.hour = 22, .night_begin_set.min = 30, .night_begin_set.sec = 0,
        .night_begin_rec.hour = 23, .night_begin_rec.min = 0, .night_begin_rec.sec = 0,
        .night_end_set.hour = 7, .night_end_set.min = 45, .night_end_set.sec = 0,
        .night_end_rec.hour = 8, .night_end_rec.min = 0, .night_end_rec.sec = 0,
        .period_set.hour = 3, .period_set.min = 10, .period_set.sec = 0,
        .period_rec.hour = 2, .period_rec.min = 20, .period_rec.sec = 0,
        .ctrl_time.hour = 0, .ctrl_time.min = 0, .ctrl_time.sec = 0,
        .last_start.hour = 0, .last_start.min = 0, .last_start.sec = 0,
        .sclk_set = 399, .sclk_rec = 399,
        .lock_set = 0, .lock_rec = 0, .state = 0, .night = 0, .pump_on = 0,
        .pressure = 0.0, .press_bar = 0.0, .vref_cal = 0, .vref_data = 0,
        .rec_flags = 0, .updated = true, .tm_updated = 0, .upd_cnt = 0 }
};

TFT_t dev;

int width = CONFIG_WIDTH;
int height = CONFIG_HEIGHT;
//FontxFile fx16M[2];
//FontxFile fx24M[2];
FontxFile fx32M[2];

uint8_t buffer[FontxGlyphBufSize];
uint8_t fontWidth;
uint8_t fontHeight;
FontxFile *fx = fx32M;


static const char mount_point[] = MOUNT_POINT;
static const char *TAG = "MAIN";

static int s_retry_num = 0;

static httpd_handle_t wserver = NULL;

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static httpd_handle_t start_webserver(void);
static void stop_webserver(httpd_handle_t server);


static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (wserver != NULL) {
            ESP_LOGI(TAG, "Stopping webserver");
            stop_webserver(wserver);
            wserver = NULL;
        }
        if (s_retry_num < ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        if (wserver == NULL) {
            ESP_LOGI(TAG, "Starting webserver");
            wserver = start_webserver();
        }
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, 
        ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, 
        IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, 
    hence we can test which event actually happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s", ESP_WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s", ESP_WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

// init file system on sd card
esp_err_t init_fs(void)
{
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
#ifndef CONFIG_SDCARD_HSPI_HOST //(default)   
    host.slot = VSPI_HOST; 
#endif
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_card_t* card;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CHAN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return ESP_FAIL;;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s)", esp_err_to_name(ret));
        }
        return ESP_FAIL;
    }
    /* print card info if mount successfully */
    sdmmc_card_print_info(stdout, card);
    return ESP_OK;
}

// set content type by file name extention
static esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filepath)
{
    const char *type = "text/plain";
    if (CHECK_FILE_EXTENSION(filepath, ".html") || CHECK_FILE_EXTENSION(filepath, ".htm")) {
        type = "text/html";
    } else if (CHECK_FILE_EXTENSION(filepath, ".js")) {
        type = "application/javascript";
    } else if (CHECK_FILE_EXTENSION(filepath, ".css")) {
        type = "text/css";
    } else if (CHECK_FILE_EXTENSION(filepath, ".png")) {
        type = "image/png";
    } else if (CHECK_FILE_EXTENSION(filepath, ".ico")) {
        type = "image/x-icon";
    } else if (CHECK_FILE_EXTENSION(filepath, ".svg")) {
        type = "text/xml";
    }
    return httpd_resp_set_type(req, type);
}

/* Send HTTP response with the contents of the requested file */
static esp_err_t common_get_handler(httpd_req_t *req)
{    
    char *file_path = (char *)malloc(FPATH_LEN);
    if (file_path == 0) {
        ESP_LOGE(TAG, "malloc error, no memory for file_path");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to open file (1)");
        return ESP_FAIL;
    }

    strlcpy(file_path, WWW_DIR, FPATH_LEN);
    if (req->uri[strlen(req->uri) - 1] == '/') {
        strlcat(file_path, INDEX_FILE, FPATH_LEN);
    } else {
        strlcat(file_path, req->uri, FPATH_LEN);
    }

    ESP_LOGI(TAG, "file_path= %s", file_path);

    int fd = open(file_path, O_RDONLY, 0);
    if (fd == -1) {
        free(file_path);
        ESP_LOGE(TAG, "Failed to open file %s", file_path);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to open file (2)");
        return ESP_FAIL;
    }

    set_content_type_from_file(req, file_path);

    char  *chunk = malloc(SCRATCH_BUFSIZE);
    if (chunk == 0) {
        free(file_path);
        ESP_LOGE(TAG, "malloc error, no memory for chunk");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to open file (3)");
        return ESP_FAIL;
    }
    ssize_t read_bytes;    
    do {
        read_bytes = read(fd, chunk, SCRATCH_BUFSIZE);
        if (read_bytes == -1) {
            ESP_LOGE(TAG, "Failed to read file : %s", file_path);
        } else if (read_bytes > 0) {
            if (httpd_resp_send_chunk(req, chunk, read_bytes) != ESP_OK) {
                close(fd);
                free(chunk);
                free(file_path);
                ESP_LOGE(TAG, "File sending failed!");
                // Abort sending file
                httpd_resp_sendstr_chunk(req, NULL);
                // Respond with 500 Internal Server Error
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
                return ESP_FAIL;
            }
        }
    } while (read_bytes > 0);
    
    close(fd);
    free(chunk);
    free(file_path);
    
    /* Respond with an empty chunk to signal HTTP response completion */
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

// receive form parameters - "/set_params"
static esp_err_t params_get_handler(httpd_req_t *req)
{
    char*  buf;
    size_t buf_len;
    uint8_t pn = 0;

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            //ESP_LOGI(TAG, "Found URL query => %s", buf);
            char param[32];
            char b[4];
            uint8_t h = 0;
            uint8_t m = 0;
            uint8_t d = 0;

            // Get value of expected key from query string 
            uint8_t fl_pname = 0;
            if (httpd_query_key_value(buf, "pname", param, sizeof(param)) == ESP_OK) {
                if (strcmp(param, &remote_device_name[0][5]) == 0) {
                    pn = 0;
                    fl_pname = 1;
                } else if (strcmp(param, &remote_device_name[1][5]) == 0) {
                    pn = 1;
                    fl_pname = 1;
                }
            }

            if (fl_pname) { // contains "pname"
                uint8_t fl_cmd_mode = false;
                if (httpd_query_key_value(buf, "period", param, sizeof(param)) == ESP_OK) {
                    strncpy(b, param, 2);
                    b[2] = '\0';
                    h = atoi(b);
                    strncpy(b, &param[5], 2);
                    b[2] = '\0';
                    m = atoi(b);
                    heat_ctrl[pn].period_set.hour = h;
                    heat_ctrl[pn].period_set.min = m;
                    if ((heat_ctrl[pn].period_rec.hour != h) || (heat_ctrl[pn].period_rec.min != m)) {
                        fl_cmd_mode = true;
                    }
                }
                if (httpd_query_key_value(buf, "mode", param, sizeof(param)) == ESP_OK) {
                    uint8_t m = atoi(param);
                    heat_ctrl[pn].mode_set = m;
                    if (heat_ctrl[pn].mode_rec != m) {
                        fl_cmd_mode = true;
                    }
                }
                if (fl_cmd_mode) { // "CCC:SMOD:[01]:HHmm"
                    if (put_to_cmd_queue(pn, SET_MODE, 0)) {
                        ESP_LOGI(TAG, "=== SET_MODE -> pump_cmd_queue[%d]", pn);
                        fl_cmd_mode = false;
                    } else {
                        ESP_LOGI(TAG, "*** Cannot send SET_MODE to pump_cmd_queue[%d]", pn);
                    }
                }

                uint8_t fl_cmd_night = false;
                if (httpd_query_key_value(buf, "nightb", param, sizeof(param)) == ESP_OK) {
                    strncpy(b, param, 2);
                    b[2] = '\0';
                    h = atoi(b);
                    strncpy(b, &param[5], 2);
                    b[2] = '\0';
                    m = atoi(b);
                    heat_ctrl[pn].night_begin_set.hour = h;
                    heat_ctrl[pn].night_begin_set.min = m;
                    if ((heat_ctrl[pn].night_begin_rec.hour != h) || (heat_ctrl[pn].night_begin_rec.min != m)) {
                        fl_cmd_night = true;
                    }
                }
                if (httpd_query_key_value(buf, "nighte", param, sizeof(param)) == ESP_OK) {
                    strncpy(b, param, 2);
                    b[2] = '\0';
                    h = atoi(b);
                    strncpy(b, &param[5], 2);
                    b[2] = '\0';
                    m = atoi(b);
                    heat_ctrl[pn].night_end_set.hour = h;
                    heat_ctrl[pn].night_end_set.min = m;
                    if ((heat_ctrl[pn].night_end_rec.hour != h) || (heat_ctrl[pn].night_end_rec.min != m)) {
                        fl_cmd_night = true;
                    }
                }
                if (fl_pname && fl_cmd_night) { // "CCC:NSET:HHMM:hhmm"
                    if (put_to_cmd_queue(pn, SET_NIGHT, 0)) {
                        ESP_LOGI(TAG, "=== SET_NIGHT -> pump_cmd_queue[%d]", pn);
                        fl_cmd_night = false;
                    } else {
                        ESP_LOGI(TAG, "*** Cannot send SET_NIGHT to pump_cmd_queue[%d]", pn);
                    }
                }

                uint8_t fl_cmd_delta = false;
                if (httpd_query_key_value(buf, "d_up", param, sizeof(param)) == ESP_OK) {
                    d = atoi(param);
                    heat_ctrl[pn].delta_on_set = d;
                    if (heat_ctrl[pn].delta_on_rec != d) {
                        fl_cmd_delta = true;
                    }
                }
                if (httpd_query_key_value(buf, "d_down", param, sizeof(param)) == ESP_OK) {
                    d = atoi(param);
                    heat_ctrl[pn].delta_off_set = d;
                    if (heat_ctrl[pn].delta_off_rec != d) {
                        fl_cmd_delta = true;
                    }
                }
                if (fl_cmd_delta) { // "CCC:DIFF:U:D"
                    if (put_to_cmd_queue(pn, SET_DIFF, 0)) {
                        ESP_LOGI(TAG, "=== SET_DIFF -> pump_cmd_queue[%d]", pn);
                        fl_cmd_delta = false;
                    } else {
                        ESP_LOGI(TAG, "*** Cannot send SET_DIFF to pump_cmd_queue[%d]", pn);
                    }
                }

                if (httpd_query_key_value(buf, "sclk", param, sizeof(param)) == ESP_OK) {
                    uint16_t d16 = atoi(param);
                    heat_ctrl[pn].sclk_set = d16;
                    if (heat_ctrl[pn].sclk_rec != d16) { // "CCC:SCLK:DDD"
                        if (put_to_cmd_queue(pn, SET_SCLK, 0)) {
                            ESP_LOGI(TAG, "=== SET_SCLK -> pump_cmd_queue[%d]", pn);
                        } else {
                            ESP_LOGI(TAG, "*** Cannot send SET_SCLK to pump_cmd_queue[%d]", pn);
                        }
                    }
                }

                if (httpd_query_key_value(buf, "lock", param, sizeof(param)) == ESP_OK) {
                    d = atoi(param);
                    heat_ctrl[pn].lock_set = d;
                    if (heat_ctrl[pn].lock_rec != d) { // "CCC:LOCK:[012]"
                        if (put_to_cmd_queue(pn, SET_LOCK, 0)) {
                            ESP_LOGI(TAG, "=== SET_LOCK -> pump_cmd_queue[%d]", pn);
                        } else {
                            ESP_LOGI(TAG, "*** Cannot send SET_LOCK to pump_cmd_queue[%d]", pn);
                        }
                    }
                }                
            }
        }
        free(buf);
    }
    return ESP_OK;
}

// fetch - "/display_val"
static esp_err_t display_get_handler(httpd_req_t *req)
{
    static uint8_t sel = 0;

    uint8_t pn = 0;
    float vbat, press;

    if ((sel == 0) || (sel == 2)  || (sel == 4)) {
        pn = 0;
    } else {
        pn = 1;
    }

    char *jsonbuf = (char *)malloc(256);

    switch (sel)
    {
    case 0:
    case 1:
        vbat = 3.3 * heat_ctrl[pn].vref_data / heat_ctrl[pn].vref_cal;
        press = (heat_ctrl[pn].pressure - vbat / 10) * 1.723662 / 1000;
        if (press < 0) {
            press = 0.0;
        }
        heat_ctrl[pn].press_bar = press;
        sprintf(jsonbuf, "{\"name\": \"%s\", \"pr\": %5.2f, "
            "\"tm\": %5.2f, \"tx\": %5.2f, \"ta\": %5.2f, "
            "\"on\": \"%d\", \"tup\": \"%02d:%02d:%02d\", "
            "\"ct\": \"%02d:%02d:%02d\"}",
            &remote_device_name[pn][5], press,
            heat_ctrl[pn].t_main, heat_ctrl[pn].t_aux, heat_ctrl[pn].t_air, heat_ctrl[pn].pump_on, 
            heat_ctrl[pn].last_start.hour, heat_ctrl[pn].last_start.min, heat_ctrl[pn].last_start.sec, 
            heat_ctrl[pn].ctrl_time.hour, heat_ctrl[pn].ctrl_time.min, heat_ctrl[pn].ctrl_time.sec);
        break;

    case 2:
    case 3:
        sprintf(jsonbuf, "{\"name\": \"%s\","
            "\"rlk\": \"%d\", \"rmod\": %d, \"rper\": \"%02d:%02d\", "
            "\"rnb\": \"%02d:%02d\", \"rne\": \"%02d:%02d\", "
            "\"rdup\": %d, \"rddn\": %d, \"rsclk\": %d}",
            &remote_device_name[pn][5], 
            heat_ctrl[pn].lock_rec, heat_ctrl[pn].mode_rec,
            heat_ctrl[pn].period_rec.hour, heat_ctrl[pn].period_rec.min,
            heat_ctrl[pn].night_begin_rec.hour, heat_ctrl[pn].night_begin_rec.min, 
            heat_ctrl[pn].night_end_rec.hour, heat_ctrl[pn].night_end_rec.min,
            heat_ctrl[pn].delta_on_rec, heat_ctrl[pn].delta_off_rec,
            heat_ctrl[pn].sclk_rec);
        break;
    case 4:
    case 5:
        sprintf(jsonbuf, "{\"name\": \"%s\","
            "\"slk\": \"%d\", \"smod\": %d, \"sper\": \"%02d:%02d\", "
            "\"snb\": \"%02d:%02d\", \"sne\": \"%02d:%02d\", "
            "\"sdup\": %d, \"sddn\": %d, \"ssclk\": %d}",
            &remote_device_name[pn][5], 
            heat_ctrl[pn].lock_set, heat_ctrl[pn].mode_set,
            heat_ctrl[pn].period_set.hour, heat_ctrl[pn].period_set.min,
            heat_ctrl[pn].night_begin_set.hour, heat_ctrl[pn].night_begin_set.min, 
            heat_ctrl[pn].night_end_set.hour, heat_ctrl[pn].night_end_set.min,
            heat_ctrl[pn].delta_on_set, heat_ctrl[pn].delta_off_set,
            heat_ctrl[pn].sclk_set);
        break;
    default:
        sprintf(jsonbuf, "{\"empty\": \"empty\"}");
    }

    if (++sel > 5) {
        sel = 0;
    }

    httpd_resp_send(req, jsonbuf, HTTPD_RESP_USE_STRLEN);

    free(jsonbuf);

    return ESP_OK;
}


esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "404 error - URI is not available");
    return ESP_FAIL;
}


// put command to queue
uint8_t put_to_cmd_queue(uint8_t pnum, uint8_t cmd, uint8_t param)
{
    command_item item;

    item.cmd = cmd;
    item.param = param;

    if (uxQueueSpacesAvailable(pump_cmd_queue[pnum]) > 0)
    {
        xQueueSend(pump_cmd_queue[pnum], (void *)&item, (TickType_t)(50 / portTICK_PERIOD_MS));
        return true;
    } else {
        return false;
    }
}

// restore parameters from file (JSON)
void restore_params(void)
{
    char *line;
    char *bb;
    cJSON *root = NULL;

    uint8_t pn = 7;
    char aa[4];

    ESP_LOGI(TAG, "Reading parameters from %s", INI_FILE);
    FILE* f = fopen(INI_FILE, "r");
    if (f == NULL) { // not exists or corrupted
        ESP_LOGE(TAG, "Failed to open file %s", INI_FILE);
        f = fopen(INI_BAK_FILE, "r"); // attempt to read .BAK file
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file %s", INI_BAK_FILE);
        }
    }

    if (f != NULL) {
        line = (char *)malloc(256);
        *line = '\0';
        bb = (char *)malloc(16);
        for (uint8_t i = 0; i < 2; i++) {
            if (fgets(line, 255, f) != NULL) {
                //ESP_LOGI(TAG, "%s", line);
                root = cJSON_Parse(line);
                if (root != NULL) {
                    cJSON *name =cJSON_GetObjectItem(root,"name");
                    if (cJSON_IsString(name)) {
                        if (strcmp(name->valuestring, &remote_device_name[0][5]) == 0) {
                            pn = 0;
                        } else if (strcmp(name->valuestring, &remote_device_name[1][5]) == 0) {
                            pn = 1;
                        }
                    }
                    if (pn < 2) {
                        cJSON *slk = cJSON_GetObjectItem(root,"slk");
                        if (cJSON_IsNumber(slk)) {
                            heat_ctrl[pn].lock_set = (uint8_t)slk->valueint;
                        }
                        cJSON *smod = cJSON_GetObjectItem(root,"smod");
                        if (cJSON_IsNumber(smod)) {
                            heat_ctrl[pn].mode_set = (uint8_t)smod->valueint;
                        }
                        cJSON *sper = cJSON_GetObjectItem(root,"sper");
                        if (cJSON_IsString(sper)) {
                            strlcpy(bb, sper->valuestring, 6);
                            strncpy(aa, bb, 2);
                            aa[2] = '\0';
                            heat_ctrl[pn].period_set.hour = atoi(aa);        
                            strncpy(aa, &bb[3], 2);
                            aa[2] = '\0';
                            heat_ctrl[pn].period_set.min = atoi(aa);;
                        }
                        cJSON *snb = cJSON_GetObjectItem(root,"snb");
                        if (cJSON_IsString(snb)) {
                            strlcpy(bb, snb->valuestring, 6);
                            strncpy(aa, bb, 2);
                            aa[2] = '\0';
                            heat_ctrl[pn].night_begin_set.hour = atoi(aa);        
                            strncpy(aa, &bb[3], 2);
                            aa[2] = '\0';
                            heat_ctrl[pn].night_begin_set.min = atoi(aa);;
                        }
                        cJSON *sne = cJSON_GetObjectItem(root,"sne");
                        if (cJSON_IsString(sne)) {
                            strlcpy(bb, sne->valuestring, 6);
                            strncpy(aa, bb, 2);
                            aa[2] = '\0';
                            heat_ctrl[pn].night_end_set.hour = atoi(aa);        
                            strncpy(aa, &bb[3], 2);
                            aa[2] = '\0';
                            heat_ctrl[pn].night_end_set.min = atoi(aa);;
                        }
                        cJSON *sdup = cJSON_GetObjectItem(root,"sdup");
                        if (cJSON_IsNumber(sdup)) {
                            heat_ctrl[pn].delta_on_set = (uint8_t)sdup->valueint;
                        }
                        cJSON *sddn = cJSON_GetObjectItem(root,"sddn");
                        if (cJSON_IsNumber(sddn)) {
                            heat_ctrl[pn].delta_off_set = (uint8_t)sddn->valueint;
                        }
                        cJSON *ssclk = cJSON_GetObjectItem(root,"ssclk");
                        if (cJSON_IsNumber(ssclk)) {
                            heat_ctrl[pn].sclk_set = (uint16_t)ssclk->valueint;
                        }
                    }
                }
            }
        }
        free(bb);
        free(line);
        fclose(f);
    } 
}

// save parameters to file
uint8_t save_params(void)
{
    struct stat st;
    if (stat(INI_BAK_FILE, &st) == 0) {
        unlink(INI_BAK_FILE);
    }

    if (stat(INI_FILE, &st) == 0) {
        if (rename(INI_FILE, INI_BAK_FILE) != 0) {
            ESP_LOGE(TAG, "Rename failed");
        }
    }

    FILE* f = fopen(INI_FILE, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open INI_FILE for writing");
        return false;
    } else {
        for (uint8_t i = 0; i < 2; i++) {
            fprintf(f, 
            "{\"name\":\"%s\", \"slk\":%d, \"smod\":%d, \"sper\":\"%02d:%02d\", "
            "\"snb\":\"%02d:%02d\", \"sne\":\"%02d:%02d\", \"sdup\":%d, \"sddn\":%d, \"ssclk\":%d}\n",
            &remote_device_name[i][5], heat_ctrl[i].lock_set, heat_ctrl[i].mode_set,
            heat_ctrl[i].period_set.hour, heat_ctrl[i].period_set.min,
            heat_ctrl[i].night_begin_set.hour, heat_ctrl[i].night_begin_set.min,
            heat_ctrl[i].night_end_set.hour, heat_ctrl[i].night_end_set.min,
            heat_ctrl[i].delta_on_set, heat_ctrl[i].delta_off_set, heat_ctrl[i].sclk_set);
        }
        fclose(f);
        ESP_LOGI(TAG, "== current settings have been saved");
    }
    return true;
}

// save statistics to file
static void periodic_timer_callback(void* arg)
{
    char fname[48];
    time_t now;
    struct tm tm_now;

    time(&now);
    localtime_r(&now, &tm_now);
    sprintf(fname, "%s/%02d-%02d-%02d.csv", MOUNT_POINT,
        tm_now.tm_year - 100, tm_now.tm_mon + 1, tm_now.tm_mday);

    FILE* f = fopen(fname, "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open %s for append", fname);
    } else {        // hh:mm:ss     tm    tx    ta    P    m  du dd  tm    tx    ta    P    m  du dd
        fprintf(f, "%02d:%02d:%02d;%5.2f;%5.2f;%5.2f;%4.2f;%d;%d;%d;%5.2f;%5.2f;%5.2f;%4.2f;%d;%d;%d\n",
            tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec,
            heat_ctrl[0].t_main, heat_ctrl[0].t_aux, heat_ctrl[0].t_air, heat_ctrl[0].pressure,
            heat_ctrl[0].mode_rec, heat_ctrl[0].delta_on_rec, heat_ctrl[0].delta_off_rec,
            heat_ctrl[1].t_main, heat_ctrl[1].t_aux, heat_ctrl[1].t_air, heat_ctrl[1].pressure,
            heat_ctrl[1].mode_rec, heat_ctrl[1].delta_on_rec, heat_ctrl[1].delta_off_rec);
        fclose(f);
        ESP_LOGI(TAG, "== %02d:%02d:%02dcurrent values have been saved",
            tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec);
    }

}

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;

    // queries parameters
    httpd_uri_t display_val = {
        .uri       = "/display_val",
        .method    = HTTP_GET,
        .handler   = display_get_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t set_params = {
        .uri       = "/set_params",
        .method    = HTTP_GET,
        .handler   = params_get_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t common_get_uri = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = common_get_handler,
        .user_ctx = NULL
    };

    httpd_handle_t server = NULL;

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    
    esp_err_t res;
    if ((res = httpd_start(&server, &config)) == ESP_OK) {    
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &display_val);
        httpd_register_uri_handler(server, &set_params);
        httpd_register_uri_handler(server, &common_get_uri);
    } else {
        ESP_LOGE(TAG, "Error starting server! res=%d server= %x", res, (uint32_t)server);
    }
    return server;
}

static void stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    httpd_stop(server);
}

//display string on tft lcd
void lcd_string(char *st, uint8_t nst, uint16_t shift, uint16_t clrshift, uint16_t color)
{
    uint16_t ll = 16 * strlen(st);
    lcdSetFontDirection(&dev, 1);

    int16_t y1 = (width - 1) - 32 * nst;
    int16_t y0 = y1 - fontHeight;
    int16_t x0 = clrshift;
    int16_t x1 = shift + ll;
    lcdDrawFillRect(&dev, y0, x0, y1, x1, BLACK);
    lcdDrawString(&dev, fx, y0, shift, (uint8_t *)st, color);
}

// ================================================================================
void app_main(void)
{
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &periodic_timer_callback,
        .name = "periodic"
    };

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    spi_master_init(&dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO,
                            CONFIG_DC_GPIO, CONFIG_RESET_GPIO, CONFIG_BL_GPIO);

    uint16_t model = 0x9341;
    lcdInit(&dev, model, CONFIG_WIDTH, CONFIG_HEIGHT, CONFIG_OFFSETX, CONFIG_OFFSETY);
    lcdFillScreen(&dev, BLACK);

    ESP_LOGI(TAG, "Initializing SD card");
    ESP_ERROR_CHECK(init_fs());

    //InitFontx(fx16M,"/sdcard/font/ILMH16XB.FNT",""); // 8x16Dot Mincyo
    //InitFontx(fx24M,"/sdcard/font/ILMH24XB.FNT",""); // 12x24Dot Mincyo
    InitFontx(fx32M,"/sdcard/font/ILMH32XB.FNT",""); // 16x32Dot Mincyo
        
    char lbuff[32];
    GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);

    sprintf(lbuff, "Restore params");
    lcd_string(lbuff, 1, 0, 0, YELLOW);
    restore_params();

    // create queues
    pump_cmd_queue[0] = xQueueCreate(CMD_QUEUE_LEN, sizeof(command_item));
    if (pump_cmd_queue[0] == NULL) {
        ESP_LOGE(TAG, "*** Cannot create command queue 0");
    }
    pump_cmd_queue[1] = xQueueCreate(CMD_QUEUE_LEN, sizeof(command_item));
    if (pump_cmd_queue[1] == NULL) {
        ESP_LOGE(TAG, "*** Cannot create command queue 1");
    }
    
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, true, true, portMAX_DELAY);

    // create timer
    esp_timer_handle_t periodic_timer;
    ESP_LOGI(TAG, "Ctreate periodic timer");
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));

    ESP_LOGI(TAG, "starting tasks...");

    sprintf(lbuff, "Start clock_task");
    lcd_string(lbuff, 2, 0, 0, YELLOW);
    xTaskCreatePinnedToCore(clock_task, 
        "ClockUpdate", 4096, ( void * ) 1, tskIDLE_PRIORITY + 5, NULL, tskNO_AFFINITY);

    sprintf(lbuff, "Wait for time sync");
    lcd_string(lbuff, 3, 0, 0, YELLOW);

    uint16_t tcnt = 1000;
    while (!fl_time && (tcnt-- > 0)) // wait for time synchronization
    {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    sprintf(lbuff, "Start timer");
    lcd_string(lbuff, 4, 0, 0, YELLOW);
    // start timer with period of 10 minutes
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 600000000));

    sprintf(lbuff, "Start ble_task...");
    lcd_string(lbuff, 5, 0, 0, YELLOW);
 
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    lcdFillScreen(&dev, BLACK);

    xTaskCreatePinnedToCore(ble_task, 
        "BLEscanner", 4096, ( void * ) 1, tskIDLE_PRIORITY + 5, NULL, tskNO_AFFINITY);

}
