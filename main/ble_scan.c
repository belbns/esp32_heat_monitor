/****************************************************************************
* Nikolay Belov
* 
* Task ble_task
* Scanning two BLE devices, show result on TFT display
*
****************************************************************************/

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "main.h"
#include "ble_scan.h"

#include "ili9340.h"
#include "fontx.h"


#define TAG "GATTC"
#define REMOTE_SERVICE_UUID        0xFFE0
#define REMOTE_NOTIFY_CHAR_UUID    0xFFE1
#define INVALID_HANDLE   0

extern pump_struct heat_ctrl[2];
extern xQueueHandle pump_cmd_queue[2];

extern TFT_t dev;
extern int width;
extern int height;
extern FontxFile fx16M[2];
extern FontxFile fx24M[2];
extern FontxFile fx32M[2];

static void parse_notif(char *not_value, uint16_t not_len, uint8_t curr_dev);
static uint8_t send_cmd(uint8_t curr_dev);

static uint8_t wrk_device = 0;

static bool connect    = false;
static bool get_server = false;
static esp_gattc_char_elem_t *char_elem_result   = NULL;
static esp_gattc_descr_elem_t *descr_elem_result = NULL;

/* Declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);


static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_SERVICE_UUID,},
};

static esp_bt_uuid_t remote_filter_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_NOTIFY_CHAR_UUID,},
};

static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,},
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0xA0,
    .scan_window            = 0x60,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

static struct gattc_profile_inst gl_profile_tab = {
    .gattc_cb = gattc_profile_event_handler,
    .gattc_if = ESP_GATT_IF_NONE
};

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(TAG, "REG_EVT");
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret){
            ESP_LOGE(TAG, "set scan params error, error code = %x", scan_ret);
        }
        break;
    case ESP_GATTC_CONNECT_EVT:{
        ESP_LOGI(TAG, "ESP_GATTC_CONNECT_EVT conn_id %d, if %d", p_data->connect.conn_id, gattc_if);
        gl_profile_tab.conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab.remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(TAG, "REMOTE BDA:");
        esp_log_buffer_hex(TAG, gl_profile_tab.remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->connect.conn_id);
        if (mtu_ret){
            ESP_LOGE(TAG, "config MTU error, error code = %x", mtu_ret);
        }
        break;
    }
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK){
            ESP_LOGE(TAG, "open failed, status %d", p_data->open.status);
            break;
        }
        ESP_LOGI(TAG, "open success");
        break;
    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(TAG, "discover service failed, status %d", param->dis_srvc_cmpl.status);
            break;
        }
        ESP_LOGI(TAG, "discover service complete conn_id %d", param->dis_srvc_cmpl.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &remote_filter_service_uuid);
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK){
            ESP_LOGE(TAG,"config mtu failed, error status = %x", param->cfg_mtu.status);
        }
        ESP_LOGI(TAG, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        ESP_LOGI(TAG, "SEARCH RES: conn_id = %x is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(TAG, "start handle %d end handle %d current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
            ESP_LOGI(TAG, "service found");
            get_server = true;
            gl_profile_tab.service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab.service_end_handle = p_data->search_res.end_handle;
            ESP_LOGI(TAG, "UUID16: %x", p_data->search_res.srvc_id.uuid.uuid.uuid16);
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(TAG, "search service failed, error status = %x", p_data->search_cmpl.status);
            break;
        }
        if(p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE) {
            ESP_LOGI(TAG, "Get service information from remote device");
        } else if (p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH) {
            ESP_LOGI(TAG, "Get service information from flash");
        } else {
            ESP_LOGI(TAG, "unknown service source");
        }
        ESP_LOGI(TAG, "ESP_GATTC_SEARCH_CMPL_EVT");
        if (get_server){
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count( 
                gattc_if, p_data->search_cmpl.conn_id,
                ESP_GATT_DB_CHARACTERISTIC,
                gl_profile_tab.service_start_handle,
                gl_profile_tab.service_end_handle,
                INVALID_HANDLE, &count);

            if (status != ESP_GATT_OK){
                ESP_LOGE(TAG, "esp_ble_gattc_get_attr_count error");
            }

            if (count > 0){
                char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result){
                    ESP_LOGE(TAG, "gattc no mem");
                }else{
                    status = esp_ble_gattc_get_char_by_uuid( 
                        gattc_if, p_data->search_cmpl.conn_id,
                        gl_profile_tab.service_start_handle,
                        gl_profile_tab.service_end_handle,
                        remote_filter_char_uuid,
                        char_elem_result, &count);

                    if (status != ESP_GATT_OK){
                        ESP_LOGE(TAG, "esp_ble_gattc_get_char_by_uuid error");
                    }

                    /*  Every service have only one char in our 'ESP_GATTS_DEMO' demo, so we used first 'char_elem_result' */
                    if (count > 0 && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)){
                        gl_profile_tab.char_handle = char_elem_result[0].char_handle;
                        esp_ble_gattc_register_for_notify(
                            gattc_if, 
                            gl_profile_tab.remote_bda, char_elem_result[0].char_handle);
                    }
                }
                /* free char_elem_result */
                free(char_elem_result);
            }else{
                ESP_LOGE(TAG, "no char found");
            }
        }
         break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        ESP_LOGI(TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT");
        if (p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(TAG, "REG FOR NOTIFY failed: error status = %d", p_data->reg_for_notify.status);
        }else{
            uint16_t count = 0;
            uint16_t notify_en = 1;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( 
                gattc_if,
                gl_profile_tab.conn_id,
                ESP_GATT_DB_DESCRIPTOR,
                gl_profile_tab.service_start_handle,
                gl_profile_tab.service_end_handle,
                gl_profile_tab.char_handle,
               &count);
            if (ret_status != ESP_GATT_OK){
                ESP_LOGE(TAG, "esp_ble_gattc_get_attr_count error");
            }
            if (count > 0){
                descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result){
                    ESP_LOGE(TAG, "malloc error, gattc no mem");
                }else{
                    ret_status = esp_ble_gattc_get_descr_by_char_handle( 
                        gattc_if,
                        gl_profile_tab.conn_id,
                        p_data->reg_for_notify.handle,
                        notify_descr_uuid, descr_elem_result, &count);

                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                    }
                    /* Every char has only one descriptor in our 'ESP_GATTS_DEMO' demo, so we used first 'descr_elem_result' */
                    if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG){
                        ret_status = esp_ble_gattc_write_char_descr(
                            gattc_if,
                            gl_profile_tab.conn_id,
                            descr_elem_result[0].handle,
                            sizeof(notify_en), (uint8_t *)&notify_en,
                            ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
                    }

                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(TAG, "esp_ble_gattc_write_char_descr error");
                    }

                    /* free descr_elem_result */
                    free(descr_elem_result);
                }
            }
            else{
                ESP_LOGE(TAG, "decsr not found");
            }

        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify){
            ESP_LOGI(TAG, "ESP_GATTC_NOTIFY_EVT, receive notify value:");
        }else{
            ESP_LOGI(TAG, "ESP_GATTC_NOTIFY_EVT, receive indicate value:");
        }
        //esp_log_buffer_hex(TAG, p_data->notify.value, p_data->notify.value_len);
        parse_notif((char *)p_data->notify.value, p_data->notify.value_len, wrk_device);
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(TAG, "write descr failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(TAG, "write descr success ");
        break;
    case ESP_GATTC_SRVC_CHG_EVT: {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
        esp_log_buffer_hex(TAG, bda, sizeof(esp_bd_addr_t));
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(TAG, "write char failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(TAG, "write char success ");
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        connect = false;
        get_server = false;
        ESP_LOGI(TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
        break;
    default:
        break;
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;

    //char buf[32];

    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        //the unit of the duration is second
        uint32_t duration = 30;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(TAG, "scan start success");

        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            esp_log_buffer_hex(TAG, scan_result->scan_rst.bda, 6);
            ESP_LOGI(TAG, "searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            ESP_LOGI(TAG, "searched Device Name Len %d", adv_name_len);
            //esp_log_buffer_char(TAG, adv_name, adv_name_len);
            //strlcpy(buf, (char *)adv_name, adv_name_len + 1);
            //ESP_LOGI(TAG, "adv_name: %s len:%d", buf, adv_name_len);

#if CONFIG_EXAMPLE_DUMP_ADV_DATA_AND_SCAN_RESP
            if (scan_result->scan_rst.adv_data_len > 0) {
                ESP_LOGI(TAG, "adv data:");
                esp_log_buffer_hex(TAG, &scan_result->scan_rst.ble_adv[0], scan_result->scan_rst.adv_data_len);
            }
            if (scan_result->scan_rst.scan_rsp_len > 0) {
                ESP_LOGI(TAG, "scan resp:");
                esp_log_buffer_hex(TAG, &scan_result->scan_rst.ble_adv[scan_result->scan_rst.adv_data_len], scan_result->scan_rst.scan_rsp_len);
            }
#endif
            ESP_LOGI(TAG, "\n");

            if (adv_name != NULL) {
                if (strlen(remote_device_name[wrk_device]) == adv_name_len &&
                    strncmp((char *)adv_name, remote_device_name[wrk_device], adv_name_len) == 0) {
                    ESP_LOGI(TAG, "searched device %s\n", remote_device_name[wrk_device]);
                    if (connect == false) {
                        connect = true;
                        ESP_LOGI(TAG, "connect to the remote device.");
                        esp_ble_gap_stop_scanning();
                        esp_ble_gattc_open(
                            gl_profile_tab.gattc_if, 
                            scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                    }
                }
            }
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            break;
        default:
            break;
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(TAG, "scan stop failed, error status = %x", param->scan_stop_cmpl.status);
            break;
        }
        ESP_LOGI(TAG, "stop scan successfully");
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(TAG, "adv stop failed, error status = %x", param->adv_stop_cmpl.status);
            break;
        }
        ESP_LOGI(TAG, "stop adv successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab.gattc_if = gattc_if;
        } else {
            ESP_LOGI(TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    // ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call profile cb function
    if (gattc_if == ESP_GATT_IF_NONE || gattc_if == gl_profile_tab.gattc_if) {
        if (gl_profile_tab.gattc_cb) {
            gl_profile_tab.gattc_cb(event, gattc_if, param);
        }
    }
}

void ble_task(void * pvParameters)
{
    ( void ) pvParameters;

    ESP_LOGI(TAG, "== ble_task started");

    time_t now;
    struct tm timeinfo;

    uint8_t buffer[FontxGlyphBufSize];
    uint8_t fontWidth;
    uint8_t fontHeight;

    FontxFile *fx = fx32M;
    GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);

    uint16_t color = CYAN;
    //lcdFillScreen(&dev, BLACK);
    uint8_t ascii[32];
    uint8_t hh = 0; uint8_t mm = 0;

    sprintf((char *)ascii, "P,bar:"); 
    lcdSetFontDirection(&dev, 1);
    lcdDrawString(&dev, fx, (width-1)-fontHeight, 16 * 10, ascii, color);
    sprintf((char *)ascii, "  t(m)  t(x)  t(a)");
    lcdDrawString(&dev, fx, (width-1)-fontHeight - 32, 0, ascii, color);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    //register the  callback function to the gap module
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret){
        ESP_LOGE(TAG, "%s gap register failed, error code = %x\n", __func__, ret);
        return;
    }

    //register the callback function to the gattc module
    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if(ret){
        ESP_LOGE(TAG, "%s gattc register failed, error code = %x\n", __func__, ret);
        return;
    }
    /*
    ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    if (ret){
        ESP_LOGE(TAG, "%s gattc app register failed, error code = %x\n", __func__, ret);
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    */
    uint16_t delay_cnt = 30;
    uint8_t err_cnt[2] = {0, 0};

    esp_ble_gap_stop_scanning();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    connect = 0;

    uint16_t cnt = 1000;
    for(;;)
    {   
        wrk_device = 0;
        do {
            ret = esp_ble_gattc_app_register(0);
            if (ret){
                ESP_LOGE(TAG, "%s gattc app register failed, error code = %x\n", __func__, ret);
            } else {
                cnt = 1000;
                while (!connect && (cnt-- > 0)) {
                    vTaskDelay(25 / portTICK_PERIOD_MS);
                }

                cnt = 1000;
                while ((heat_ctrl[wrk_device].rec_flags != 0x1f) && (cnt-- > 0)) {
                    vTaskDelay(25 / portTICK_PERIOD_MS);
                }
                ESP_LOGI(TAG, "0 == flags[%d]:%d cnt:%d", wrk_device, heat_ctrl[0].rec_flags, cnt);

                if (heat_ctrl[wrk_device].rec_flags != 0x1f) {
                    err_cnt[wrk_device]++;
                } else {
                    send_cmd(wrk_device);
                    if (err_cnt[wrk_device] > 0) {
                        err_cnt[wrk_device]--;
                    }
                }
            }
            vTaskDelay(500 / portTICK_PERIOD_MS);
            esp_ble_gattc_close(gl_profile_tab.gattc_if, gl_profile_tab.conn_id);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            esp_ble_gap_disconnect(gl_profile_tab.remote_bda);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            esp_ble_gattc_app_unregister(gl_profile_tab.gattc_if);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            wrk_device++;
        } while (wrk_device < 2);

        ESP_LOGI(TAG, "\n == heap_size=%d stack=%d",
            esp_get_free_heap_size(), uxTaskGetStackHighWaterMark(NULL));

        if ((uxQueueMessagesWaiting(pump_cmd_queue[0]) > 0) ||
            (uxQueueMessagesWaiting(pump_cmd_queue[1]) > 0) || 
            (err_cnt[0] > 0) || (err_cnt[1] > 0)) {
            delay_cnt = 15;
        } else {
            delay_cnt = 30;
        }

        ESP_LOGI(TAG, "\n== pause %d sec", delay_cnt);

        color = BLUE;
        for (uint16_t i = 0; i < delay_cnt; i++) {
            time(&now);
            localtime_r(&now, &timeinfo);
            sprintf((char *)ascii, "%02u:%02u:%02u", 
              timeinfo.tm_hour & 0x3f, timeinfo.tm_min & 0x3f, timeinfo.tm_sec & 0x3f);
            uint16_t xb = 16 * 6;
            if (timeinfo.tm_min != mm) {
                xb = 16 * 3;
            }
            if (timeinfo.tm_hour != hh) {
                    xb = 0;
            }
            hh = timeinfo.tm_hour;
            mm = timeinfo.tm_min;

            lcdDrawFillRect(&dev, (width-1)-fontHeight, xb, (width-1), 16 * 8, BLACK);
            lcdDrawString(&dev, fx, (width-1)-fontHeight, 0, ascii, color);

            if ((i % 5) == 0) {
                if ((heat_ctrl[0].press_bar >= 0.7) && (heat_ctrl[0].press_bar < 1.0))
                {
                    color = YELLOW;
                } else if (heat_ctrl[0].press_bar < 0.7) {
                    color = RED;
                } else {
                    color = BLUE;
                }
                sprintf((char *)ascii, "%4.2f", heat_ctrl[0].press_bar);
                lcdDrawFillRect(&dev, (width-1)-fontHeight, 16 * 16, (width-1), 16 * 20 - 1, BLACK);
                lcdDrawString(&dev, fx, (width-1)-fontHeight, 16 * 16, ascii, color);

                if (heat_ctrl[0].pump_on) {
                    color = YELLOW;
                } else {
                    color = BLUE;
                }
                sprintf((char *)ascii, 
                    "E %5.2f %5.2f %5.2f", heat_ctrl[0].t_main, heat_ctrl[0].t_aux, heat_ctrl[0].t_air);
                lcdDrawFillRect(&dev, (width-1)-fontHeight - 32 * 2, 0, (width-1) - 32 * 2, (height-1), BLACK);
                lcdDrawString(&dev, fx, (width-1)-fontHeight - 32 * 2, 0, ascii, color);

                if (heat_ctrl[1].pump_on) {
                    color = YELLOW;
                } else {
                    color = BLUE;
                }
                sprintf((char *)ascii, 
                    "W %5.2f %5.2f %5.2f", heat_ctrl[1].t_main, heat_ctrl[1].t_aux, heat_ctrl[1].t_air);
                lcdDrawFillRect(&dev, (width-1)-fontHeight - 32 * 3, 0, (width-1) - 32 * 3, (height-1), BLACK);
                lcdDrawString(&dev, fx, (width-1)-fontHeight - 32 * 3, 0, ascii, color);
            }

            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        heat_ctrl[0].rec_flags = 0;
        heat_ctrl[1].rec_flags = 0;
    }
}

static void parse_notif(char *not_value, uint16_t not_len, uint8_t curr_dev)
{
    char tbuff[40];

    strlcpy(tbuff, not_value, not_len);
    ESP_LOGI(TAG, "dev:%d notif:%s", curr_dev, tbuff);

    char *token, *ptr;
    char st1[4];

    ptr = tbuff;
    token = strsep(&ptr, ":");
    if (token != NULL) {
        uint8_t pknum = atoi(token); // номер пакета
        if (pknum < 5) {

            switch (pknum)
            {
            case 0: // 0:1:81:122451:1522
                    token = strsep(&ptr, ":");
                if (token != NULL) {
                    uint8_t m = atoi(token); // mode
                    if (m < 2) {
                        heat_ctrl[curr_dev].mode_rec = m;
                    }
                    token = strsep(&ptr, ":");
                    if ((token != NULL) && (strlen(token) == 2)) { // delta
                        uint8_t d = (uint8_t)token[0] - 0x30;
                        heat_ctrl[curr_dev].delta_on_rec = d;
                        d = (uint8_t)token[1] - 0x30;
                        heat_ctrl[curr_dev].delta_off_rec = d;
                        token = strsep(&ptr, ":");
                        if ((token != NULL) && (strlen(token) == 6)) {
                            strncpy(st1, token, 2);
                            st1[2] = '\0';
                            heat_ctrl[curr_dev].ctrl_time.hour = atoi(st1);
                            strncpy(st1, token + 2, 2);
                            st1[2] = '\0';
                            heat_ctrl[curr_dev].ctrl_time.min = atoi(st1);
                            strncpy(st1, token + 4, 2);
                            st1[2] = '\0';
                            heat_ctrl[curr_dev].ctrl_time.sec = atoi(st1);
                            token = strsep(&ptr, ":");
                            heat_ctrl[curr_dev].tm_updated = 1;
                            if (token != NULL) {
                                heat_ctrl[curr_dev].vref_cal = atoi(token);
                            }
                        }
                    }
                    heat_ctrl[curr_dev].rec_flags |= 1;
                }
                break;
            case 1: // 1:0220:2315:0715
                token = strsep(&ptr, ":");
                if ((token != NULL) && (strlen(token) == 4)) {
                    uint8_t hh, mm;
                    strncpy(st1, token, 2);
                    st1[2] = '\0';
                    hh = atoi(st1);
                    heat_ctrl[curr_dev].period_rec.hour = hh;
                    strncpy(st1, token + 2, 2);
                    st1[2] = '\0';
                    mm = atoi(st1);
                    heat_ctrl[curr_dev].period_rec.min = mm;
                    //ESP_LOGI(TAG, "=1 case 1: hh=%d mm=%d", hh, mm);
                    token = strsep(&ptr, ":");
                    if ((token != NULL) && (strlen(token) == 4)) {
                        strncpy(st1, token, 2);
                        st1[2] = '\0';
                        hh = atoi(st1);
                        heat_ctrl[curr_dev].night_begin_rec.hour = hh;
                        strncpy(st1, token + 2, 2);
                        st1[2] = '\0';
                        mm = atoi(st1);
                        heat_ctrl[curr_dev].night_begin_rec.min = mm;
                        //ESP_LOGI(TAG, "=2 case 1: hh=%d mm=%d", hh, mm);
                        token = strsep(&ptr, ":");
                        if ((token != NULL) && (strlen(token) > 3)) {
                            strncpy(st1, token, 2);
                            st1[2] = '\0';
                            hh = atoi(st1);
                            heat_ctrl[curr_dev].night_end_rec.hour = hh;
                            strncpy(st1, token + 2, 2);
                            st1[2] = '\0';
                            mm = atoi(st1);
                            heat_ctrl[curr_dev].night_end_rec.min = mm;
                            //ESP_LOGI(TAG, "=3 case 1: hh=%d mm=%d", hh, mm);
                        }
                    }
                    heat_ctrl[curr_dev].rec_flags |= 2;
                }
                break;
            case 2: // 2:30.50:22.75:18.00
                token = strsep(&ptr, ":");
                if (token != NULL) {
                    heat_ctrl[curr_dev].t_main = atof(token);
                    token = strsep(&ptr, ":");
                    if (token != NULL) {
                        heat_ctrl[curr_dev].t_aux = atof(token);
                        token = strsep(&ptr, ":");
                        if (token != NULL) {
                            heat_ctrl[curr_dev].t_air = atof(token);
                        }
                    }
                    heat_ctrl[curr_dev].rec_flags |= 4;
                }
                break;
            case 3: // "3:VVVV:PPP:L:SS:DDD\n"
                token = strsep(&ptr, ":");
                if (token != NULL) {
                    heat_ctrl[curr_dev].vref_data = atoi(token);
                    token = strsep(&ptr, ":");
                    if (token != NULL) {
                        heat_ctrl[curr_dev].pressure = 1.0 * atoi(token); //??
                        token = strsep(&ptr, ":");
                        if (token != NULL) {
                            heat_ctrl[curr_dev].lock_rec = atoi(token);
                            token = strsep(&ptr, ":");
                            if (token != NULL) {
                                heat_ctrl[curr_dev].state = atoi(token);
                                token = strsep(&ptr, ":");
                                if (token != NULL) {
                                    heat_ctrl[curr_dev].sclk_rec = atoi(token);
                                }
                            }
                        }
                    }
                    heat_ctrl[curr_dev].rec_flags |= 8;
                }
                break;
            case 4: // 4:0:112001:99:99:99
                token = strsep(&ptr, ":");
                if (token != NULL) {
                    heat_ctrl[curr_dev].pump_on = atoi(token);
                    token = strsep(&ptr, ":");
                    if ((token != NULL) && (strlen(token) == 6)) {
                        strncpy(st1, token, 2);
                        st1[2] = '\0';
                        heat_ctrl[curr_dev].last_start.hour = atoi(st1);
                        strncpy(st1, token + 2, 2);
                        st1[2] = '\0';
                        heat_ctrl[curr_dev].last_start.min = atoi(st1);
                        strncpy(st1, token + 4, 2);
                        st1[2] = '\0';
                        heat_ctrl[curr_dev].last_start.sec = atoi(st1);                            
                        token = strsep(&ptr, ":");
                        if (token != NULL) {
                            heat_ctrl[curr_dev].err_main = atoi(token);
                            token = strsep(&ptr, ":");
                            if (token != NULL) {
                                heat_ctrl[curr_dev].err_aux = atoi(token);
                                token = strsep(&ptr, ":");
                                if (token != NULL) {
                                    heat_ctrl[curr_dev].err_air = atoi(token);
                                }
                            }
                        }
                    }
                    heat_ctrl[curr_dev].rec_flags |= 16;
                }
            }
            heat_ctrl[curr_dev].updated = 1;
        }
    }
}

static uint8_t send_cmd(uint8_t curr_dev)
{
    time_t now;
    struct tm timeinfo;
    char snd_data[32];

    command_item item;
    uint8_t fl_cmd = false;

    if (xQueueReceive(pump_cmd_queue[curr_dev], &item, (TickType_t)(10 / portTICK_PERIOD_MS)) == pdPASS) {
        ESP_LOGI(TAG, " == dev=%d cmd=%d", curr_dev, item.cmd);
        fl_cmd = true;
        switch (item.cmd)
        {
        case SET_MODE: // "CCC:SMOD:[01]:HHmm"
            sprintf(snd_data, "cccccc:smod:%d:%02d%02d\n",
                    heat_ctrl[curr_dev].mode_set, 
                    heat_ctrl[curr_dev].period_set.hour,
                    heat_ctrl[curr_dev].period_set.min);
            break;
        case SET_NIGHT: // "CCC:NSET:HHMM:hhmm"
            sprintf(snd_data, "ccc:nset:%02d%02d:%02d%02d\n",
                    heat_ctrl[curr_dev].night_begin_set.hour,
                    heat_ctrl[curr_dev].night_begin_set.min,
                    heat_ctrl[curr_dev].night_end_set.hour,
                    heat_ctrl[curr_dev].night_end_set.min);
            break;
        case SET_LOCK: // "CCC:LOCK:[012]"
            sprintf(snd_data, "ccccccccccc:lock:%d\n", heat_ctrl[curr_dev].lock_set); 
            break;
        case SET_DIFF: // "CCC:DIFF:U:D
            sprintf(snd_data, "ccccccccc:diff:%d:%d\n", 
                    heat_ctrl[curr_dev].delta_on_set, heat_ctrl[curr_dev].delta_off_set); 
            break;
        case SET_SCLK: // "CCC:SCLK:DDD"
            sprintf(snd_data, "ccccccccc:sclk:%d\n", heat_ctrl[curr_dev].sclk_set); 
            break;
        case SET_TIME: // "CCC:TSET:HHmmSS"
            time(&now);
            localtime_r(&now, &timeinfo);
            sprintf(snd_data, "cccccc:tset:%02d%02d%02d\n",
                    timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
            break;
        default:
            fl_cmd = false;
        }

        if (fl_cmd) {
            ESP_LOGI(TAG, " == dev=%d snd_data=%s", curr_dev, snd_data);
            esp_ble_gattc_write_char(gl_profile_tab.gattc_if,
            gl_profile_tab.conn_id, gl_profile_tab.char_handle,
            sizeof(snd_data), (uint8_t *)snd_data,
            ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
            return true;
        }
    }
    return false;
}
