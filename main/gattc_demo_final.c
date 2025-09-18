/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/****************************************************************************
*
* This demo showcases BLE GATT client with optimized display integration.
* 优化版本：解决蓝牙和显示同时运行时的刷屏和冲突问题
*
****************************************************************************/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_memory_utils.h"
#include "esp_psram.h"
#include "esp_heap_caps.h"
#include "lvgl.h"
#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "lv_demos.h"
#include "esp_system.h"
#include "esp_random.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "freertos/semphr.h"

// 同步机制
SemaphoreHandle_t conn_status_mutex;
QueueHandle_t ui_data_queue;
EventGroupHandle_t ui_event_group;

// 事件位定义
#define UI_UPDATE_STATUS_BIT    BIT0
#define UI_UPDATE_DATA_BIT      BIT1

#define GATTC_TAG "GATTC_DEMO"
#define REMOTE_SERVICE_UUID        0x00FF

#define PROFILE_NUM      1
#define PROFILE_A_APP_ID 0
#define INVALID_HANDLE   0

// UI数据结构
typedef struct {
    uint8_t wave_power[5];    // δ, θ, α, β, γ
    uint8_t metric_values[4]; // 专注, 放松, 疲劳, 冥想
    bool is_wave_data;        // true=波段数据, false=指标数据
} ui_data_t;

LV_FONT_DECLARE(GenJyuuGothic_32);//字体

//******************************************************************************** */
//需要展示的脑电参数和显示参数初始化********************************************************** */
//******************************************************************************** */

typedef enum {
    CONN_STATUS_DISCONNECTED, // 未连接
    CONN_STATUS_CONNECTING,   // 连接中
    CONN_STATUS_CONNECTED     // 已连接
} conn_status_t;

static volatile conn_status_t conn_status = CONN_STATUS_DISCONNECTED;
static lv_obj_t *root_ui = NULL; // 当前界面的根对象

// 结构体存放每行的控件
typedef struct {
    lv_obj_t *label;   // 左边文字标签
    lv_obj_t *value;   // 数值显示
    lv_obj_t *bar;     // 横向条
} wave_ui_t;
static wave_ui_t waves[5];  // 5行
static const char *wave_names[5] = {"δ波", "θ波", "α波", "β波", "γ波"};

typedef struct {
    lv_obj_t *title;
    lv_obj_t *value;
} metric_ui_t;
static metric_ui_t metrics[4];
static const char *metric_names[4] = {"专注", "放松", "疲劳", "冥想"};

//******************************************************************************** */
//蓝牙相关函数和参数初始化********************************************************** */
//******************************************************************************** */

// 目标设备名字
static char remote_device_name[ESP_BLE_ADV_NAME_LEN_MAX] = "KSEEG102";

typedef struct {
    uint8_t remote_service_uuid[16]; // 128-bit UUID
    uint16_t start_handle;           // 服务的起始句柄
    uint16_t end_handle;             // 服务的结束句柄
    uint16_t char_handle;            // 特征句柄（可选）
} service_info_t;

static bool connect    = false;
static bool get_server = false;
static esp_gattc_char_elem_t *char_elem_result   = NULL;
static esp_gattc_descr_elem_t *descr_elem_result = NULL;

/* Declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid = {.uuid128 = {0x40, 0xe3, 0x4a, 0x1d,0xc2, 0x5f, 0xb0, 0x9c,0xb7, 0x47, 0xe6, 0x43,0x0a, 0x00, 0x53, 0x86},
    },
};

// EEG 特征值接收 UUID: 0x8653000d-43e6-47b7-9cb0-5fc21d4ae340
static const esp_bt_uuid_t uuid_eeg_feature = {
    .len = ESP_UUID_LEN_128,
    .uuid.uuid128 = {0x40,0xe3,0x4a,0x1d,0xc2,0x5f,0xb0,0x9c,
                     0xb7,0x47,0xe6,0x43,0x0d,0x00,0x53,0x86}
};

// EEG 频谱值接收 UUID: 0x8653000e-43e6-47b7-9cb0-5fc21d4ae340
static const esp_bt_uuid_t uuid_eeg_spectrum = {
    .len = ESP_UUID_LEN_128,
    .uuid.uuid128 = {0x40,0xe3,0x4a,0x1d,0xc2,0x5f,0xb0,0x9c,
                     0xb7,0x47,0xe6,0x43,0x0e,0x00,0x53,0x86}
};

static const esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,},
};

static esp_bt_uuid_t target_uuids[] = {
    uuid_eeg_feature,   // 特征值（专注度，放松度，疲劳度，冥想值）
    uuid_eeg_spectrum,  // 频谱（δ，θ，α，β，γ）
};
#define target_uuid_num 2

// 存储句柄数组
static uint16_t char_handles[target_uuid_num] = {0};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
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
    uint16_t cmd_rw_handle;
    uint16_t eeg_raw_handle;
    uint16_t eeg_feature_handle;
    uint16_t eeg_spectrum_handle;
    uint16_t sensor_handle;
    uint16_t blink_bite_handle;
};

static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,
    },
};

// 安全设置连接状态
static void set_conn_status(conn_status_t new_status) {
    if (xSemaphoreTake(conn_status_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (conn_status != new_status) {
            conn_status = new_status;
            xEventGroupSetBits(ui_event_group, UI_UPDATE_STATUS_BIT);
        }
        xSemaphoreGive(conn_status_mutex);
    }
}

// 安全获取连接状态
static conn_status_t get_conn_status(void) {
    conn_status_t status = CONN_STATUS_DISCONNECTED;
    if (xSemaphoreTake(conn_status_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        status = conn_status;
        xSemaphoreGive(conn_status_mutex);
    }
    return status;
}

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "GATT client register, status %d, app_id %d, gattc_if %d", param->reg.status, param->reg.app_id, gattc_if);
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret){
            ESP_LOGE(GATTC_TAG, "set scan params error, error code = %x", scan_ret);
        }
        break;
    case ESP_GATTC_CONNECT_EVT:{
        ESP_LOGI(GATTC_TAG, "Connected, conn_id %d, remote "ESP_BD_ADDR_STR"", p_data->connect.conn_id,
                 ESP_BD_ADDR_HEX(p_data->connect.remote_bda));
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->connect.conn_id);
        if (mtu_ret){
            ESP_LOGE(GATTC_TAG, "Config MTU error, error code = %x", mtu_ret);
        }
        break;
    }
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Open failed, status %d", p_data->open.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Open successfully, MTU %u", p_data->open.mtu);
        break;
    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Service discover failed, status %d", param->dis_srvc_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Service discover complete, conn_id %d", param->dis_srvc_cmpl.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->dis_srvc_cmpl.conn_id, &remote_filter_service_uuid);
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        ESP_LOGI(GATTC_TAG, "MTU exchange, status %d, MTU %d", param->cfg_mtu.status, param->cfg_mtu.mtu);
        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        ESP_LOGI(GATTC_TAG, "Service search result, conn_id = %x, is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(GATTC_TAG, "start handle %d, end handle %d, current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        esp_gatt_id_t *srvc_id = &p_data->search_res.srvc_id;
        ESP_LOGI(GATTC_TAG, "服务信息打印开始↓");
        if (srvc_id->uuid.len == ESP_UUID_LEN_16) {
            ESP_LOGI(GATTC_TAG, "Service UUID16: 0x%04x", (unsigned int)srvc_id->uuid.uuid.uuid16);
        } else if (srvc_id->uuid.len == ESP_UUID_LEN_32) {
            ESP_LOGI(GATTC_TAG, "Service UUID32: 0x%08" PRIx32, srvc_id->uuid.uuid.uuid32);
        } else if (srvc_id->uuid.len == ESP_UUID_LEN_128) {
            ESP_LOGI(GATTC_TAG, "Service UUID128:");
            ESP_LOG_BUFFER_HEX(GATTC_TAG, srvc_id->uuid.uuid.uuid128, 16);
            if (memcmp(srvc_id->uuid.uuid.uuid128, remote_filter_service_uuid.uuid.uuid128, 16) == 0) {
                ESP_LOGI(GATTC_TAG, "Matched target service UUID!");
                gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = p_data->search_res.start_handle;
                gl_profile_tab[PROFILE_A_APP_ID].service_end_handle   = p_data->search_res.end_handle;
                get_server = true;
        }
        }
        ESP_LOGI(GATTC_TAG, "服务信息打印完毕↑");
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Service search failed, status %x", p_data->search_cmpl.status);
            break;
        }
        if(p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE) {
            ESP_LOGI(GATTC_TAG, "Get service information from remote device");
        } else if (p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH) {
            ESP_LOGI(GATTC_TAG, "Get service information from flash");
        } else {
            ESP_LOGI(GATTC_TAG, "Unknown service source");
        }
        ESP_LOGI(GATTC_TAG, "Service search complete");
        if (get_server){
            ESP_LOGI(GATTC_TAG, "Get Server");
            
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     p_data->search_cmpl.conn_id,
                                                                     ESP_GATT_DB_CHARACTERISTIC,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                     INVALID_HANDLE,
                                                                     &count);
            if (count > 0 && status == ESP_GATT_OK){
                char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result){
                    ESP_LOGE(GATTC_TAG, "gattc no mem");
                    break;
                }else{
                    for (int i = 0; i < target_uuid_num; i++) {
                        status = esp_ble_gattc_get_char_by_uuid( gattc_if,
                                                                p_data->search_cmpl.conn_id,
                                                                gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                target_uuids[i],
                                                                char_elem_result,
                                                                &count);

                        if (count > 0 && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)){
                            gl_profile_tab[PROFILE_A_APP_ID].char_handle = char_elem_result[0].char_handle;
                            char_handles[i] = char_elem_result[0].char_handle;

                            ESP_LOGI(GATTC_TAG, "已获得特征句柄: %d", gl_profile_tab[PROFILE_A_APP_ID].char_handle);

                            esp_ble_gattc_register_for_notify (gattc_if, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, gl_profile_tab[PROFILE_A_APP_ID].char_handle);
                        }
                    }
                }
            }else{
                ESP_LOGE(GATTC_TAG, "No characteristic found");
            }
        }
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        if (p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Register for notify failed, error status %x", p_data->reg_for_notify.status);
            break;
        }
        uint16_t count = 0;
        uint16_t notify_en = 1;
        esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                     ESP_GATT_DB_DESCRIPTOR,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                                     &count);
        if (ret_status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
        }
        if (count > 0){
            descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
            if (!descr_elem_result){
                ESP_LOGE(GATTC_TAG, "malloc error, gattc no mem");
            }else{
                ret_status = esp_ble_gattc_get_descr_by_char_handle( gattc_if,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                     p_data->reg_for_notify.handle,
                                                                     notify_descr_uuid,
                                                                     descr_elem_result,
                                                                     &count);
                if (ret_status != ESP_GATT_OK){
                    ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                }
                if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG){
                    ret_status = esp_ble_gattc_write_char_descr( gattc_if,
                                                                 gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                 descr_elem_result[0].handle,
                                                                 sizeof(notify_en),
                                                                 (uint8_t *)&notify_en,
                                                                 ESP_GATT_WRITE_TYPE_RSP,
                                                                 ESP_GATT_AUTH_REQ_NONE);
                }

                if (ret_status != ESP_GATT_OK){
                    ESP_LOGE(GATTC_TAG, "esp_ble_gattc_write_char_descr error");
                }
                free(descr_elem_result);
            }
        }
        else{
            ESP_LOGE(GATTC_TAG, "decsr not found");
        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify){
            ESP_LOGI(GATTC_TAG, "Received notification, handle: %d", p_data->notify.handle);
        }else{
            ESP_LOGI(GATTC_TAG, "Received indication, handle: %d", p_data->notify.handle);
        }

        // 确定接收到的数据类型
        int handle_index = -1;
        for (int i = 0; i < target_uuid_num; i++) {
            if (char_handles[i] == p_data->notify.handle) {
                handle_index = i;
                break;
            }
        }

        ui_data_t ui_data;
        switch (handle_index) {
        case 0: // EEG 特征值接收
            ESP_LOGI(GATTC_TAG, "收到特征数据");
            if (p_data->notify.value_len >= 4) {
                ui_data.is_wave_data = false;
                ui_data.metric_values[0] = p_data->notify.value[0];  // 专注
                ui_data.metric_values[1] = p_data->notify.value[1];  // 放松  
                ui_data.metric_values[2] = p_data->notify.value[2];  // 疲劳
                ui_data.metric_values[3] = p_data->notify.value[3];  // 冥想

                ESP_LOGI(GATTC_TAG, "专注: %d, 放松: %d, 疲劳: %d, 冥想: %d",
                        ui_data.metric_values[0], ui_data.metric_values[1], 
                        ui_data.metric_values[2], ui_data.metric_values[3]);
                
                // 发送到UI队列
                if (xQueueSend(ui_data_queue, &ui_data, 0) != pdTRUE) {
                    ESP_LOGW(GATTC_TAG, "UI队列满，丢弃特征数据");
                }
            } else {
                ESP_LOGW(GATTC_TAG, "特征数据长度不足: %d", p_data->notify.value_len);
            }
            break;

        case 1: // EEG 频谱值接收
            ESP_LOGI(GATTC_TAG, "收到频谱数据");
            if (p_data->notify.value_len >= 5) {
                ui_data.is_wave_data = true;
                ui_data.wave_power[0] = p_data->notify.value[0];  // 德尔塔
                ui_data.wave_power[1] = p_data->notify.value[1];  // 西塔
                ui_data.wave_power[2] = p_data->notify.value[2];  // 阿尔法
                ui_data.wave_power[3] = p_data->notify.value[3];  // 贝塔
                ui_data.wave_power[4] = p_data->notify.value[4];  // 伽马

                ESP_LOGI(GATTC_TAG, "Delta(δ): %d, Theta(θ): %d, Alpha(α): %d, Beta(β): %d, Gamma(γ): %d",
                        ui_data.wave_power[0], ui_data.wave_power[1], ui_data.wave_power[2], 
                        ui_data.wave_power[3], ui_data.wave_power[4]);
                
                // 发送到UI队列
                if (xQueueSend(ui_data_queue, &ui_data, 0) != pdTRUE) {
                    ESP_LOGW(GATTC_TAG, "UI队列满，丢弃频谱数据");
                }
            } else {
                ESP_LOGW(GATTC_TAG, "频谱数据长度不足: %d", p_data->notify.value_len);
            }
            break;
        default:
            ESP_LOGI(GATTC_TAG, "接收到其他未知数据");
            break;
        }
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Descriptor write failed, status %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Descriptor write successfully");
        set_conn_status(CONN_STATUS_CONNECTED);
        uint8_t write_char_data[35];
        for (int i = 0; i < sizeof(write_char_data); ++i)
        {
            write_char_data[i] = i % 256;
        }
        esp_ble_gattc_write_char( gattc_if,
                                  gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                  gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                  sizeof(write_char_data),
                                  write_char_data,
                                  ESP_GATT_WRITE_TYPE_RSP,
                                  ESP_GATT_AUTH_REQ_NONE);
        break;
    case ESP_GATTC_SRVC_CHG_EVT: {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "Service change from "ESP_BD_ADDR_STR"", ESP_BD_ADDR_HEX(bda));
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Characteristic write failed, status %x)", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Characteristic write successfully");
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        connect = false;
        get_server = false;
        set_conn_status(CONN_STATUS_DISCONNECTED);
        ESP_LOGI(GATTC_TAG, "Disconnected, remote "ESP_BD_ADDR_STR", reason 0x%02x",
                 ESP_BD_ADDR_HEX(p_data->disconnect.remote_bda), p_data->disconnect.reason);
        break;
    default:
        break;
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        uint32_t duration = 30;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTC_TAG, "Scanning start failed, status %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Scanning start successfully");
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            
            if (adv_name != NULL && adv_name_len == strlen(remote_device_name) &&
                strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0) {
                ESP_LOGI(GATTC_TAG, "找到目标设备: %s", remote_device_name);
                
                if (connect == false) {
                    connect = true;
                    ESP_LOGI(GATTC_TAG, "连接到远程设备");
                    esp_ble_gap_stop_scanning();
                    set_conn_status(CONN_STATUS_CONNECTING);
                    esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
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
            ESP_LOGE(GATTC_TAG, "Scanning stop failed, status %x", param->scan_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Stop scan successfully");
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "Advertising stop failed, status %x", param->adv_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Stop advertising successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(GATTC_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
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
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGI(GATTC_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE || 
                    gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

//******************************************************************************** */
//UI相关函数和参数初始化************************************************************ */
//******************************************************************************** */

// 未连接界面
static lv_obj_t* ui_disconnected(void) {
    lv_obj_t *scr = lv_obj_create(lv_scr_act());
    lv_obj_center(scr);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);

    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_text(label, "未连接");
    lv_obj_set_style_text_font(label, &GenJyuuGothic_32, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(0xFF0000), 0);
    lv_obj_center(label);
    lv_obj_clear_flag(label, LV_OBJ_FLAG_SCROLLABLE);

    return scr;
}

// 连接中界面
static lv_obj_t* ui_connecting(void) {
    lv_obj_t *scr = lv_obj_create(lv_scr_act());
    lv_obj_center(scr);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);

    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_text(label, "连接中");
    lv_obj_set_style_text_font(label, &GenJyuuGothic_32, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFF00), 0);
    lv_obj_center(label);
    lv_obj_clear_flag(label, LV_OBJ_FLAG_SCROLLABLE);

    return scr;
}

// 已连接界面
static lv_obj_t* ui_connected(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    
    // 清除屏幕并设置背景色
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);

    // 总容器
    lv_obj_t *cont = lv_obj_create(scr);
    lv_obj_set_size(cont, lv_pct(100), lv_pct(100));
    lv_obj_center(cont);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(cont, 15, 0);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(cont, lv_color_hex(0x000000), 0);
    lv_obj_set_style_border_width(cont, 0, 0);

    // ====== 上半部分：5个波段 ======
    for (int i = 0; i < 5; i++) {
        lv_obj_t *row = lv_obj_create(cont);
        lv_obj_set_size(row, lv_pct(100), 50);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_style_pad_column(row, 10, 0);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_bg_color(row, lv_color_hex(0x000000), 0);
        lv_obj_set_style_border_width(row, 0, 0);

        waves[i].label = lv_label_create(row);
        lv_label_set_text(waves[i].label, wave_names[i]);
        lv_obj_set_style_text_font(waves[i].label, &GenJyuuGothic_32, 0);
        lv_obj_set_style_text_color(waves[i].label, lv_color_hex(0xFFFFFF), 0);

        waves[i].value = lv_label_create(row);
        lv_label_set_text(waves[i].value, "0");
        lv_obj_set_style_text_font(waves[i].value, &GenJyuuGothic_32, 0);
        lv_obj_set_style_text_color(waves[i].value, lv_color_hex(0x00FF00), 0);
        lv_obj_set_width(waves[i].value, 50);

        waves[i].bar = lv_bar_create(row);
        lv_obj_set_size(waves[i].bar, 150, 20);
        lv_bar_set_range(waves[i].bar, 0, 100);
        lv_bar_set_value(waves[i].bar, 0, LV_ANIM_OFF);
        lv_bar_set_mode(waves[i].bar, LV_BAR_MODE_NORMAL);
        // 设置进度条颜色
        lv_obj_set_style_bg_color(waves[i].bar, lv_color_hex(0x333333), LV_PART_MAIN);
        lv_obj_set_style_bg_color(waves[i].bar, lv_color_hex(0x0080FF), LV_PART_INDICATOR);
    }

    // ====== 下半部分：4个指标 ======
    lv_obj_t *row_metrics = lv_obj_create(cont);
    lv_obj_set_size(row_metrics, lv_pct(100), 100);
    lv_obj_set_flex_flow(row_metrics, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(row_metrics, 20, 0);
    lv_obj_set_flex_align(row_metrics, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(row_metrics, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(row_metrics, lv_color_hex(0x000000), 0);
    lv_obj_set_style_border_width(row_metrics, 0, 0);

    for (int i = 0; i < 4; i++) {
        lv_obj_t *col = lv_obj_create(row_metrics);
        lv_obj_set_size(col, 70, 100);
        lv_obj_set_flex_flow(col, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_row(col, 5, 0);
        lv_obj_set_flex_align(col, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_border_width(col, 0, 0);
        lv_obj_clear_flag(col, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_bg_color(col, lv_color_hex(0x000000), 0);

        metrics[i].title = lv_label_create(col);
        lv_label_set_text(metrics[i].title, metric_names[i]);
        lv_obj_set_style_text_font(metrics[i].title, &GenJyuuGothic_32, 0);
        lv_obj_set_style_text_color(metrics[i].title, lv_color_hex(0xFFFFFF), 0);

        metrics[i].value = lv_label_create(col);
        lv_label_set_text(metrics[i].value, "0");
        lv_obj_set_style_text_font(metrics[i].value, &GenJyuuGothic_32, 0);
        lv_obj_set_style_text_color(metrics[i].value, lv_color_hex(0xFFFF00), 0);
    }
    return cont;
}

// 更新某一行的功率
static void update_wave(int idx, int value)
{
    if (idx < 0 || idx >= 5 || waves[idx].value == NULL) return;

    char buf[8];
    snprintf(buf, sizeof(buf), "%d", value);
    lv_label_set_text(waves[idx].value, buf);
    lv_bar_set_value(waves[idx].bar, value, LV_ANIM_OFF);
}

// 更新某一指标的数值
static void update_metric(int idx, int value)
{
    if (idx < 0 || idx >= 4 || metrics[idx].value == NULL) return;

    char buf[8];
    snprintf(buf, sizeof(buf), "%d", value);
    lv_label_set_text(metrics[idx].value, buf);
}

// 更新界面，根据连接状态切换不同的UI
static void ui_update(void)
{
    bsp_display_lock(0);
    
    // 清除UI指针
    memset(waves, 0, sizeof(waves));
    memset(metrics, 0, sizeof(metrics));
    
    // 删除旧界面
    if (root_ui) {
        lv_obj_del(root_ui);
        root_ui = NULL;
    }

    // 创建新界面
    conn_status_t current_status = get_conn_status();
    switch (current_status) {
        case CONN_STATUS_DISCONNECTED:
            root_ui = ui_disconnected();
            break;
        case CONN_STATUS_CONNECTING:
            root_ui = ui_connecting();
            break;
        case CONN_STATUS_CONNECTED:
            root_ui = ui_connected();
            break;
    }
    
    // 确保UI刷新
    lv_refr_now(NULL);
    bsp_display_unlock();
}

// UI管理任务 - 优化版本
static void ui_manager_task(void *arg)
{
    ui_data_t data;
    EventBits_t bits;
    TickType_t last_update_time = 0;
    const TickType_t MIN_UPDATE_INTERVAL = pdMS_TO_TICKS(50); // 最小50ms更新间隔
    
    while (1) {
        // 等待事件
        bits = xEventGroupWaitBits(ui_event_group, 
                                   UI_UPDATE_STATUS_BIT | UI_UPDATE_DATA_BIT,
                                   pdTRUE,  // 清除位
                                   pdFALSE, // 任意一个位被设置就返回
                                   pdMS_TO_TICKS(500)); // 500ms超时

        // 处理状态更新
        if (bits & UI_UPDATE_STATUS_BIT) {
            ui_update();
            last_update_time = xTaskGetTickCount();
        }
        
        // 处理数据更新（只在已连接状态下，且控制更新频率）
        if ((bits & UI_UPDATE_DATA_BIT) || (get_conn_status() == CONN_STATUS_CONNECTED)) {
            TickType_t current_time = xTaskGetTickCount();
            if ((current_time - last_update_time) >= MIN_UPDATE_INTERVAL) {
                // 批量处理队列中的数据
                int processed = 0;
                while (xQueueReceive(ui_data_queue, &data, 0) == pdTRUE && processed < 5) {
                    bsp_display_lock(0);
                    if (data.is_wave_data) {
                        // 更新波段数据
                        for (int i = 0; i < 5; i++) {
                            update_wave(i, data.wave_power[i]);
                        }
                    } else {
                        // 更新指标数据
                        for (int i = 0; i < 4; i++) {
                            update_metric(i, data.metric_values[i]);
                        }
                    }
                    bsp_display_unlock();
                    processed++;
                }
                
                if (processed > 0) {
                    last_update_time = current_time;
                }
            } else {
                // 清空队列但不更新UI，避免过频更新
                while (xQueueReceive(ui_data_queue, &data, 0) == pdTRUE) {
                    // 只是清空，不处理
                }
            }
        }
    }
}

void app_main(void)
{
    printf("ESP32 starting up...\n");
    
    // 创建同步对象
    conn_status_mutex = xSemaphoreCreateMutex();
    ui_data_queue = xQueueCreate(20, sizeof(ui_data_t));
    ui_event_group = xEventGroupCreate();
    
    if (!conn_status_mutex || !ui_data_queue || !ui_event_group) {
        printf("ERROR: Failed to create sync objects\n");
        return;
    }
    printf("Sync objects created successfully\n");

    //******************************************************************************** */
    //蓝牙部分：让所有蓝牙相关内容运行*************************************************** */
    //******************************************************************************** */
    printf("Initializing NVS...\n");
    // Initialize NVS.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    printf("NVS initialized\n");

    #if CONFIG_EXAMPLE_CI_PIPELINE_ID
    memcpy(remote_device_name, esp_bluedroid_get_example_name(), sizeof(remote_device_name));
    #endif

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret){
        ESP_LOGE(GATTC_TAG, "%s gap register failed, error code = %x", __func__, ret);
        return;
    }

    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if(ret){
        ESP_LOGE(GATTC_TAG, "%s gattc register failed, error code = %x", __func__, ret);
        return;
    }

    ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    if (ret){
        ESP_LOGE(GATTC_TAG, "%s gattc app register failed, error code = %x", __func__, ret);
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTC_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    //******************************************************************************** */
    //UI正式搭建部分******************************************************************* */
    //******************************************************************************** */
    printf("Starting display system...\n");
    bsp_display_start();
    printf("Display system started\n");

    // 初始显示
    bsp_display_lock(0);
    set_conn_status(CONN_STATUS_DISCONNECTED);
    ui_update();
    bsp_display_unlock();

    // 创建UI管理任务 - 较低优先级，固定到核心0，增加栈空间
    xTaskCreatePinnedToCore(ui_manager_task, "ui_manager", 8192, NULL, 3, NULL, 0);
    
    ESP_LOGI(GATTC_TAG, "System initialization completed");
}