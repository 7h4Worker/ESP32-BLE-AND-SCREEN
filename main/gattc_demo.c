/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */



/****************************************************************************
*
* This demo showcases BLE GATT client. It can scan BLE devices and connect to one device.
* Run the gatt_server demo, the client demo will automatically connect to the gatt_server demo.
* Client demo will enable gatt_server's notify after connection. The two devices will then exchange
* data.
*
****************************************************************************/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_memory_utils.h"
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

SemaphoreHandle_t data_mutex;

#define GATTC_TAG "GATTC_DEMO"
#define REMOTE_SERVICE_UUID        0x00FF

#define PROFILE_NUM      1
#define PROFILE_A_APP_ID 0
#define INVALID_HANDLE   0
#define MAX_SERVICES 10 

LV_FONT_DECLARE(GenJyuuGothic_32);//字体

//******************************************************************************** */
//需要展示的脑电参数和显示参数初始化********************************************************** */
//******************************************************************************** */

typedef enum {
    CONN_STATUS_DISCONNECTED, // 未连接
    CONN_STATUS_CONNECTING,       // 连接中
    CONN_STATUS_CONNECTED         // 已连接
} conn_status_t;
static conn_status_t conn_status = CONN_STATUS_DISCONNECTED;
static int last_status = -1;  // 记录上一次的状态，初始设为无效值
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
static uint8_t wave_power[5] = {0}; // 存放每个波段的功率值
static uint8_t metric_values[4] = {0}; // 存放专注、放松、疲劳、冥想的值

// 目标设备名字
static char remote_device_name[ESP_BLE_ADV_NAME_LEN_MAX] = "KSEEG102";

// 目标 UUID (128-bit)，以字节数组形式表示
static const uint8_t remote_device_uuid[16] = {
    0x40, 0xE3, 0x4A, 0x1D,
    0xC2, 0x5F,
    0xB0, 0x9C,
    0xB7, 0x47,
    0xE6, 0x43,
    0x0A, 0x00, 0x53, 0x86
};

typedef struct {
    uint8_t remote_service_uuid[16]; // 128-bit UUID
    uint16_t start_handle;           // 服务的起始句柄
    uint16_t end_handle;             // 服务的结束句柄
    uint16_t char_handle;            // 特征句柄（可选）
} service_info_t;

// 用一个全局数组来保存多个服务的信息
static service_info_t saved_service_info[MAX_SERVICES];

static bool connect    = false;
static bool get_server = false;
static esp_gattc_char_elem_t *char_elem_result   = NULL;
static esp_gattc_descr_elem_t *descr_elem_result = NULL;

/* Declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

static void scan_result_callback(esp_ble_gap_cb_param_t *scan_result_param) {
    esp_ble_gap_cb_param_t *scan_rst = scan_result_param;
    ESP_LOGI(GATTC_TAG, "开始打印设备信息");

    if (scan_rst->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
        uint8_t *adv_name = NULL;
        uint8_t adv_name_len = 0;
        uint8_t *uuid16 = NULL;
        uint8_t uuid16_len = 0;
        uint8_t *uuid128 = NULL;
        uint8_t uuid128_len = 0;
        uint8_t *flags = NULL;
        uint8_t flags_len = 0;

        // Flags
        flags = esp_ble_resolve_adv_data(scan_rst->scan_rst.ble_adv,
                                         ESP_BLE_AD_TYPE_FLAG,
                                         &flags_len);
        if (flags != NULL && flags_len > 0) {
            ESP_LOGI(GATTC_TAG, "Flags: 0x%02X", flags[0]);
        }

        // Device Name
        adv_name = esp_ble_resolve_adv_data(scan_rst->scan_rst.ble_adv,
                                            ESP_BLE_AD_TYPE_NAME_CMPL,
                                            &adv_name_len);
        if (adv_name != NULL && adv_name_len > 0) {
            ESP_LOGI(GATTC_TAG, "Device Name (%d bytes): %.*s", adv_name_len, adv_name_len, adv_name);
        }

        // 16-bit UUIDs
        uuid16 = esp_ble_resolve_adv_data(scan_rst->scan_rst.ble_adv,
                                          ESP_BLE_AD_TYPE_16SRV_CMPL,
                                          &uuid16_len);
        if (uuid16 != NULL && uuid16_len > 0) {
            ESP_LOGI(GATTC_TAG, "16-bit Service UUIDs (%d bytes):", uuid16_len);
            for (int i = 0; i < uuid16_len; i+=2) {
                printf("%02X%02X ", uuid16[i+1], uuid16[i]); // 小端
            }
            printf("\n");
        }

        // 128-bit UUIDs
        uuid128 = esp_ble_resolve_adv_data(scan_rst->scan_rst.ble_adv,
                                           ESP_BLE_AD_TYPE_128SRV_CMPL,
                                           &uuid128_len);
        if (uuid128 != NULL && uuid128_len > 0) {
            ESP_LOGI(GATTC_TAG, "128-bit Service UUIDs (%d bytes):", uuid128_len);
            for (int i = 0; i < uuid128_len; i+=16) {
                printf("UUID: ");
                for (int j = 0; j < 16; j++) {
                    printf("%02X", uuid128[i+j]);
                }
                printf("\n");
            }
        }

        // 打印蓝牙地址
        ESP_LOGI(GATTC_TAG, "Device Addr: "ESP_BD_ADDR_STR,
                 ESP_BD_ADDR_HEX(scan_rst->scan_rst.bda));
    }
    ESP_LOGI(GATTC_TAG, "结束打印设备信息 ");
}

static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid = {.uuid128 = {0x40, 0xe3, 0x4a, 0x1d,0xc2, 0x5f, 0xb0, 0x9c,0xb7, 0x47, 0xe6, 0x43,0x0a, 0x00, 0x53, 0x86},
    },
};

// 特征 UUIDs (128-bit)
// // 蓝牙指令读写服务 UUID: 0x8653000c-43e6-47b7-9cb0-5fc21d4ae340
// static const esp_bt_uuid_t uuid_cmd_rw = {
//     .len = ESP_UUID_LEN_128,
//     .uuid.uuid128 = {0x40,0xe3,0x4a,0x1d,0xc2,0x5f,0xb0,0x9c,
//                      0xb7,0x47,0xe6,0x43,0x0c,0x00,0x53,0x86}
// };

// // EEG 原始数据接收 UUID: 0x8653000b-43e6-47b7-9cb0-5fc21d4ae340
// static const esp_bt_uuid_t uuid_eeg_raw = {
//     .len = ESP_UUID_LEN_128,
//     .uuid.uuid128 = {0x40,0xe3,0x4a,0x1d,0xc2,0x5f,0xb0,0x9c,
//                      0xb7,0x47,0xe6,0x43,0x0b,0x00,0x53,0x86}
// };

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

// // 外置 G-Sensor/PPG 原始数据接收 UUID: 0x8653000f-43e6-47b7-9cb0-5fc21d4ae340
// static const esp_bt_uuid_t uuid_sensor_raw = {
//     .len = ESP_UUID_LEN_128,
//     .uuid.uuid128 = {0x40,0xe3,0x4a,0x1d,0xc2,0x5f,0xb0,0x9c,
//                      0xb7,0x47,0xe6,0x43,0x0f,0x00,0x53,0x86}
// };

// // 眨眼/咬牙数据接收 UUID: 0x86530010-43e6-47b7-9cb0-5fc21d4ae340
// static const esp_bt_uuid_t uuid_eye_bite = {
//     .len = ESP_UUID_LEN_128,
//     .uuid.uuid128 = {0x40,0xe3,0x4a,0x1d,0xc2,0x5f,0xb0,0x9c,
//                      0xb7,0x47,0xe6,0x43,0x10,0x00,0x53,0x86}
// };

static const esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,},
};

static esp_bt_uuid_t target_uuids[] = {
    //特征值（专注度，放松度，疲劳度，冥想值）
    uuid_eeg_feature,
    //频谱（δ，θ，α，β，γ）
    uuid_eeg_spectrum,
    //以上是要订阅的所有特征 UUID
};
#define target_uuid_num 2//上面的特征 UUID 数量，也就是子服务的个数，target_uuids扩充时必须实时跟着修改

// 存储句柄数组（负责依次存储上面的特征的handle）
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

    uint16_t cmd_rw_handle;            // 指令读写
    uint16_t eeg_raw_handle;           // EEG 原始数据
    uint16_t eeg_feature_handle;       // EEG 特征
    uint16_t eeg_spectrum_handle;      // EEG 频谱
    uint16_t sensor_handle;            // G-sensor / PPG
    uint16_t blink_bite_handle;        // 眨眼咬牙

};

/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

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
        esp_gatt_srvc_id_t *srvc_id = &p_data->search_res.srvc_id;
        ESP_LOGI(GATTC_TAG, "服务信息打印开始↓");
        if (srvc_id->id.uuid.len == ESP_UUID_LEN_16) {
            ESP_LOGI(GATTC_TAG, "Service UUID16: 0x%04x", (unsigned int)srvc_id->id.uuid.uuid.uuid16);
        } else if (srvc_id->id.uuid.len == ESP_UUID_LEN_32) {
            ESP_LOGI(GATTC_TAG, "Service UUID32: 0x%08" PRIx32, srvc_id->id.uuid.uuid.uuid32);
        } else if (srvc_id->id.uuid.len == ESP_UUID_LEN_128) {
            ESP_LOGI(GATTC_TAG, "Service UUID128:");
            ESP_LOG_BUFFER_HEX(GATTC_TAG, srvc_id->id.uuid.uuid.uuid128, 16);
            // ====== 保存目标服务的句柄范围 ======
            if (memcmp(srvc_id->id.uuid.uuid.uuid128, remote_filter_service_uuid.uuid.uuid128, 16) == 0) {
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
                    //遍历目标 UUIDs以获取所有特征句柄
                    for (int i = 0; i < target_uuid_num; i++) {
                        status = esp_ble_gattc_get_char_by_uuid( gattc_if,
                                                                p_data->search_cmpl.conn_id,
                                                                gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                target_uuids[i], // 遍历特征 UUID
                                                                char_elem_result,
                                                                &count);

                        /* 服务的特征存储在char_elem_result里，现在把handle取出来 */
                        if (count > 0 && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)){
                            gl_profile_tab[PROFILE_A_APP_ID].char_handle = char_elem_result[0].char_handle;

                            // 将特征句柄存储到 char_handles 数组中
                            char_handles[i] = char_elem_result[0].char_handle;

                            ESP_LOGI(GATTC_TAG, "已获得特征句柄: %d", gl_profile_tab[PROFILE_A_APP_ID].char_handle);
                            
                            //现在使用存储的特征句柄对特征进行注册
                            esp_ble_gattc_register_for_notify (gattc_if, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, gl_profile_tab[PROFILE_A_APP_ID].char_handle);
                        }
                    }
                }
                /* free char_elem_result */
                free(char_elem_result);
            }else{
                ESP_LOGE(GATTC_TAG, "no char found");
            }
        }
         break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        if (p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Notification register failed, status %d", p_data->reg_for_notify.status);
        }else{
            ESP_LOGI(GATTC_TAG, "Notification register successfully");
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
                break;
            }
            if (count > 0){
                descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result){
                    ESP_LOGE(GATTC_TAG, "malloc error, gattc no mem");
                    break;
                }else{
                    ret_status = esp_ble_gattc_get_descr_by_char_handle( gattc_if,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                         p_data->reg_for_notify.handle,
                                                                         notify_descr_uuid,
                                                                         descr_elem_result,
                                                                         &count);
                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                        free(descr_elem_result);
                        descr_elem_result = NULL;
                        break;
                    }
                    /* Every char has only one descriptor in our 'ESP_GATTS_DEMO' demo, so we used first 'descr_elem_result' */
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



                    /* free descr_elem_result */
                    free(descr_elem_result);
                }
            }
            else{
                ESP_LOGE(GATTC_TAG, "decsr not found");
            }

        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        uint16_t handle = p_data->notify.handle;
        uint16_t handle_index = 0;
        for (int i = 0; i < target_uuid_num; i++) if (handle == char_handles[i]) handle_index = i;

        switch (handle_index) {
        case 0: // EEG 特征值接收
            ESP_LOGI(GATTC_TAG, "收到特征值数据");
            if (xSemaphoreTake(data_mutex, portMAX_DELAY)){
                if (p_data->notify.value_len >= 4) {
                metric_values[0] = p_data->notify.value[0]; // 专注度
                metric_values[1] = p_data->notify.value[1]; // 放松度
                metric_values[2] = p_data->notify.value[2]; // 疲劳值
                metric_values[3] = p_data->notify.value[3]; // 冥想值

                ESP_LOGI(GATTC_TAG, "专注度: %d, 放松度: %d, 疲劳值: %d, 冥想值: %d",
                        metric_values[0], metric_values[1], metric_values[2], metric_values[3]);
                } 
                else {
                    ESP_LOGW(GATTC_TAG, "特征值数据长度不足: %d", p_data->notify.value_len);
                }
                xSemaphoreGive(data_mutex);
            }
            break;

        case 1: // EEG 频谱值接收
            ESP_LOGI(GATTC_TAG, "收到频谱数据");
            if (xSemaphoreTake(data_mutex, portMAX_DELAY)){
                if (p_data->notify.value_len >= 5) {
                wave_power[0] = p_data->notify.value[0];  // 德尔塔
                wave_power[1] = p_data->notify.value[1];  // 西塔
                wave_power[2] = p_data->notify.value[2];  // 阿尔法
                wave_power[3] = p_data->notify.value[3];  // 贝塔
                wave_power[4] = p_data->notify.value[4];  // 伽马

                ESP_LOGI(GATTC_TAG, "Delta(δ): %d, Theta(θ): %d, Alpha(α): %d, Beta(β): %d, Gamma(γ): %d",
                        wave_power[0], wave_power[1], wave_power[2], wave_power[3], wave_power[4]);
                } 
                else {
                    ESP_LOGW(GATTC_TAG, "频谱数据长度不足: %d", p_data->notify.value_len);
                }
                xSemaphoreGive(data_mutex);
            }
            
            break;
        default:
            ESP_LOGI(GATTC_TAG, "接收到其他未知数据");
            // 这里可以添加更多的处理逻辑，比如根据 handle_index 进行不同的处理
            break;
        }
        ESP_LOGI(GATTC_TAG, " ");
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Descriptor write failed, status %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Descriptor write successfully");
        conn_status = CONN_STATUS_CONNECTED;
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
        conn_status = CONN_STATUS_DISCONNECTED;
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
    uint8_t *uuid_data = NULL;
    uint8_t uuid_len = 0;
    uint8_t adv_name_len = 0;
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
            ESP_LOGE(GATTC_TAG, "Scanning start failed, status %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Scanning start successfully");

        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        //scan_result_callback(param);  // <--- 打印扫描到的设备信息的函数，避免输出太多现在就不打印了
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            // 提取 UUID
            
            uuid_data = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                     ESP_BLE_AD_TYPE_128SRV_CMPL,
                                     &uuid_len);

            // 提取设备名
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                 ESP_BLE_AD_TYPE_NAME_CMPL,
                                                 &adv_name_len);

            if(connect == false){
                 ESP_LOGI(GATTC_TAG, "Scan result, device "ESP_BD_ADDR_STR", name len %u", ESP_BD_ADDR_HEX(scan_result->scan_rst.bda), adv_name_len);
                 ESP_LOG_BUFFER_CHAR(GATTC_TAG, adv_name, adv_name_len);
            }

            // ---- 判断逻辑 ----
            bool target_found = false;

            // 1. 先比对 UUID
            if (uuid_data != NULL && uuid_len == 16) {   // 128-bit UUID
                if (memcmp(uuid_data, remote_device_uuid, 16) == 0) {
                    target_found = true;
                    ESP_LOGI(GATTC_TAG, "Found target by UUID");
                }
            }

            // 2. 如果 UUID 没有，就尝试比对设备名
            if (!target_found && adv_name != NULL) {
                if (strlen(remote_device_name) == adv_name_len &&
                    strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0) {
                    target_found = true;
                    ESP_LOGI(GATTC_TAG, "Found target by Name: %s", remote_device_name);
                }
            }

            // 3. 真正连接
            if (target_found && connect == false) {
                conn_status = CONN_STATUS_CONNECTING;
                connect = true;
                
                ESP_LOGI(GATTC_TAG, "Connecting...");

                esp_ble_gap_stop_scanning();
                esp_ble_gatt_creat_conn_params_t creat_conn_params = {0};
                memcpy(&creat_conn_params.remote_bda, scan_result->scan_rst.bda, ESP_BD_ADDR_LEN);
                creat_conn_params.remote_addr_type = scan_result->scan_rst.ble_addr_type;
                creat_conn_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
                creat_conn_params.is_direct = true;
                creat_conn_params.is_aux = false;
                creat_conn_params.phy_mask = 0x0;

                esp_ble_gattc_enh_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
                                    &creat_conn_params);
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
        ESP_LOGI(GATTC_TAG, "Scanning stop successfully");
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "Advertising stop failed, status %x", param->adv_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Advertising stop successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(GATTC_TAG, "Connection params update, status %d, conn_int %d, latency %d, timeout %d",
                  param->update_conn_params.status,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
        ESP_LOGI(GATTC_TAG, "Packet length update, status %d, rx %d, tx %d",
                  param->pkt_data_length_cmpl.status,
                  param->pkt_data_length_cmpl.params.rx_len,
                  param->pkt_data_length_cmpl.params.tx_len);
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
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGI(GATTC_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
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

// 未连接界面*************************************************************************/
static lv_obj_t* ui_disconnected(void) {
    lv_obj_t *scr = lv_obj_create(lv_scr_act());
    lv_obj_center(scr);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_text(label, "未连接");
    lv_obj_set_style_text_font(label, &GenJyuuGothic_32, 0);
    lv_obj_center(label);
    lv_obj_clear_flag(label, LV_OBJ_FLAG_SCROLLABLE);

    return scr;
}

// 连接中界面*************************************************************************/
static lv_obj_t* ui_connecting(void) {
    lv_obj_t *scr = lv_obj_create(lv_scr_act());
    lv_obj_center(scr);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_text(label, "连接中");
    lv_obj_set_style_text_font(label, &GenJyuuGothic_32, 0);
    lv_obj_center(label);
    lv_obj_clear_flag(label, LV_OBJ_FLAG_SCROLLABLE);

    return scr;
}

//已连接界面*************************************************************************/
static lv_obj_t* ui_connected(void)
{
    lv_obj_t *scr = lv_scr_act();

    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);// 清除滚动条标志

    // 总容器（垂直排列，上面波段 + 下面四个指标）
    lv_obj_t *cont = lv_obj_create(scr);
    lv_obj_set_size(cont, lv_pct(100), lv_pct(100));
    lv_obj_center(cont);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(cont, 15, 0);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

    // ====== 上半部分：5个波段 ======
    for (int i = 0; i < 5; i++) {
        // 每一行容器：水平排布
        lv_obj_t *row = lv_obj_create(cont);
        lv_obj_set_size(row, lv_pct(100), 50);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_style_pad_column(row, 10, 0);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

        // 左侧标签
        waves[i].label = lv_label_create(row);
        lv_label_set_text(waves[i].label, wave_names[i]);
        lv_obj_set_style_text_font(waves[i].label, &GenJyuuGothic_32, 0);

        // 数值标签
        waves[i].value = lv_label_create(row);
        lv_label_set_text(waves[i].value, "0");
        lv_obj_set_style_text_font(waves[i].value, &GenJyuuGothic_32, 0);
        lv_obj_set_width(waves[i].value, 50);   // 固定宽度，例如 50px

        // 进度条
        waves[i].bar = lv_bar_create(row);
        lv_obj_set_size(waves[i].bar, 150, 20);
        lv_bar_set_range(waves[i].bar, 0, 100);
        lv_bar_set_value(waves[i].bar, 0, LV_ANIM_OFF);
        lv_bar_set_mode(waves[i].bar, LV_BAR_MODE_NORMAL); // 水平条
    }

    // ====== 下半部分：4个指标 ======
    lv_obj_t *row_metrics = lv_obj_create(cont);
    lv_obj_set_size(row_metrics, lv_pct(100), 100);
    lv_obj_set_flex_flow(row_metrics, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(row_metrics, 20, 0);
    lv_obj_set_flex_align(row_metrics, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(row_metrics, LV_OBJ_FLAG_SCROLLABLE);

    for (int i = 0; i < 4; i++) {
        lv_obj_t *col = lv_obj_create(row_metrics);
        lv_obj_set_size(col, 70, 100);
        lv_obj_set_flex_flow(col, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_row(col, 5, 0);
        lv_obj_set_flex_align(col, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_border_width(col, 0, 0);
        lv_obj_clear_flag(col, LV_OBJ_FLAG_SCROLLABLE);

        // 标题
        metrics[i].title = lv_label_create(col);
        lv_label_set_text(metrics[i].title, metric_names[i]);
        lv_obj_set_style_text_font(metrics[i].title, &GenJyuuGothic_32, 0);

        // 数值
        metrics[i].value = lv_label_create(col);
        lv_label_set_text(metrics[i].value, "0");
        lv_obj_set_style_text_font(metrics[i].value, &GenJyuuGothic_32, 0);
    }
    return cont;
}

// 更新某一行的功率
static void update_wave(int idx, int value)
{
    if (idx < 0 || idx >= 5) return;

    char buf[8];
    snprintf(buf, sizeof(buf), "%d", value);
    lv_label_set_text(waves[idx].value, buf);
    lv_bar_set_value(waves[idx].bar, value, LV_ANIM_OFF);
}

// 更新某一指标的数值
static void update_metric(int idx, int value)
{
    if (idx < 0 || idx >= 4) return;

    char buf[8];
    snprintf(buf, sizeof(buf), "%d", value);
    lv_label_set_text(metrics[idx].value, buf);
}

//更新所有ui值
static void update_task(void *arg)
{
    int val;
    while (1) {
        // 更新5个波段
        for (int i = 0; i < 5; i++) {
            val = esp_random() % 101;
            bsp_display_lock(0);
            update_wave(i, val);
            bsp_display_unlock();
        }

        // 更新4个指标
        for (int i = 0; i < 4; i++) {
            val = esp_random() % 101;
            bsp_display_lock(0);
            update_metric(i, val);
            bsp_display_unlock();
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// 更新界面，根据连接状态切换不同的UI
static void ui_update(void)
{
    // 删除旧界面
    if (root_ui) {
        lv_obj_del(root_ui);
        root_ui = NULL;
    }

    // 创建新界面
    switch (conn_status) {
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
}


void app_main(void)
{
    //定义互斥锁
    data_mutex = xSemaphoreCreateMutex();
    //******************************************************************************** */
    //蓝牙部分：让所有蓝牙相关内容运行*************************************************** */
    //******************************************************************************** */
    // // Initialize NVS.
    // esp_err_t ret = nvs_flash_init();
    // if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    //     ESP_ERROR_CHECK(nvs_flash_erase());
    //     ret = nvs_flash_init();
    // }
    // ESP_ERROR_CHECK( ret );

    // #if CONFIG_EXAMPLE_CI_PIPELINE_ID
    // memcpy(remote_device_name, esp_bluedroid_get_example_name(), sizeof(remote_device_name));
    // #endif

    // ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    // esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    // ret = esp_bt_controller_init(&bt_cfg);
    // if (ret) {
    //     ESP_LOGE(GATTC_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
    //     return;
    // }

    // ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    // if (ret) {
    //     ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
    //     return;
    // }

    // ret = esp_bluedroid_init();
    // if (ret) {
    //     ESP_LOGE(GATTC_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
    //     return;
    // }

    // ret = esp_bluedroid_enable();
    // if (ret) {
    //     ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
    //     return;
    // }
    // // Note: Avoid performing time-consuming operations within callback functions.
    // // Register the callback function to the gap module
    // ret = esp_ble_gap_register_callback(esp_gap_cb);
    // if (ret){
    //     ESP_LOGE(GATTC_TAG, "%s gap register failed, error code = %x", __func__, ret);
    //     return;
    // }

    // // Register the callback function to the gattc module
    // ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    // if(ret){
    //     ESP_LOGE(GATTC_TAG, "%s gattc register failed, error code = %x", __func__, ret);
    //     return;
    // }

    // ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    // if (ret){
    //     ESP_LOGE(GATTC_TAG, "%s gattc app register failed, error code = %x", __func__, ret);
    // }
    // esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    // if (local_mtu_ret){
    //     ESP_LOGE(GATTC_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    // }

    //******************************************************************************** */
    //UI正式搭建部分******************************************************************* */
    //******************************************************************************** */
    #define LV_MEM_SIZE (1024 * 1024)  // 例如1MB
    bsp_display_start();
    bsp_display_lock(0);

    // 默认显示未连接
    conn_status = CONN_STATUS_DISCONNECTED;
    ui_update();

    bsp_display_unlock();

    // 模拟延时切换状态
    vTaskDelay(pdMS_TO_TICKS(2000));
    bsp_display_lock(0);
    conn_status = CONN_STATUS_CONNECTING;
    ui_update();
    bsp_display_unlock();

    vTaskDelay(pdMS_TO_TICKS(2000));
    bsp_display_lock(0);
    conn_status = CONN_STATUS_CONNECTED;
    ui_update();
    bsp_display_unlock();

    // 已连接时才启动刷新任务
    xTaskCreatePinnedToCore(update_task, "update_task", 8192, NULL, 10, NULL, 1);
}
