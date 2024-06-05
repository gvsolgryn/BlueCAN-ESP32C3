#ifndef _MAIN_
#define _MAIN_

/*
 * SPDX-FileCopyrightText: 2010-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "sdkconfig.h"

#include "driver/twai.h"

/* --------------------- Definitions and static variables ------------------ */
#define RX_TASK_PRIO        3
#define TX_TASK_PRIO        4
#define CTRL_TASK_PRIO      5
#define STATUS_TASK_PRIO    6

#define TX_GPIO_NUM     0
#define RX_GPIO_NUM     1

#define APP_TAG     "BlueCAN"

/* --------------------- BLE server Definitions and static variables --------------------- */
#define GATTS_SERVICE_UUID      0x00FF
#define GATTS_CHAR_UUID         0xFF01
#define GATTS_DESCR_UUID        0x3333
#define GATTS_NUM_HANDLE        4

#define DEVICE_NAME             "BlueCAN"
#define MANUFACTURER_DATA_LEN   17

#define GATTS_CHAR_VAR_LEN_MAX  0x40

#define PREPARE_BUF_MAX_SIZE 1024

typedef struct {
    uint8_t                *prepare_buf;
    int                     prepare_len;
    uint16_t                handle;
} prepare_type_env_t;

void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

esp_err_t send_can_to_client(twai_message_t msg);
esp_err_t ble_app_main(void);

/* --------------------- BLE client Definitions and static variables --------------------- */

#endif