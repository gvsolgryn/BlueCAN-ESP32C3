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

#endif