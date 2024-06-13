#ifndef _MAIN_
#define _MAIN_

/*
 * SPDX-FileCopyrightText: 2010-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_random.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"

#include "esp_now.h"
#include "esp_crc.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_netif.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "sdkconfig.h"

#include "driver/twai.h"

/* --------------------- TWAI Definitions and static variables ------------------ */
#define CTRL_TASK_PRIO      5
#define STATUS_TASK_PRIO    6

#define TX_GPIO_NUM     CONFIG_TWAI_TX_GPIO_NUM
#define RX_GPIO_NUM     CONFIG_TWAI_RX_GPIO_NUM

static void twai_init(void);
void twai_status_task(void *arg);
static void twai_ctrl_task(void *arg);

#endif // _MAIN_ ifndef eof


#ifndef ESP_NOW_DEF
#define ESP_NOW_DEF

#define ESPNOW_WIFI_MODE_SOFTAP     0
#define ESPNOW_WIFI_MODE_STATION    1
#define ESPNOW_WIFi_MODE_SEL        ESPNOW_WIFI_MODE_SOFTAP

/* --------------------- ESPNOW Definitions and static variables ------------------ */
#if ESPNOW_WIFI_MODE_SEL == ESPNOW_WIFI_MODE_SOFTAP
    #define ESPNOW_WIFI_MODE    WIFI_MODE_AP
    #define ESPNOW_WIFI_IF      ESP_IF_WIFI_AP
    #define APP_TAG     "NOWCAN_AP"
#elif ESPNOW_WIFI_MODE_SEL == ESPNOW_WIFI_MODE_STATION
    #define ESPNOW_WIFI_MODE    WIFI_MODE_STA
    #define ESPNOW_WIFI_IF      ESP_IF_WIFI_STA
    #define APP_TAG     "NOWCAN_STA"
#else
    #if CONFIG_ESPNOW_WIFI_MODE_STATION
        #define ESPNOW_WIFI_MODE    WIFI_MODE_STA
        #define ESPNOW_WIFI_IF      ESP_IF_WIFI_STA
        #define APP_TAG     "NOWCAN_STA"
    #else
        #define ESPNOW_WIFI_MODE    WIFI_MODE_AP
        #define ESPNOW_WIFI_IF      ESP_IF_WIFI_AP
        #define APP_TAG     "NOWCAN_AP"
    #endif
#endif

#define ESPNOW_TASK_PRIO 3

#define ESPNOW_MAXDELAY 512

#define ESPNOW_QUEUE_SIZE 24

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

typedef enum {
    ESPNOW_SEND_CB,
    ESPNOW_RECV_CB,
} espnow_event_id_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} espnow_event_send_cb_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
} espnow_event_recv_cb_t;

typedef union {
    espnow_event_send_cb_t send_cb;
    espnow_event_recv_cb_t recv_cb;
} espnow_event_info_t;

typedef struct {
    espnow_event_id_t id;
    espnow_event_info_t info;
} espnow_event_t;

enum {
    ESPNOW_DATA_BROADCAST,
    ESPNOW_DATA_UNICAST,
    ESPNOW_DATA_MAX,
};

typedef struct {
    uint8_t type;
    uint8_t state;
    uint16_t seq_num;
    uint16_t crc;
    uint32_t magic;
    uint8_t payload[0];
} __attribute__((packed)) espnow_data_t;

typedef struct {
    bool unicast;
    bool broadcast;
    uint8_t state;
    uint32_t magic;
    uint16_t count;
    uint16_t delay;
    int len;
    uint8_t *buffer;
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];
} espnow_send_param_t;

/* --------------------------- ESPNOW Functions -------------------------- */
static void wifi_init(void);

static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);

int espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, int *magic);

void espnow_data_prepare(espnow_send_param_t *send_param);

static void espnow_task(void *pvParameter);

static esp_err_t espnow_init(void);

static void espnow_deinit(espnow_send_param_t *send_param);

#endif // ESP_NOW_DEF ifndef eof