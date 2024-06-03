/*
 * SPDX-FileCopyrightText: 2010-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "main.h"

/* --------------------- BLE Definitions and static variables --------------------- */
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

#define GATTS_SERVICE_UUID      0x00FF
#define GATTS_CHAR_UUID         0xFF01
#define GATTS_DESCR_UUID        0x3333
#define GATTS_NUM_HANDLE        4

#define DEVICE_NAME             "BlueCAN"
#define MANUFACTURER_DATA_LEN   17

#define GATTS_CHAR_VAR_LEN_MAX  0x40

static uint8_t char_str[] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77};
static esp_gatt_char_prop_t property = 0;

static esp_attr_value_t gatts_char_val = {
    .attr_max_len   = GATTS_CHAR_VAR_LEN_MAX,
    .attr_len       = sizeof(char_str),
    .attr_value     = char_str,
};

static uint8_t adv_config_done = 0;

#define adv_config_flag         (1 << 0)
#define scan_rsp_config_flag    (1 << 1)

static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

/* --------------------- Definitions and static variables ------------------ */
static SemaphoreHandle_t tx_task_sem;
static SemaphoreHandle_t rx_task_sem;
static SemaphoreHandle_t ctrl_task_sem;

static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
//Set TX queue length to 0 due to listen only mode
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);

/* --------------------------- Tasks and Functions -------------------------- */

static void twai_receive_task(void *arg) {
    xSemaphoreTake(rx_task_sem, portMAX_DELAY);

    while (true) {
        twai_message_t rx_msg;
        esp_err_t ret = twai_receive(&rx_msg, pdMS_TO_TICKS(1000));
        if (ret == ESP_OK) {
            uint32_t data = 0;
            for (int i = 0; i < rx_msg.data_length_code; i++) {
                data |= (rx_msg.data[i] << (i * 8));
            }
            ESP_LOGI(APP_TAG, "Received data value %"PRIu32, data);
        }
    }

    xSemaphoreGive(rx_task_sem);
    vTaskDelete(NULL);
}

static void twai_transmit_task(void *arg) {
    while (true) {
        xSemaphoreTake(tx_task_sem, portMAX_DELAY);
        
        twai_message_t tx_msg;

        tx_msg.extd = 1;
        tx_msg.identifier = 0x1fffffff;
        tx_msg.data_length_code = 8;

        for (int i = 0; i < tx_msg.data_length_code; i++) {
            tx_msg.data[i] = i * 8;
        }

        esp_err_t ret = twai_transmit(&tx_msg, pdMS_TO_TICKS(1000));

        if (ret == ESP_OK) {
            ESP_LOGI(APP_TAG, "Transmitted test CAN Message");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));

        xSemaphoreGive(tx_task_sem);
    }

    vTaskDelete(NULL);
}

static void twai_ctrl_task(void *arg) {
    xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(APP_TAG, "Driver started");

    uint32_t alerts =   TWAI_ALERT_TX_SUCCESS       |
                        TWAI_ALERT_RX_DATA          |
                        TWAI_ALERT_ABOVE_ERR_WARN   |
                        TWAI_ALERT_BUS_ERROR        |
                        TWAI_ALERT_TX_FAILED        |
                        TWAI_ALERT_ERR_PASS         |
                        TWAI_ALERT_BUS_OFF          ;

    twai_reconfigure_alerts(alerts, NULL);
    xSemaphoreGive(tx_task_sem);

    while(true) {
        uint32_t alerts;
        twai_status_info_t twaiStatus;

        twai_read_alerts(&alerts, portMAX_DELAY);
        twai_get_status_info(&twaiStatus);

        if (alerts & TWAI_ALERT_RX_DATA) {
            xSemaphoreGive(rx_task_sem);
        }

        if (alerts & TWAI_ALERT_ABOVE_ERR_WARN) {

        }

        if (alerts & TWAI_ALERT_ERR_PASS) {
            ESP_LOGI(APP_TAG, "Alert: TWAI controller has become error passive.");
        }

        if (alerts & TWAI_ALERT_BUS_OFF) {
            ESP_LOGI(APP_TAG, "Bus Off state");
            twai_reconfigure_alerts(TWAI_ALERT_BUS_RECOVERED, NULL);
            for (int i = 3; i > 0; i--) {
                ESP_LOGW(APP_TAG, "Initiate bus recovery in %d", i);
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            twai_initiate_recovery();
            ESP_LOGI(APP_TAG, "Initiate bus recovery");
        }

        if (alerts & TWAI_ALERT_BUS_RECOVERED) {
            ESP_LOGI(APP_TAG, "Bus Recovered");
            break;
        }
    }

    xSemaphoreGive(ctrl_task_sem);
    vTaskDelete(NULL);
}

void app_main(void) {
    esp_err_t ret;

    ret = 
    tx_task_sem = xSemaphoreCreateBinary();
    rx_task_sem = xSemaphoreCreateBinary();
    ctrl_task_sem = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_transmit_task, "TWAI_tx", 4096, NULL, TX_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_ctrl_task, "TWAI_ctrl", 4096, NULL, CTRL_TASK_PRIO, NULL, tskNO_AFFINITY);

    //Install and start TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(APP_TAG, "Driver installed");

    xSemaphoreGive(ctrl_task_sem);
    vTaskDelay(pdMS_TO_TICKS(100));
    xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);

    //Stop and uninstall TWAI driver
    ESP_ERROR_CHECK(twai_stop());
    ESP_LOGI(APP_TAG, "Driver stopped");
    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(APP_TAG, "Driver uninstalled");

    //Cleanup
    vSemaphoreDelete(rx_task_sem);
    vSemaphoreDelete(tx_task_sem);
    vSemaphoreDelete(ctrl_task_sem);
}
