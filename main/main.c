/*
 * SPDX-FileCopyrightText: 2010-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "main.h"

/* --------------------- Definitions and static variables ------------------ */
static SemaphoreHandle_t tx_task_sem;
static SemaphoreHandle_t rx_task_sem;
static SemaphoreHandle_t ctrl_task_sem;
static SemaphoreHandle_t status_task_sem;

static TaskHandle_t tx_task_handle = NULL;
static TaskHandle_t rx_task_handle = NULL;
static TaskHandle_t ctrl_task_handle = NULL;
static TaskHandle_t status_task_handle = NULL;

static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
//Set TX queue length to 0 due to listen only mode
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);

static twai_message_t test_msg = {
    .extd               = 1,
    .identifier         = 0x1FFFFFFF,
    .data_length_code   = 8,
    .data[0]            = 0x99,
    .data[1]            = 0x88,
    .data[2]            = 0x77,
    .data[3]            = 0x66,
    .data[4]            = 0x55,
    .data[5]            = 0x44,
    .data[6]            = 0x33,
    .data[7]            = 0x22,
};

uint32_t alerts =   TWAI_ALERT_TX_SUCCESS       |
                    TWAI_ALERT_RX_DATA          |
                    TWAI_ALERT_ABOVE_ERR_WARN   |
                    TWAI_ALERT_BUS_ERROR        |
                    TWAI_ALERT_TX_FAILED        |
                    TWAI_ALERT_ERR_PASS         |
                    TWAI_ALERT_BUS_OFF          |
                    TWAI_ALERT_RX_FIFO_OVERRUN;

extern bool is_client_connected;

/* --------------------------- Tasks and Functions -------------------------- */

void twai_status_task(void *arg) {
    xSemaphoreTake(status_task_sem, portMAX_DELAY);

    twai_status_info_t xStatusInfo;

    while (true) {
        if (twai_get_status_info(&xStatusInfo) == ESP_OK) {
            ESP_LOGI(APP_TAG, "State: %d, TX error: %lu, RX error: %lu, TX buffer: %lu, RX buffer: %lu",
                        xStatusInfo.state,
                        xStatusInfo.tx_error_counter,
                        xStatusInfo.rx_error_counter,
                        xStatusInfo.msgs_to_tx,
                        xStatusInfo.msgs_to_rx);

            if (xStatusInfo.state == TWAI_STATE_BUS_OFF) {
                twai_initiate_recovery();
            }
            if (xStatusInfo.state == TWAI_STATE_STOPPED) {
                break;
            }
        } else {
            ESP_LOGE(APP_TAG, "Failed to get TWAI status info");

            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    xSemaphoreGive(status_task_sem);
    vTaskDelete(NULL);
}

static void twai_transmit_task(void *arg) {
    xSemaphoreTake(tx_task_sem, portMAX_DELAY);
    twai_message_t *tx_msg = (twai_message_t *)arg;
    while (true) {
        esp_err_t ret = twai_transmit(&test_msg, 0);

        if (ret == ESP_ERR_INVALID_STATE) {
            ESP_LOGW(APP_TAG, "Transmitted test failed - ESP_ERR_INVALID_STATE");
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    xSemaphoreGive(tx_task_sem);
    vTaskDelete(NULL);
}

static void twai_ctrl_task(void *arg) {
    xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(APP_TAG, "Driver started");
    ESP_LOGI(APP_TAG, "Starting transmissions");
    xSemaphoreGive(tx_task_sem);

    int error_cnt = 0;

    while(true) {
        uint32_t read_alert;

        twai_read_alerts(&read_alert, portMAX_DELAY);

        if (read_alert & TWAI_ALERT_RX_DATA) {
            twai_message_t rx_msg;
            esp_err_t ret = twai_receive(&rx_msg, pdMS_TO_TICKS(1000));
            if (ret == ESP_OK) {
                uint32_t data = 0;
                for (int i = 0; i < rx_msg.data_length_code; i++) {
                    data |= (rx_msg.data[i] << (i * 8));
                }
                ESP_LOGI(APP_TAG, "Received data value %"PRIu32, data);
            }
            send_can_to_client(rx_msg);
        }

        if (read_alert & TWAI_ALERT_ABOVE_ERR_WARN) {
            ESP_LOGI(APP_TAG, "Alert: Surpassed Error Warning Limit");
        }

        if (read_alert & TWAI_ALERT_BUS_ERROR) {
            ESP_LOGE(APP_TAG, "Alert: TWAI Bus Error cnt");
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (read_alert & TWAI_ALERT_TX_FAILED) {
            ESP_LOGW(APP_TAG, "Alert: TX Failed");
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (read_alert & TWAI_ALERT_ERR_PASS) {
            ESP_LOGI(APP_TAG, "Alert: TWAI controller has become error passive");
        }

        if (read_alert & TWAI_ALERT_BUS_OFF) {
            ESP_LOGE(APP_TAG, "Bus Off state");
            
            xSemaphoreTake(tx_task_sem, portMAX_DELAY);
            xSemaphoreTake(status_task_sem, portMAX_DELAY);

            for (int i = 3; i > 0; i--) {
                ESP_LOGW(APP_TAG, "Warn: Initiate bus recovery in %d", i);
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            twai_initiate_recovery();    //Needs 128 occurrences of bus free signal
            ESP_LOGI(APP_TAG, "Alert: Initiate bus recovery");
            
            //Uninstall TWAI driver
            ESP_ERROR_CHECK(twai_driver_uninstall());
            ESP_LOGI(APP_TAG, "Alert: Driver uninstalled");

            ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
            ESP_LOGI(APP_TAG, "Alert: Driver installed");

            twai_reconfigure_alerts(alerts, NULL);
            ESP_LOGI(APP_TAG, "Alert: alert reconfigure done");

            ESP_ERROR_CHECK(twai_start());
            ESP_LOGI(APP_TAG, "Alert: Driver started");

            xTaskCreatePinnedToCore(twai_transmit_task, "TWAI_tx", 4096, NULL, TX_TASK_PRIO, &tx_task_handle, tskNO_AFFINITY);
            xTaskCreatePinnedToCore(twai_status_task, "TWAI_status", 4096, NULL, STATUS_TASK_PRIO, &status_task_handle, tskNO_AFFINITY);
            ESP_LOGI(APP_TAG, "Alert: Transmit task recreated");
            ESP_LOGI(APP_TAG, "Alert: Status task recreated");

            xSemaphoreGive(tx_task_sem);
            xSemaphoreGive(status_task_sem);
        }

        if (read_alert & TWAI_ALERT_BUS_RECOVERED) {
            ESP_LOGI(APP_TAG, "Alert: Bus Restart");
        }
    }

    xSemaphoreGive(ctrl_task_sem);
    vTaskDelete(NULL);
}

void app_main(void) {
    ESP_LOGI(APP_TAG, "app_main() open");

    ESP_LOGI(APP_TAG, "%s ble init", __func__);
    ESP_ERROR_CHECK(ble_server_app_main());

    tx_task_sem = xSemaphoreCreateBinary();
    ctrl_task_sem = xSemaphoreCreateBinary();
    status_task_sem = xSemaphoreCreateBinary();
    ESP_LOGI(APP_TAG, "task semaphore create done");

    xTaskCreatePinnedToCore(twai_transmit_task, "TWAI_tx", 4096, NULL, TX_TASK_PRIO, &tx_task_handle, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_ctrl_task, "TWAI_ctrl", 4096, NULL, CTRL_TASK_PRIO, &ctrl_task_handle, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_status_task, "TWAI_status", 4096, NULL, STATUS_TASK_PRIO, &status_task_handle, tskNO_AFFINITY);
    ESP_LOGI(APP_TAG, "task create done");

    //Install and start TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(APP_TAG, "Driver installed");

    twai_reconfigure_alerts(alerts, NULL);
    ESP_LOGI(APP_TAG, "alert reconfigure done");
    
    xSemaphoreGive(ctrl_task_sem);
    xSemaphoreGive(status_task_sem);
    vTaskDelay(pdMS_TO_TICKS(100));
    xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);

    //Uninstall TWAI driver
    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(APP_TAG, "Driver uninstalled");

    //Cleanup
    vSemaphoreDelete(tx_task_sem);
    vSemaphoreDelete(ctrl_task_sem);
    vSemaphoreDelete(status_task_sem);
    ESP_LOGI(APP_TAG, "semaphore delete done");

    ESP_LOGE(APP_TAG, "app_main() close");
}
