/*
 * SPDX-FileCopyrightText: 2010-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "main.h"

/* --------------------- TWAI Definitions and static variables ------------------ */
static SemaphoreHandle_t ctrl_task_sem;
static SemaphoreHandle_t status_task_sem;

static TaskHandle_t ctrl_task_handle = NULL;
static TaskHandle_t status_task_handle = NULL;

static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
//Set TX queue length to 0 due to listen only mode
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);

uint32_t alerts =   TWAI_ALERT_TX_SUCCESS       |
                    TWAI_ALERT_RX_DATA          |
                    TWAI_ALERT_ABOVE_ERR_WARN   |
                    TWAI_ALERT_BUS_ERROR        |
                    TWAI_ALERT_TX_FAILED        |
                    TWAI_ALERT_ERR_PASS         |
                    TWAI_ALERT_BUS_OFF          |
                    TWAI_ALERT_RX_FIFO_OVERRUN;

/* --------------------------- TWAI Tasks and Functions -------------------------- */
static void twai_init(void) {
    ctrl_task_sem = xSemaphoreCreateBinary();
    status_task_sem = xSemaphoreCreateBinary();

    ESP_LOGI(APP_TAG, "task semaphore create done");

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
    vSemaphoreDelete(ctrl_task_sem);
    vSemaphoreDelete(status_task_sem);
    ESP_LOGI(APP_TAG, "semaphore delete done");
}

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

static void twai_ctrl_task(void *arg) {
    xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(APP_TAG, "Driver started");
    ESP_LOGI(APP_TAG, "Starting transmissions");

    while(true) {
        uint32_t read_alert;

        twai_read_alerts(&read_alert, portMAX_DELAY);

        if (read_alert & TWAI_ALERT_RX_DATA) {
            twai_message_t rx_msg;

            esp_err_t ret = twai_receive(&rx_msg, pdMS_TO_TICKS(1000));
            if (ret == ESP_OK) {
                esp_err_t ret = esp_now_send(dest_mac_addr, &rx_msg, sizeof(rx_msg));
                if (ret != ESP_OK) {
                    ESP_LOGE(APP_TAG, "%s: espnow send error -> %s", __func__, esp_err_to_name(ret));
                }
            }
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
            
            xTaskCreatePinnedToCore(twai_status_task, "TWAI_status", 4096, NULL, STATUS_TASK_PRIO, &status_task_handle, tskNO_AFFINITY);
            ESP_LOGI(APP_TAG, "Alert: Transmit task recreated");
            ESP_LOGI(APP_TAG, "Alert: Status task recreated");

            xSemaphoreGive(status_task_sem);
        }

        if (read_alert & TWAI_ALERT_BUS_RECOVERED) {
            ESP_LOGI(APP_TAG, "Alert: Bus Restart");
        }
    }

    xSemaphoreGive(ctrl_task_sem);
    vTaskDelete(NULL);
}

/* --------------------- ESPNOW Definitions and static variables ------------------ */


/* --------------------------- ESPNOW Tasks and Functions -------------------------- */
static void wifi_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));

    ESP_ERROR_CHECK(esp_wifi_set_mac(ESPNOW_WIFI_IF, mac_addr));

    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    #if CONFIG_ESPNOW_ENABLE_LONG_RANGE
        ESP_ERROR_CHECK(esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR));
    #endif
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    // espnow_event_t evt;
    // espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

    uint8_t *mac_addr = recv_info->src_addr;
    uint8_t *des_addr = recv_info->des_addr;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGD(APP_TAG, "%s: Receive broadcast ESPNOW data", __func__);
    } else {
        ESP_LOGD(APP_TAG, "%s: Receive unicast ESPNOW data", __func__);
    }

    if (len == sizeof(twai_message_t)) {
        twai_message_t msg;
        memcpy(&msg, data, len);

        msg.self = 0;

        esp_err_t ret = twai_transmit(&msg, pdMS_TO_TICKS(1000));

        if (ret != ESP_OK) {
            ESP_LOGE(APP_TAG, "%s: TWAI Transmit Error", __func__);
        }
    }
}

static esp_err_t espnow_init(void) {
    // espnow_send_param_t *send_param;

    ESP_ERROR_CHECK(esp_now_init());
    // ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    #if CONFIG_ESPNOW_ENABLE_POWER_SAVE
        ESP_ERROR_CHECK(esp_now_set_wake_window(CONFIG_ESPNOW_WAKE_WINDOW));
        ESP_ERROR_CHECK(esp_wifi_connectionless_module_set_wake_interval(CONFIG_ESPNOW_WAKE_INTERVAL));
    #endif

    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK));

    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));

    if (peer == NULL) {
        ESP_LOGE(APP_TAG, "%s: Malloc peer information failed", __func__);

        esp_now_deinit();

        return ESP_FAIL;
    }

    memset(peer, 0, sizeof(esp_now_peer_info_t));

    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;

    memcpy(peer->peer_addr, dest_mac_addr, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    free(peer);

    return ESP_OK;
}

static void espnow_deinit(espnow_send_param_t *send_param) {
    free(send_param->buffer);
    free(send_param);

    esp_now_deinit();
}

/* --------------------------- APP Start position -------------------------- */
void app_main(void) {
    ESP_LOGI(APP_TAG, "app_main() open");

    esp_err_t ret;

    ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init();
    espnow_init();
    twai_init();

    ESP_LOGE(APP_TAG, "app_main() close");
}
