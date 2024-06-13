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
                ESP_LOGI("TEST", "RCV CAN MESSAGE");
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
static QueueHandle_t s_espnow_queue;

static uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint16_t s_espnow_seq[ESPNOW_DATA_MAX] = {0, 0};

/* --------------------------- ESPNOW Tasks and Functions -------------------------- */
static void wifi_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    #if CONFIG_ESPNOW_ENABLE_LONG_RANGE
        ESP_ERROR_CHECK(esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR));
    #endif
}

static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
    espnow_event_t evt;
    espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (mac_addr == NULL) {
        ESP_LOGE(APP_TAG, "%s Send cb arg error", __func__);

        return;
    }

    evt.id = ESPNOW_SEND_CB;

    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);

    send_cb->status = status;

    if (xQueueSend(s_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(APP_TAG, "%s Send send queue failed", __func__);
    }
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    espnow_event_t evt;
    espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

    uint8_t *mac_addr = recv_info->src_addr;
    uint8_t *des_addr = recv_info->des_addr;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGD(APP_TAG, "%s Receive broadcast ESPNOW data", __func__);
    } else {
        ESP_LOGD(APP_TAG, "%s Receive unicast ESPNOW data", __func__);
    }

    evt.id = ESPNOW_RECV_CB;

    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);

    recv_cb->data = malloc(len);

    if (recv_cb->data == NULL) {
        ESP_LOGE(APP_TAG, "%s Malloc receive data failed", __func__);

        return;
    }

    memcpy(recv_cb->data, data, len);

    recv_cb->data_len = len;

    if (xQueueSend(s_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(APP_TAG, "%s Send receive queue failed", __func__);

        free(recv_cb->data);
    }
}

int espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, int *magic) {
    espnow_data_t *buf = (espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(espnow_data_t)) {
        ESP_LOGE(APP_TAG, "%s Receive ESPNOW data too short, len: %d", __func__, data_len);

        return -1;
    }

    *state = buf->state;
    *seq = buf->seq_num;
    *magic = buf->magic;

    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal == crc) {
        return buf->type;
    }

    return -1;
}

void espnow_data_prepare(espnow_send_param_t *send_param) {
    espnow_data_t *buf = (espnow_data_t *)send_param->buffer;

    assert(send_param->len >= sizeof(espnow_data_t));

    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? ESPNOW_DATA_BROADCAST : ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = s_espnow_seq[buf->type]++;
    buf->crc = 0;
    buf->magic = send_param->magic;

    esp_fill_random(buf->payload, send_param->len - sizeof(espnow_data_t));

    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}

static void espnow_task(void *pvParameter) {
    espnow_event_t evt;
    
    uint8_t recv_state = 0;
    uint16_t recv_seq = 0;

    int recv_magic = 0;
    int ret = 0;

    bool is_broadcast = false;

    vTaskDelay(5000 / portTICK_PERIOD_MS);

    ESP_LOGI(APP_TAG, "%s Start sending broadcast data", __func__);

    espnow_send_param_t *send_param = (espnow_send_param_t *)pvParameter;

    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
        ESP_LOGE(APP_TAG, "%s Send Error", __func__);

        espnow_deinit(send_param);
        vTaskDelete(NULL);
    }

    while (xQueueReceive(s_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            case ESPNOW_SEND_CB: {
                espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

                ESP_LOGD(APP_TAG, "%s, Send data to "MACSTR", status 1: %d", __func__, MAC2STR(send_cb->mac_addr), send_cb->status);

                if (is_broadcast && (send_param->broadcast == false)) {
                    break;
                }

                if (!is_broadcast) {
                    send_param->count--;

                    if (send_param->count == 0) {
                        ESP_LOGI(APP_TAG, "%s Send Done", __func__);

                        espnow_deinit(send_param);
                        vTaskDelete(NULL);
                    }
                }

                if (send_param->delay > 0) {
                    vTaskDelay(send_param->delay / portTICK_PERIOD_MS);
                }

                ESP_LOGI(APP_TAG, "%s send data to "MACSTR"", __func__, MAC2STR(send_cb->mac_addr));

                memcpy(send_param->dest_mac, send_cb->mac_addr, ESP_NOW_ETH_ALEN);
                espnow_data_prepare(send_param);

                if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                    ESP_LOGE(APP_TAG, "%s Send Error", __func__);

                    espnow_deinit(send_param);
                    vTaskDelete(NULL);
                }

                break;
            }

            case ESPNOW_RECV_CB: {
                espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

                ret = espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &recv_magic);

                free(recv_cb->data);

                if (ret == ESPNOW_DATA_BROADCAST) {
                    ESP_LOGI(APP_TAG, "%s Receive %dth broadcast data from: "MACSTR", len: %d", __func__, recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    if (esp_now_is_peer_exist(recv_cb->mac_addr) == false) {
                        esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));

                        if (peer == NULL) {
                            ESP_LOGE(APP_TAG, "%s Malloc peer information failed", __func__);

                            espnow_deinit(send_param);
                            vTaskDelete(NULL);
                        }

                        memset(peer, 0, sizeof(esp_now_peer_info_t));

                        peer->channel = CONFIG_ESPNOW_CHANNEL;
                        peer->ifidx = ESPNOW_WIFI_IF;
                        peer->encrypt = true;

                        memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
                        memcpy(peer->peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);

                        ESP_ERROR_CHECK(esp_now_add_peer(peer));
                        
                        free(peer);
                    }

                    if (send_param->state == 0) {
                        send_param->state = 1;
                    }

                    if (recv_state == 1) {
                        if (send_param->unicast == false && send_param->magic >= recv_magic) {
                            ESP_LOGI(APP_TAG, "%s Start sending unicast data", __func__);
                            ESP_LOGI(APP_TAG, "%s send data to "MACSTR"", __func__, MAC2STR(recv_cb->mac_addr));

                            memcpy(send_param->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);

                            espnow_data_prepare(send_param);

                            if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                                ESP_LOGE(APP_TAG, "%s Send Error", __func__);
                                
                                espnow_deinit(send_param);
                                vTaskDelete(NULL);
                            } else {
                                send_param->broadcast = false;
                                send_param->unicast = true;
                            }
                        }
                    }
                } else if (ret == ESPNOW_DATA_UNICAST) {
                    ESP_LOGI(APP_TAG, "%s Receive %cth unicast data from: "MACSTR", len: %d", __func__, recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    send_param->broadcast = false;
                } else {
                    ESP_LOGI(APP_TAG, "%s Receive error data from: "MACSTR"", __func__, MAC2STR(recv_cb->mac_addr));
                }

                break;
            }

            default: {
                break;
            }
        }
    }
}

static esp_err_t espnow_init(void) {
    espnow_send_param_t *send_param;

    s_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));

    if (s_espnow_queue == NULL) {
        ESP_LOGE(APP_TAG, "%s Create mutex failed", __func__);

        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    #if CONFIG_ESPNOW_ENABLE_POWER_SAVE
        ESP_ERROR_CHECK(esp_now_set_wake_window(CONFIG_ESPNOW_WAKE_WINDOW));
        ESP_ERROR_CHECK(esp_wifi_connectionless_module_set_wake_interval(CONFIG_ESPNOW_WAKE_INTERVAL));
    #endif

    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK));

    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));

    if (peer == NULL) {
        ESP_LOGE(APP_TAG, "%s Malloc peer information failed", __func__);

        vSemaphoreDelete(s_espnow_queue);
        esp_now_deinit();

        return ESP_FAIL;
    }

    memset(peer, 0, sizeof(esp_now_peer_info_t));

    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;

    memcpy(peer->peer_addr, s_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    free(peer);

    send_param = malloc(sizeof(espnow_send_param_t));

    if (send_param == NULL) {
        ESP_LOGE(APP_TAG, "%s Malloc send parameter failed", __func__);

        vSemaphoreDelete(s_espnow_queue);
        esp_now_deinit();

        return ESP_FAIL;
    }

    memset(send_param, 0, sizeof(espnow_send_param_t));

    send_param->unicast = false;
    send_param->broadcast = true;
    send_param->state = 0;
    send_param->magic = esp_random();
    send_param->count = CONFIG_ESPNOW_SEND_COUNT;
    send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
    send_param->len = CONFIG_ESPNOW_SEND_LEN;
    send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);

    if (send_param->buffer == NULL) {
        ESP_LOGE(APP_TAG, "%s Malloc send buffer failed", __func__);

        free(send_param);

        vSemaphoreDelete(s_espnow_queue);
        esp_now_deinit();

        return ESP_FAIL;
    }

    memcpy(send_param->dest_mac, s_broadcast_mac, ESP_NOW_ETH_ALEN);
    espnow_data_prepare(send_param);

    xTaskCreate(espnow_task, "ESPNOW_task", 2048, send_param, ESPNOW_TASK_PRIO, NULL);

    return ESP_OK;
}

static void espnow_deinit(espnow_send_param_t *send_param) {
    free(send_param->buffer);
    free(send_param);

    vSemaphoreDelete(s_espnow_queue);

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
