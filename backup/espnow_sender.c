#include "main.h"

static const char *TAG = "ESP_NOW_SENDER";

uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = {0x24, 0x6F, 0x28, 0xAE, 0x5B, 0xC4};

typedef struct message {
    char a[32];
    int b;
    float c;
    bool d;
} ESPNOW_Message_t;

ESPNOW_Message_t myData;

void wifi_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));

}

void send_callback(const uint8_t *macAddr, esp_now_send_status_t status) {
    ESP_LOGI(TAG, "Last Packet Send Status: %s:", status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void rcv_callback(const uint8_t *macAddr, const uint8_t *rcvData, int len) {
    ESPNOW_Message_t rcvMsg;

    memcpy(&rcvMsg, rcvData, sizeof(rcvMsg));

    ESP_LOGI(TAG, "Received data from peer:");
    ESP_LOGI(TAG, "Char: %s:", rcvMsg.a);
    ESP_LOGI(TAG, "Int: %d", rcvMsg.b);
    ESP_LOGI(TAG, "Float: %.2f", rcvMsg.c);
    ESP_LOGI(TAG, "Bool: %d", rcvMsg.d);
}

esp_err_t espnow_sender_init(void) {
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());

}