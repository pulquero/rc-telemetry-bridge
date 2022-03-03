#include <esp_now.h>
#include "espnow-transport.h"
#include "debug.h"

static Telemetry* _telemetry;
bool isRunning = false;

void espnowBegin(Telemetry* telemetry) {
  if (!isRunning && telemetry->config.input.source == SOURCE_ESPNOW || telemetry->config.espnow.mode != MODE_DISABLED) {
    _telemetry = telemetry;
    if (esp_now_init() != ESP_OK) {
      LOGE("Failed to init ESP-NOW");
    }
    if (telemetry->config.input.source == SOURCE_ESPNOW) {
      if (esp_now_register_recv_cb([](const uint8_t* senderMac, const uint8_t* data, int len) {
        _telemetry->copyToIncoming(data, len, SOURCE_ESPNOW);
      }) != ESP_OK) {
        LOGE("Failed to register ESP-NOW receive callback");
      }
    }
    if (telemetry->config.espnow.mode != MODE_DISABLED) {
        esp_now_peer_info_t peerInfo;
        sprintf(telemetry->config.espnow.mac, "%02X:%02X:%02X:%02X:%02X:%02X",
            &(peerInfo.peer_addr[0]),
            &(peerInfo.peer_addr[1]),
            &(peerInfo.peer_addr[2]),
            &(peerInfo.peer_addr[3]),
            &(peerInfo.peer_addr[4]),
            &(peerInfo.peer_addr[5])
        );
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        esp_err_t rc = esp_now_add_peer(&peerInfo);
      if (rc != ESP_OK) {
        LOGE("Failed to add ESP-NOW peer: %d", rc);
      }
    }
    isRunning = true;
  }
}

void espnowStop() {
  if (isRunning) {
    if (esp_now_deinit() != ESP_OK) {
      LOGE("Failed to de-init ESP-NOW");
    }
    isRunning = false;
  }
}

void espnowWrite(const uint8_t* data, int len) {
  esp_err_t rc = esp_now_send(nullptr, data, len);
  if (rc != ESP_OK) {
    LOGE("ESP-NOW failed to send: %d", rc);
  }
}
