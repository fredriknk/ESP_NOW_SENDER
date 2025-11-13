#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include "shared_message.h"

#ifndef ESPNOW_CHANNEL
#define ESPNOW_CHANNEL 1
#endif

// Receiver's STA MAC (from your print)
uint8_t RECEIVER_MAC[6] = { 0xAC,0x67,0xB2,0x59,0x54,0x84 };

volatile esp_now_send_status_t last_status = ESP_NOW_SEND_FAIL;

Telemetry makeSample(uint32_t id) {
  Telemetry t{};
  t.msg_id = id;
  t.temperature_c = 21.5f + (id % 10) * 0.1f;
  t.battery_v = 3.87f;
  return t;
}

void onSent(const uint8_t *mac, esp_now_send_status_t status) {
  last_status = status;
  Serial.printf("Send -> %02X:%02X:%02X:%02X:%02X:%02X : %s\n",
    mac[0],mac[1],mac[2],mac[3],mac[4],mac[5],
    status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

bool connectToAnchorAP(const char* ssid) {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.persistent(false);
  WiFi.setAutoConnect(false);
  WiFi.setAutoReconnect(false);

  Serial.printf("Connecting to AP \"%s\" to lock channel...\n", ssid);
  WiFi.begin(ssid);  // open AP, visible

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 4000) {
    delay(100);
  }

  wl_status_t st = WiFi.status();
  Serial.printf("AP connect status: %d (0=IDLE,3=GOT_IP)\n", (int)st);
  Serial.printf("Current channel (Arduino): %d\n", WiFi.channel());

  // Also print via esp_wifi_get_channel for certainty
  uint8_t pri = 0; wifi_second_chan_t sec = WIFI_SECOND_CHAN_NONE;
  esp_wifi_get_channel(&pri, &sec);
  Serial.printf("esp_wifi_get_channel => pri=%u sec=%d\n", pri, (int)sec);

  // Even if we didn't get IP (open AP), association still locks the channel.
  return (st == WL_CONNECTED || WiFi.channel() == ESPNOW_CHANNEL || pri == ESPNOW_CHANNEL);
}

bool sendWithRetry(const uint8_t *peer, const uint8_t *data, size_t len,
                   int attempts = 3, int gap_ms = 40) {
  for (int i = 0; i < attempts; ++i) {
    last_status = (esp_now_send_status_t)0xFF;
    if (esp_now_send(peer, data, len) == ESP_OK) {
      unsigned long t0 = millis();
      while (last_status == (esp_now_send_status_t)0xFF && millis() - t0 < 200) { delay(1); }
      if (last_status == ESP_NOW_SEND_SUCCESS) return true;
    }
    delay(gap_ms);
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // 1) Join receiver's AP to force the channel
  if (!connectToAnchorAP("ANCHOR32")) {
    Serial.println("Failed to lock channel via AP (but trying ESPNOW anyway)...");
  }

  // 2) Now init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_send_cb(onSent);

  // 3) Add unencrypted peer (channel 0 lets ESP-NOW use current channel)
  esp_now_peer_info_t p{};
  memcpy(p.peer_addr, RECEIVER_MAC, 6);
  p.channel = 0;        // IMPORTANT: use current channel (already locked by AP)
  p.encrypt = false;
  if (esp_now_add_peer(&p) != ESP_OK) {
    Serial.println("Add peer failed");
    return;
  }

  Serial.printf("Sender STA MAC: %s | ch(Arduino)=%d\n", WiFi.macAddress().c_str(), WiFi.channel());
  uint8_t pri = 0; wifi_second_chan_t sec = WIFI_SECOND_CHAN_NONE;
  esp_wifi_get_channel(&pri, &sec);
  Serial.printf("esp_wifi_get_channel => pri=%u sec=%d\n", pri, (int)sec);
}

void loop() {
  static uint32_t id = 0;
  Telemetry t = makeSample(id++);

  bool ok = sendWithRetry(RECEIVER_MAC, (uint8_t*)&t, sizeof(t), 3, 30);
  if (!ok) {
    Serial.println("Retry exhausted");
  }
  delay(500);
}
