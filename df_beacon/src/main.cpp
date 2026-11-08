// Team 16 search-and-rescue homing beacon (ESP32-S3).
//
// Transmits a steady, recognisable 2.4 GHz signal on ONE fixed channel so rescuers can direction-find
// to it (walk toward rising signal strength, or sweep a directional antenna for the peak):
//   * Wi-Fi: raw 802.11 beacon frames, BEACON_RATE_HZ per second, SSID BEACON_SSID. Any phone/laptop
//            Wi-Fi scanner shows it with signal strength; a second ESP32 or SDR can lock on to it.
//   * BLE:   advertising every 20 ms under BLE_NAME (optional, ENABLE_BLE).
// It never hops channels and never joins a network, so it does not disturb other 2.4 GHz users beyond
// the beacon itself. Transmit power is the stock maximum for the module; add no amplifier.
#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#if ENABLE_BLE
#include <BLEDevice.h>
#include <BLEAdvertising.h>
#endif

#ifndef BEACON_CHANNEL
#define BEACON_CHANNEL 6
#endif
#ifndef BEACON_RATE_HZ
#define BEACON_RATE_HZ 50
#endif
#ifndef BEACON_SSID
#define BEACON_SSID "TEAM16-DF"
#endif

static uint8_t frame[128];
static size_t frameLen = 0;
static uint16_t seqNum = 0;
static uint32_t sent = 0, failed = 0;

static void buildBeacon(const uint8_t mac[6]) {
  uint8_t *p = frame;
  *p++ = 0x80; *p++ = 0x00;                 // frame control: beacon
  *p++ = 0x00; *p++ = 0x00;                 // duration
  for (int i = 0; i < 6; i++) *p++ = 0xFF;  // destination: broadcast
  memcpy(p, mac, 6); p += 6;                // source
  memcpy(p, mac, 6); p += 6;                // BSSID
  *p++ = 0x00; *p++ = 0x00;                 // sequence control (filled per frame)
  for (int i = 0; i < 8; i++) *p++ = 0x00;  // timestamp (filled per frame)
  *p++ = 0x64; *p++ = 0x00;                 // beacon interval 100 TU
  *p++ = 0x01; *p++ = 0x04;                 // capabilities: ESS, short slot (open network)
  const char *ssid = BEACON_SSID;
  uint8_t n = strlen(ssid);
  *p++ = 0x00; *p++ = n; memcpy(p, ssid, n); p += n;                       // SSID
  static const uint8_t rates[] = {0x01, 0x08, 0x82, 0x84, 0x8B, 0x96, 0x24, 0x30, 0x48, 0x6C};
  memcpy(p, rates, sizeof(rates)); p += sizeof(rates);                     // supported rates
  *p++ = 0x03; *p++ = 0x01; *p++ = BEACON_CHANNEL;                         // DS parameter set
  frameLen = p - frame;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  esp_err_t chErr = esp_wifi_set_channel(BEACON_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (chErr != ESP_OK) Serial.printf("WARNING: set_channel(%d) failed: %d\n", BEACON_CHANNEL, chErr);
  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  buildBeacon(mac);
  Serial.printf("TEAM16 DF beacon | MAC %02x:%02x:%02x:%02x:%02x:%02x | Wi-Fi ch %d | %d frames/s | SSID %s\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], BEACON_CHANNEL, BEACON_RATE_HZ, BEACON_SSID);
#if ENABLE_BLE
  BLEDevice::init(BLE_NAME);
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->setMinInterval(32);   // 20 ms (units of 0.625 ms)
  adv->setMaxInterval(32);
  BLEDevice::startAdvertising();
  Serial.println("BLE advertising started");
#endif
}

void loop() {
  static uint32_t lastPrint = 0;
  uint64_t t = esp_timer_get_time();
  memcpy(&frame[24], &t, 8);                          // beacon timestamp
  uint16_t sc = (seqNum++ & 0x0FFF) << 4;
  frame[22] = sc & 0xFF; frame[23] = sc >> 8;
  if (esp_wifi_80211_tx(WIFI_IF_STA, frame, frameLen, false) == ESP_OK) sent++; else failed++;
  if (millis() - lastPrint >= 5000) {
    lastPrint = millis();
    Serial.printf("beacons sent %lu, failed %lu\n", (unsigned long)sent, (unsigned long)failed);
  }
  delay(1000 / BEACON_RATE_HZ);
}
