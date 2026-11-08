// Team 16 search-and-rescue homing beacon.
//
// Transmits steady, recognisable signals so rescuers can direction-find to it:
//   * LoRa (ENABLE_LORA, LilyGO LoRa32 915 MHz / SX1276): a short packet every LORA_INTERVAL_MS. This is the
//     long-range signal (sub-GHz goes through trees far better than 2.4 GHz). Needs a LoRa receiver:
//     build the esp32-pico-rx environment on a second board and watch RSSI, or use a directional antenna.
//   * Wi-Fi (ENABLE_WIFI_BEACON): raw 802.11 beacon frames on ONE fixed channel, SSID BEACON_SSID. Any
//     phone shows it with signal strength. Short range, handy for testing.
//   * BLE (ENABLE_BLE): advertising every 20 ms under BLE_NAME. Also phone-friendly.
// It never hops channels and never joins a network. LORA_RX_MODE turns the same board into the receiver.
#include <Arduino.h>

#ifndef ENABLE_WIFI_BEACON
#define ENABLE_WIFI_BEACON 1
#endif
#ifndef ENABLE_BLE
#define ENABLE_BLE 0
#endif
#ifndef ENABLE_LORA
#define ENABLE_LORA 0
#endif
#ifndef LORA_RX_MODE
#define LORA_RX_MODE 0
#endif
#ifndef BEACON_CHANNEL
#define BEACON_CHANNEL 11
#endif
#ifndef BEACON_RATE_HZ
#define BEACON_RATE_HZ 50
#endif
#ifndef BEACON_SSID
#define BEACON_SSID "TEAM16-DF"
#endif
#ifndef LORA_INTERVAL_MS
#define LORA_INTERVAL_MS 1000
#endif

#if ENABLE_WIFI_BEACON && !LORA_RX_MODE
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#endif
#if ENABLE_BLE && !LORA_RX_MODE
#include <BLEDevice.h>
#include <BLEAdvertising.h>
#endif
#if ENABLE_LORA
#include <SPI.h>
#include <LoRa.h>
#endif

static uint8_t mac[6];

// ------------------------------------------------------------------ Wi-Fi beacon frames
#if ENABLE_WIFI_BEACON && !LORA_RX_MODE
static uint8_t frame[128];
static size_t frameLen = 0;
static uint16_t seqNum = 0;
static uint32_t wifiSent = 0, wifiFailed = 0;

static void buildBeacon() {
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

static void wifiSetup() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  esp_err_t chErr = esp_wifi_set_channel(BEACON_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (chErr != ESP_OK) Serial.printf("WARNING: set_channel(%d) failed: %d\n", BEACON_CHANNEL, chErr);
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  buildBeacon();
  Serial.printf("Wi-Fi beacon: ch %d, %d frames/s, SSID %s\n", BEACON_CHANNEL, BEACON_RATE_HZ, BEACON_SSID);
}

static void wifiBeacon() {
  static uint32_t last = 0;
  if (millis() - last < 1000 / BEACON_RATE_HZ) return;
  last = millis();
  uint64_t t = esp_timer_get_time();
  memcpy(&frame[24], &t, 8);                          // beacon timestamp
  uint16_t sc = (seqNum++ & 0x0FFF) << 4;
  frame[22] = sc & 0xFF; frame[23] = sc >> 8;
  if (esp_wifi_80211_tx(WIFI_IF_STA, frame, frameLen, false) == ESP_OK) wifiSent++; else wifiFailed++;
}
#endif

// ------------------------------------------------------------------ LoRa
#if ENABLE_LORA
static bool loraOk = false;
static uint32_t loraSeq = 0, loraSent = 0;
static volatile bool loraTxDone = true;
static void onLoraTxDone() { loraTxDone = true; }

static void loraSetup() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  loraOk = LoRa.begin(LORA_FREQ);
  if (!loraOk) {
    Serial.println("ERROR: LoRa radio did not start (check pins / board variant)");
    return;
  }
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0x16);              // private network id, ignores LoRaWAN traffic
  LoRa.setPreambleLength(8);
#if LORA_RX_MODE
  LoRa.enableCrc();
  LoRa.receive();
  Serial.printf("LoRa RECEIVER: %.1f MHz SF%d BW%.0f kHz - printing RSSI of every beacon heard\n",
                LORA_FREQ / 1e6, LORA_SF, LORA_BW / 1e3);
#else
  LoRa.setTxPower(LORA_TX_DBM, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.enableCrc();
  LoRa.onTxDone(onLoraTxDone);
  Serial.printf("LoRa beacon: %.1f MHz SF%d BW%.0f kHz %d dBm, one packet every %d ms\n",
                LORA_FREQ / 1e6, LORA_SF, LORA_BW / 1e3, LORA_TX_DBM, LORA_INTERVAL_MS);
#endif
}

#if !LORA_RX_MODE
static void loraBeacon() {
  static uint32_t last = 0;
  if (!loraOk || !loraTxDone || millis() - last < LORA_INTERVAL_MS) return;
  last = millis();
  loraTxDone = false;
  char msg[40];
  snprintf(msg, sizeof(msg), "T16DF,%02X%02X%02X,%lu,%lu", mac[3], mac[4], mac[5],
           (unsigned long)loraSeq++, (unsigned long)(millis() / 1000));
  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket(true);                // async: the Wi-Fi/BLE beacons keep running while it transmits
  loraSent++;
}
#else
static void loraListen() {
  int size = LoRa.parsePacket();
  if (!size) return;
  String s;
  while (LoRa.available()) s += (char)LoRa.read();
  Serial.printf("RX rssi=%d dBm snr=%.1f dB len=%d  %s\n", LoRa.packetRssi(), LoRa.packetSnr(), size, s.c_str());
}
#endif
#endif

// ------------------------------------------------------------------ main
void setup() {
  Serial.begin(115200);
  delay(1500);
#if ENABLE_WIFI_BEACON && !LORA_RX_MODE
  wifiSetup();
#else
  {  // no Wi-Fi started: take the id from the chip's factory MAC
    uint64_t m = ESP.getEfuseMac();
    for (int i = 0; i < 6; i++) mac[i] = (m >> (8 * i)) & 0xFF;
  }
#endif
  Serial.printf("TEAM16 DF %s | id %02x:%02x:%02x:%02x:%02x:%02x\n", LORA_RX_MODE ? "RECEIVER" : "beacon",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
#if ENABLE_BLE && !LORA_RX_MODE
  BLEDevice::init(BLE_NAME);
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->setMinInterval(32);   // 20 ms (units of 0.625 ms)
  adv->setMaxInterval(32);
  BLEDevice::startAdvertising();
  Serial.println("BLE advertising started");
#endif
#if ENABLE_LORA
  loraSetup();
#endif
}

void loop() {
#if ENABLE_WIFI_BEACON && !LORA_RX_MODE
  wifiBeacon();
#endif
#if ENABLE_LORA
#if LORA_RX_MODE
  loraListen();
#else
  loraBeacon();
#endif
#endif
#if !LORA_RX_MODE
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 5000) {
    lastPrint = millis();
    Serial.printf("status:");
#if ENABLE_WIFI_BEACON
    Serial.printf(" wifi sent %lu failed %lu", (unsigned long)wifiSent, (unsigned long)wifiFailed);
#endif
#if ENABLE_LORA
    Serial.printf(" | lora sent %lu", (unsigned long)loraSent);
#endif
    Serial.println();
  }
#endif
  delay(1);
}
