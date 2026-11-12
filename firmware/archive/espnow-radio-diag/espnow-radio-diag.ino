/*
 * ESP-NOW radio diagnostic - ESP32-S3 SuperMini
 *
 * Same sketch on BOTH boards. No servos, no wiring. Answers: is the radio
 * actually transmitting, and is it receiving anything at all?
 *
 * Every second it broadcasts a "hello" and prints a summary line every 3 s:
 *   send ok/fail  : MAC-layer result of each transmit (did it leave the antenna)
 *   rx            : packets delivered to the ESP-NOW receive callback (unfiltered)
 *   promisc       : ALL 802.11 frames heard on the channel (only when promisc is on)
 *
 * Serial 115200, newline. Commands:
 *   status         print counters now
 *   scan           scan for Wi-Fi networks (proves the receiver works at all)
 *   promisc on|off count every frame heard on channel 1 (off by default)
 *   power <dBm>    set TX power, 2-20 (e.g. power 8)
 *   chan <n>       set channel (both boards must match)
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>

static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
int channel = 11;

volatile uint32_t sendOk = 0, sendFail = 0, rxRaw = 0, promiscFrames = 0;
volatile int8_t lastRssi = 0;
volatile uint8_t lastSrc[6] = {0};
volatile int lastLen = 0;
uint32_t txCalls = 0, txErr = 0, helloN = 0;
unsigned long lastHello = 0, lastPrint = 0;
bool promiscOn = false;

// ---- send callback: signature changed in core 3.3 (IDF 5.5)
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 3, 0)
void onSend(const wifi_tx_info_t* info, esp_now_send_status_t status) {
#else
void onSend(const uint8_t* mac, esp_now_send_status_t status) {
#endif
  if (status == ESP_NOW_SEND_SUCCESS) sendOk++; else sendFail++;
}

// ---- receive callback: counts EVERYTHING, no filtering
void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  rxRaw++;
  lastLen = len;
  lastRssi = info->rx_ctrl ? info->rx_ctrl->rssi : 0;
  memcpy((void*)lastSrc, info->src_addr, 6);
}

void onPromisc(void* buf, wifi_promiscuous_pkt_type_t type) {
  promiscFrames++;
}

esp_err_t setChannel(int c) {
  channel = c;
  esp_err_t e = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  uint8_t primary = 0;
  wifi_second_chan_t second;
  esp_wifi_get_channel(&primary, &second);
  Serial.printf("set channel %d -> %s | radio now on channel %d\n", c, esp_err_to_name(e), primary);
  return e;
}

void printStatus() {
  uint8_t primary = 0;
  wifi_second_chan_t second;
  esp_wifi_get_channel(&primary, &second);
  Serial.printf("[ch %d] tx calls %lu (err %lu) | send ok %lu fail %lu | rx %lu | promisc %lu",
                primary, (unsigned long)txCalls, (unsigned long)txErr,
                (unsigned long)sendOk, (unsigned long)sendFail,
                (unsigned long)rxRaw, (unsigned long)promiscFrames);
  if (rxRaw > 0) {
    Serial.printf(" | last rx from %02X:%02X:%02X:%02X:%02X:%02X len %d RSSI %d",
                  lastSrc[0], lastSrc[1], lastSrc[2], lastSrc[3], lastSrc[4], lastSrc[5],
                  lastLen, lastRssi);
  }
  Serial.println();
}

void doScan() {
  Serial.println("Scanning (takes a few seconds)...");
  int n = WiFi.scanNetworks();
  Serial.printf("Scan found %d networks\n", n);
  for (int i = 0; i < n && i < 8; i++) {
    Serial.printf("  %-24s ch %2d  RSSI %d\n", WiFi.SSID(i).c_str(), WiFi.channel(i), WiFi.RSSI(i));
  }
  WiFi.scanDelete();
  setChannel(channel);   // scanning hops channels, so put it back
  if (n <= 0) Serial.println("  Nothing heard. If you know a router is nearby, the receiver or antenna is the problem.");
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (!cmd.length()) return;
  if (cmd == "status") printStatus();
  else if (cmd == "scan") doScan();
  else if (cmd == "promisc on" || cmd == "promisc off") {
    promiscOn = cmd.endsWith("on");
    esp_wifi_set_promiscuous(promiscOn);
    Serial.printf("Promiscuous %s\n", promiscOn ? "ON" : "OFF");
  }
  else if (cmd.startsWith("power ")) {
    int dbm = constrain(cmd.substring(6).toInt(), 2, 20);
    esp_err_t e = esp_wifi_set_max_tx_power(dbm * 4);
    Serial.printf("TX power %d dBm -> %s\n", dbm, esp_err_to_name(e));
  }
  else if (cmd.startsWith("chan ")) {
    setChannel(constrain(cmd.substring(5).toInt(), 1, 13));
  }
  else Serial.println("Commands: status, scan, promisc on|off, power <dBm>, chan <n>");
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n=== ESP-NOW radio diagnostic ===");
  Serial.printf("Core %d.%d.%d\n", ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);   // no modem sleep
  delay(100);
  setChannel(channel);
  Serial.println("MAC " + WiFi.macAddress());

  esp_err_t e = esp_now_init();
  Serial.printf("esp_now_init: %s\n", esp_err_to_name(e));
  if (e != ESP_OK) return;

  esp_now_register_send_cb(onSend);
  esp_now_register_recv_cb(onRecv);
  esp_wifi_set_promiscuous_rx_cb(onPromisc);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BCAST, 6);
  peer.channel = 0;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  e = esp_now_add_peer(&peer);
  Serial.printf("add broadcast peer: %s\n", esp_err_to_name(e));
  Serial.println("Commands: status, scan, promisc on|off, power <dBm>, chan <n>");
}

void loop() {
  unsigned long now = millis();
  if (now - lastHello >= 1000) {
    lastHello = now;
    char msg[40];
    int n = snprintf(msg, sizeof(msg), "hello %lu", (unsigned long)helloN++);
    txCalls++;
    esp_err_t e = esp_now_send(BCAST, (const uint8_t*)msg, n);
    if (e != ESP_OK) {
      txErr++;
      Serial.printf("esp_now_send error: %s\n", esp_err_to_name(e));
    }
  }
  if (now - lastPrint >= 3000) {
    lastPrint = now;
    printStatus();
  }
  if (Serial.available()) handleCommand(Serial.readStringUntil('\n'));
}
