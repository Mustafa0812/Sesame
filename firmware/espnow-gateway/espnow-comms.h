#pragma once
// ======================================================================
// ESP-NOW communication layer (generic, no robot behaviours yet)
// ======================================================================
// What this provides:
//   - Broadcast send of small typed messages to any nearby Sesame bot.
//   - A receive queue: the radio callback only copies packets into a
//     FreeRTOS queue; they are handled later from loop() via espnowService().
//   - A heartbeat every ESPNOW_HEARTBEAT_MS so bots discover each other,
//     and a small peer table (MAC, name, RSSI, last seen).
//   - espnowOnMessage: optional hook for future behaviours.
//
// CHANNEL: ESP-NOW only works between radios on the same Wi-Fi channel.
// The robot's soft AP is pinned to ESPNOW_CHANNEL so both bots match.
// If a bot joins a router (station mode), the radio moves to the router's
// channel and the link to the other bot breaks unless it is on that
// channel too. For two-bot use, keep ENABLE_NETWORK_MODE false and don't
// connect to Wi-Fi from the web UI.
//
// RSSI: reported with ESP32 Arduino core 3.x. On core 2.x the receive
// callback has no RSSI, so it is reported as 0 (unknown).
//
// Messages are broadcast and NOT encrypted. Any nearby ESP-NOW device can
// read them or send look-alike packets. Fine for experiments; revisit
// before relying on it.
// ======================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#define ESPNOW_CHANNEL        1       // soft AP + ESP-NOW channel, same on every bot
#define ESPNOW_MAGIC          0x5E5A  // marks Sesame packets, others are ignored
#define ESPNOW_PROTO_VERSION  1
#define ESPNOW_MAX_PAYLOAD    200     // ESP-NOW limit is 250 bytes per packet
#define ESPNOW_RX_QUEUE_LEN   10
#define ESPNOW_HEARTBEAT_MS   2000
#define ESPNOW_MAX_PEERS      4
#define ESPNOW_PEER_TIMEOUT_MS 10000  // peer counts as gone after this

enum EspNowMsgType : uint8_t {
  ESPNOW_MSG_HEARTBEAT = 1,   // payload: sender's name (AP SSID)
  ESPNOW_MSG_TEXT      = 2,   // payload: free text (debug / testing)
  ESPNOW_MSG_PING      = 3,   // payload: PingPayload (tester)
  ESPNOW_MSG_PONG      = 4,   // payload: echo of the PING payload (tester)
  ESPNOW_MSG_POSE      = 5,   // payload: 4 servo angles (tester)
  ESPNOW_MSG_POSE8     = 6,   // payload: 8 servo angles (sequence tester)
  // Future behaviours (fall alert, follow-the-leader, commands) go here.
};

struct __attribute__((packed)) EspNowHeader {
  uint16_t magic;
  uint8_t  version;
  uint8_t  type;
  uint16_t seq;
  uint8_t  len;               // payload length
};

struct __attribute__((packed)) EspNowPacket {
  EspNowHeader hdr;
  uint8_t payload[ESPNOW_MAX_PAYLOAD];
};

struct EspNowRxMsg {
  uint8_t mac[6];
  int8_t rssi;                // dBm, 0 = unknown
  EspNowHeader hdr;
  uint8_t payload[ESPNOW_MAX_PAYLOAD + 1];  // +1 so text can be NUL-terminated
};

struct EspNowPeer {
  bool used;
  uint8_t mac[6];
  char name[33];
  int8_t rssi;
  unsigned long lastSeenMs;
  uint32_t msgCount;
};

#ifdef ESPNOW_USE_STA
#define ESPNOW_OWN_MAC WiFi.macAddress()
#else
#define ESPNOW_OWN_MAC WiFi.softAPmacAddress()
#endif

static const uint8_t ESPNOW_BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

bool espnowReady = false;
esp_err_t espnowLastErr = ESP_OK;   // result of the last esp_now_send, for the gateway's error reports
String espnowName = "";
QueueHandle_t espnowRxQueue = nullptr;
uint16_t espnowTxSeq = 0;
volatile uint32_t espnowRawRx = 0;   // every packet the radio delivered, before filtering
uint32_t espnowTxCount = 0, espnowTxFail = 0, espnowRxCount = 0, espnowRxDropped = 0;
unsigned long espnowLastHeartbeatMs = 0;
EspNowPeer espnowPeers[ESPNOW_MAX_PEERS] = {};

// Optional hook for future behaviours. Called from loop() context
// (never from the radio callback) for every valid received message.
void (*espnowOnMessage)(const EspNowRxMsg& msg) = nullptr;

String espnowMacToString(const uint8_t* mac) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

// --- Radio callback: runs in the Wi-Fi task. Keep it short: validate, copy, queue.
static void espnowQueueRx(const uint8_t* mac, int8_t rssi, const uint8_t* data, int len) {
  espnowRawRx++;
  if (espnowRxQueue == nullptr || len < (int)sizeof(EspNowHeader)) return;
  EspNowHeader hdr;
  memcpy(&hdr, data, sizeof(hdr));
  if (hdr.magic != ESPNOW_MAGIC || hdr.version != ESPNOW_PROTO_VERSION) return;
  if (hdr.len > ESPNOW_MAX_PAYLOAD || (int)(sizeof(hdr) + hdr.len) > len) return;

  EspNowRxMsg msg;
  memcpy(msg.mac, mac, 6);
  msg.rssi = rssi;
  msg.hdr = hdr;
  memcpy(msg.payload, data + sizeof(hdr), hdr.len);
  msg.payload[hdr.len] = 0;
  if (xQueueSend(espnowRxQueue, &msg, 0) != pdTRUE) espnowRxDropped++;
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void espnowRecvCb(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  int8_t rssi = (info->rx_ctrl != nullptr) ? (int8_t)info->rx_ctrl->rssi : 0;
  espnowQueueRx(info->src_addr, rssi, data, len);
}
#else
static void espnowRecvCb(const uint8_t* mac, const uint8_t* data, int len) {
  espnowQueueRx(mac, 0, data, len);
}
#endif

// Send one message to all nearby bots. Returns true if handed to the radio
// (broadcast has no delivery confirmation).
bool espnowSend(uint8_t type, const uint8_t* payload, uint8_t len) {
  if (!espnowReady || len > ESPNOW_MAX_PAYLOAD) return false;
  EspNowPacket pkt;
  pkt.hdr.magic = ESPNOW_MAGIC;
  pkt.hdr.version = ESPNOW_PROTO_VERSION;
  pkt.hdr.type = type;
  pkt.hdr.seq = espnowTxSeq++;
  pkt.hdr.len = len;
  if (len > 0) memcpy(pkt.payload, payload, len);
  esp_err_t err = esp_now_send(ESPNOW_BROADCAST_MAC, (const uint8_t*)&pkt, sizeof(EspNowHeader) + len);
  espnowLastErr = err;
  if (err == ESP_OK) { espnowTxCount++; return true; }
  espnowTxFail++;
  return false;
}

bool espnowSendText(const String& text) {
  uint8_t len = (uint8_t)min((size_t)ESPNOW_MAX_PAYLOAD, (size_t)text.length());
  return espnowSend(ESPNOW_MSG_TEXT, (const uint8_t*)text.c_str(), len);
}

// Call after the soft AP is up (WiFi.softAP on ESPNOW_CHANNEL).
bool espnowBegin(const String& name) {
  espnowName = name;
  if (espnowRxQueue == nullptr) {
    espnowRxQueue = xQueueCreate(ESPNOW_RX_QUEUE_LEN, sizeof(EspNowRxMsg));
    if (espnowRxQueue == nullptr) {
      Serial.println("ESP-NOW: queue allocation failed");
      return false;
    }
  }
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW: init failed");
    return false;
  }
  esp_now_register_recv_cb(espnowRecvCb);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, ESPNOW_BROADCAST_MAC, 6);
  peer.channel = 0;             // 0 = whatever channel the radio is on now
#ifdef ESPNOW_USE_STA
  peer.ifidx = WIFI_IF_STA;
#else
  peer.ifidx = WIFI_IF_AP;      // the robot always runs its soft AP
#endif
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("ESP-NOW: adding broadcast peer failed");
    esp_now_deinit();
    return false;
  }

  espnowReady = true;
  Serial.println("ESP-NOW: ready on channel " + String(ESPNOW_CHANNEL) +
                 ", MAC " + ESPNOW_OWN_MAC);
  return true;
}

static void espnowUpdatePeer(const EspNowRxMsg& msg) {
  int slot = -1, freeSlot = -1, oldest = 0;
  for (int i = 0; i < ESPNOW_MAX_PEERS; i++) {
    if (espnowPeers[i].used && memcmp(espnowPeers[i].mac, msg.mac, 6) == 0) { slot = i; break; }
    if (!espnowPeers[i].used && freeSlot < 0) freeSlot = i;
    if (espnowPeers[i].lastSeenMs < espnowPeers[oldest].lastSeenMs) oldest = i;
  }
  bool isNew = (slot < 0);
  if (isNew) {
    slot = (freeSlot >= 0) ? freeSlot : oldest;   // table full: replace the stalest
    espnowPeers[slot] = {};
    espnowPeers[slot].used = true;
    memcpy(espnowPeers[slot].mac, msg.mac, 6);
  }
  EspNowPeer& p = espnowPeers[slot];
  p.rssi = msg.rssi;
  p.lastSeenMs = millis();
  p.msgCount++;
  if (msg.hdr.type == ESPNOW_MSG_HEARTBEAT) {
    size_t n = min((size_t)msg.hdr.len, sizeof(p.name) - 1);
    memcpy(p.name, msg.payload, n);
    p.name[n] = 0;
  }
  if (isNew) {
    Serial.println("ESP-NOW: new peer " + espnowMacToString(msg.mac) +
                   (p.name[0] ? (" (" + String(p.name) + ")") : String("")));
  }
}

// Call often from loop() and long waits. Drains received messages and
// sends the periodic heartbeat.
void espnowService() {
  if (!espnowReady) return;

  EspNowRxMsg msg;
  while (xQueueReceive(espnowRxQueue, &msg, 0) == pdTRUE) {
    espnowRxCount++;
    espnowUpdatePeer(msg);
    if (msg.hdr.type == ESPNOW_MSG_TEXT) {
      Serial.printf("ESP-NOW text from %s (RSSI %d, seq %u): %s\n",
                    espnowMacToString(msg.mac).c_str(), msg.rssi, msg.hdr.seq, (const char*)msg.payload);
    }
    if (espnowOnMessage) espnowOnMessage(msg);
  }

  unsigned long now = millis();
  if (now - espnowLastHeartbeatMs >= ESPNOW_HEARTBEAT_MS) {
    espnowLastHeartbeatMs = now;
    uint8_t len = (uint8_t)min((size_t)32, (size_t)espnowName.length());
    espnowSend(ESPNOW_MSG_HEARTBEAT, (const uint8_t*)espnowName.c_str(), len);
  }
}

void espnowPrintInfo() {
  uint8_t primary = 0;
  wifi_second_chan_t second;
  esp_wifi_get_channel(&primary, &second);
  Serial.println("ESP-NOW: " + String(espnowReady ? "ready" : "NOT ready") +
                 " | name " + espnowName + " | MAC " + ESPNOW_OWN_MAC +
                 " | radio channel " + String(primary) + " (expected " + String(ESPNOW_CHANNEL) + ")");
  Serial.printf("  tx %lu (fail %lu) | rx %lu (dropped %lu) | raw radio rx %lu\n",
                (unsigned long)espnowTxCount, (unsigned long)espnowTxFail,
                (unsigned long)espnowRxCount, (unsigned long)espnowRxDropped,
                (unsigned long)espnowRawRx);
  unsigned long now = millis();
  bool any = false;
  for (int i = 0; i < ESPNOW_MAX_PEERS; i++) {
    const EspNowPeer& p = espnowPeers[i];
    if (!p.used) continue;
    any = true;
    unsigned long age = now - p.lastSeenMs;
    Serial.printf("  peer %s %-12s RSSI %d | seen %lus ago%s | msgs %lu\n",
                  espnowMacToString(p.mac).c_str(), p.name, p.rssi, age / 1000,
                  age > ESPNOW_PEER_TIMEOUT_MS ? " (gone)" : "", (unsigned long)p.msgCount);
  }
  if (!any) Serial.println("  no peers seen yet");
}
