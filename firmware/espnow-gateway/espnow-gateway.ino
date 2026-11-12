/*
 * ESP-NOW <-> USB-serial gateway for the Raspberry Pi ground station.
 * ESP32-S3 SuperMini, no servos. Plug it into the Pi over USB; it relays between the
 * Pi (newline-terminated text lines at 115200) and the Sesame bots (ESP-NOW, channel 1).
 *
 * Pi -> gateway (one command per line):
 *   cmd <id|*> <text>      send a bot command (the same text you would type on the bot's serial,
 *                          e.g. "cmd * seq wave", "cmd A1B2 leader"). <id> is the board ID from
 *                          the bot's boot message, '*' = every bot.
 *   pose a0 a1 ... a7      broadcast one 8-servo pose (0-180 each), same packet the leader streams
 *   power <dBm>            set TX power (2-20, default 14, see the TX power notes)
 *   status                 print gateway counters
 *   ping                   gateway answers "@PONG"
 *
 * Gateway -> Pi (every line starts with '@', so Pi code can ignore boot noise):
 *   @READY <mac> ch<n> tx<dBm>
 *   @OK / @ERR <reason>
 *   @HB  <mac> <rssi> <name>        bot heartbeat
 *   @RX  <mac> <rssi> <text>        text from a bot (telemetry "TLM:..." and replies "ACK:...")
 *   @STAT tx=<n> fail=<n> rx=<n> drop=<n>
 *   @PONG
 *
 * Libraries: none beyond the ESP32 Arduino core. ESP-NOW is broadcast and not encrypted
 * (same as the bots), so anyone nearby on channel 1 can read or send look-alike packets.
 */

#define ESPNOW_USE_STA

#include <WiFi.h>
#include "espnow-comms.h"

int txPowerDbm = 14;   // 8 fixed ~97% loss at 30 cm on these boards; 14 worked at 1.5-3 m
String lineBuf;

// Called from loop() context for every valid Sesame packet.
void onMessage(const EspNowRxMsg& m) {
  String mac = espnowMacToString(m.mac);
  if (m.hdr.type == ESPNOW_MSG_HEARTBEAT) {
    Serial.printf("@HB %s %d %s\n", mac.c_str(), m.rssi, (const char*)m.payload);
  } else if (m.hdr.type == ESPNOW_MSG_TEXT) {
    Serial.printf("@RX %s %d %s\n", mac.c_str(), m.rssi, (const char*)m.payload);
  }
  // POSE/POSE8 from a leader bot are not forwarded: the Pi doesn't need the 50 Hz stream.
}

void handleLine(String line) {
  line.trim();
  if (!line.length()) return;

  if (line == "ping") { Serial.println("@PONG"); return; }

  if (line == "status") {
    Serial.printf("@STAT tx=%lu fail=%lu rx=%lu drop=%lu\n",
                  (unsigned long)espnowTxCount, (unsigned long)espnowTxFail,
                  (unsigned long)espnowRxCount, (unsigned long)espnowRxDropped);
    return;
  }

  if (line.startsWith("power ")) {
    txPowerDbm = constrain(line.substring(6).toInt(), 2, 20);
    esp_wifi_set_max_tx_power(txPowerDbm * 4);
    Serial.println("@OK");
    return;
  }

  if (line.startsWith("cmd ")) {
    // cmd <id|*> <text> -> TEXT packet "CMD:<id>:<text>"
    String rest = line.substring(4);
    rest.trim();
    int sp = rest.indexOf(' ');
    if (sp <= 0) { Serial.println("@ERR usage: cmd <id|*> <text>"); return; }
    String id = rest.substring(0, sp);
    String text = rest.substring(sp + 1);
    text.trim();
    String payload = "CMD:" + id + ":" + text;
    if (payload.length() > ESPNOW_MAX_PAYLOAD) { Serial.println("@ERR too long"); return; }
    Serial.println(espnowSendText(payload) ? "@OK" : "@ERR send failed");
    return;
  }

  if (line.startsWith("pose ")) {
    uint8_t angles[8];
    int n = 0;
    String rest = line.substring(5);
    rest.trim();
    while (rest.length() && n < 8) {
      int sp = rest.indexOf(' ');
      String tok = (sp < 0) ? rest : rest.substring(0, sp);
      angles[n++] = (uint8_t)constrain(tok.toInt(), 0, 180);
      rest = (sp < 0) ? "" : rest.substring(sp + 1);
      rest.trim();
    }
    if (n != 8) { Serial.println("@ERR pose needs 8 angles"); return; }
    Serial.println(espnowSend(ESPNOW_MSG_POSE8, angles, 8) ? "@OK" : "@ERR send failed");
    return;
  }

  Serial.println("@ERR unknown command");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_max_tx_power(txPowerDbm * 4);
  delay(100);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (!espnowBegin("gateway")) {
    Serial.println("@ERR ESP-NOW failed to start");
    return;
  }
  espnowOnMessage = onMessage;
  Serial.printf("@READY %s ch%d tx%d\n", WiFi.macAddress().c_str(), ESPNOW_CHANNEL, txPowerDbm);
}

void loop() {
  espnowService();
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineBuf.length()) { String l = lineBuf; lineBuf = ""; handleLine(l); }
    } else if (lineBuf.length() < 300) {
      lineBuf += c;
    }
  }
}
