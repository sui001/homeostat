/*
 * PROJECT: Distributed Homeostat -- Room Installation
 * DEVICE:  Sensor Node (motion input)
 * 
 * This node watches the room. When it detects motion, it broadcasts
 * a knock event to all four equation nodes via ESP-NOW. The knock
 * carries a random strength value -- a disturbance from the environment
 * entering the system, as Ashby intended: perturbation from outside
 * the viable range, demanding adaptation.
 * 
 * CONFIGURATION:
 *   DEVICE_ID: 0 (sensor node -- do not change)
 *   PIR_PIN: GPIO connected to PIR signal out
 * 
 * WIRING:
 *   GPIO 2  -> PIR sensor signal out
 *   PIR VCC -> 3.3V or 5V (check your module)
 *   PIR GND -> GND
 */

#include <esp_now.h>
#include <WiFi.h>

// ---- Config ----------------------------------------------------------------

const int    DEVICE_ID            = 0;
const int    PIR_PIN              = 2;

const float  KNOCK_STRENGTH_MIN   = 2.0f;
const float  KNOCK_STRENGTH_MAX   = 5.0f;

const unsigned long KNOCK_COOLDOWN_MS = 1500;  // min ms between knocks
const unsigned long BOOT_DELAY_MS    = 500;    // stagger boot

// Broadcast address -- reaches all ESP-NOW peers on the network
uint8_t broadcastAddress[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// ---- Message types ---------------------------------------------------------

typedef struct {
  uint8_t  msg_type;      // 1 = knock
  uint8_t  source_id;     // always 0 for sensor node
  float    strength;      // knock magnitude
} KnockMessage;

// ---- State -----------------------------------------------------------------

unsigned long lastKnockTime = 0;
bool pirLastState = LOW;

// ---- ESP-NOW callbacks -----------------------------------------------------

void onSent(const uint8_t *mac, esp_now_send_status_t status) {
  // silent -- we don't need confirmation for broadcast knocks
}

// ---- Setup -----------------------------------------------------------------

void setup() {
  delay(BOOT_DELAY_MS * DEVICE_ID);  // stagger boot (sensor = 0, instant)

  Serial.begin(115200);
  Serial.println("[sensor] Homeostat sensor node booting...");

  pinMode(PIR_PIN, INPUT);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("[sensor] ESP-NOW init failed. Halting.");
    while (true) delay(1000);
  }

  esp_now_register_send_cb(onSent);

  // Register broadcast peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  Serial.println("[sensor] Ready. Watching the room.");
}

// ---- Main loop -------------------------------------------------------------

void loop() {
  bool pirState = digitalRead(PIR_PIN);
  unsigned long now = millis();

  // Rising edge on PIR = motion detected
  if (pirState == HIGH && pirLastState == LOW) {
    if (now - lastKnockTime > KNOCK_COOLDOWN_MS) {
      broadcastKnock();
      lastKnockTime = now;
    }
  }

  pirLastState = pirState;
  delay(20);
}

// ---- Knock -----------------------------------------------------------------

void broadcastKnock() {
  float strength = KNOCK_STRENGTH_MIN +
    (float)random(1000) / 1000.0f * (KNOCK_STRENGTH_MAX - KNOCK_STRENGTH_MIN);

  KnockMessage msg;
  msg.msg_type  = 1;
  msg.source_id = DEVICE_ID;
  msg.strength  = strength;

  esp_now_send(broadcastAddress, (uint8_t *)&msg, sizeof(msg));

  Serial.print("[sensor] Knock broadcast -- strength: ");
  Serial.println(strength);
}

/*
 * FLASH INSTRUCTIONS:
 *   Board:    ESP32S3 Dev Module (or ESP32-S3 Supermini)
 *   USB Mode: Hardware CDC and JTAG
 *   This file only -- sensor node does not need FastLED
 *   No DEVICE_ID change needed (always 0)
 */
