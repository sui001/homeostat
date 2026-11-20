/*
 * PROJECT: Distributed Homeostat -- Room Installation
 * DEVICE:  Sensor Node (PIR motion input)
 * 
 * Watches the room. Motion detected = perturbation broadcast to all
 * equation nodes via ESP-NOW. Node 1 absorbs it; propagates through
 * coupling to the other three units.
 * 
 * This node is the environment. Not part of the simulation.
 * Ashby's hand pushing the needle.
 * 
 * CONFIGURATION:
 *   DEVICE_ID: 0 -- DO NOT CHANGE
 *   PIR_PIN:   GPIO 2 -- PIR signal out
 * 
 * WIRING:
 *   GPIO 2  -> PIR signal out
 *   PIR VCC -> 3.3V or 5V (check your module)
 *   PIR GND -> GND
 */

#include <esp_now.h>
#include <WiFi.h>

// ---- Config ----------------------------------------------------------------

const int    DEVICE_ID            = 0;  // DO NOT CHANGE
const int    PIR_PIN              = 2;

const float  PERTURB_MIN          = 5.0f;
const float  PERTURB_MAX          = 10.0f;

const unsigned long PERTURB_COOLDOWN_MS = 1500;
const unsigned long BOOT_DELAY_MS       = 0;

// Broadcast address
uint8_t broadcastAddress[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// ---- Message types ---------------------------------------------------------

typedef struct __attribute__((packed)) {
  uint8_t  msg_type;    // 1 = perturbation
  uint8_t  source_id;   // 0 = sensor node
  float    strength;
} KnockMessage;

// ---- State -----------------------------------------------------------------

unsigned long lastPerturbTime = 0;
bool pirLastState = LOW;

// ---- ESP-NOW callbacks -----------------------------------------------------

void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // silent
}

// ---- Setup -----------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  Serial.println("[sensor] Homeostat sensor node booting...");

  pinMode(PIR_PIN, INPUT);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[sensor] ESP-NOW init failed. Halting.");
    while (true) delay(1000);
  }

  esp_now_register_send_cb(onSent);

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

  // Rising edge = motion detected
  if (pirState == HIGH && pirLastState == LOW) {
    if (now - lastPerturbTime > PERTURB_COOLDOWN_MS) {
      broadcastPerturbation();
      lastPerturbTime = now;
    }
  }

  // Debug: print PIR state changes
  if (pirState != pirLastState) {
    Serial.print("[pir] ");
    Serial.println(pirState == HIGH ? "HIGH -- motion detected" : "LOW -- clear");
  }

  pirLastState = pirState;
  delay(20);
}

// ---- Perturbation ----------------------------------------------------------

void broadcastPerturbation() {
  float strength = PERTURB_MIN +
    (float)random(1000) / 1000.0f * (PERTURB_MAX - PERTURB_MIN);

  KnockMessage msg;
  msg.msg_type  = 1;
  msg.source_id = DEVICE_ID;
  msg.strength  = strength;

  esp_now_send(broadcastAddress, (uint8_t *)&msg, sizeof(msg));

  Serial.print("[sensor] Perturbation broadcast -- strength: ");
  Serial.println(strength);
}

/*
 * FLASH INSTRUCTIONS:
 *   Board:    ESP32S3 Dev Module (or ESP32-S3 Supermini)
 *   USB Mode: Hardware CDC and JTAG
 *   No FastLED needed
 *   DEVICE_ID always 0 -- never change
 */
