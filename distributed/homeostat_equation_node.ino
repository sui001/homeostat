/*
 * PROJECT: Distributed Homeostat -- Room Installation
 * DEVICE:  Equation Node (one per corner, x[DEVICE_ID - 1])
 * 
 * One unit of Ashby's homeostat. Owns one state variable x[i] and one
 * row of the coupling matrix k[i][*]. Receives peer x values via ESP-NOW,
 * computes dx/dt, updates x[i], broadcasts back into the room.
 * 
 * When |x[i]| exceeds BOUND, this node's S2 fires: randomises its k row
 * and lets the system recover through its new dynamics -- no reset to zero.
 * The system heals by changing the relations among its parts, not by
 * erasing the disturbance. (Ashby / T. Zafiropulos)
 * 
 * THREE VISIBLE EVENTS:
 *   Yellow flash  = perturbation received
 *   Red/Blue      = system stress (x moving away from zero)
 *   White flash   = S2 reorganisation event
 * 
 * CONFIGURATION:
 *   *** ONLY CHANGE DEVICE_ID BETWEEN FLASHES ***
 * 
 * WIRING:
 *   GPIO 4  -> WS2812 LED strip data
 *   GPIO 0  -> Boot button (perturbation input)
 */

#include <esp_now.h>
#include <WiFi.h>
#include <FastLED.h>

// ---- Identity -- CHANGE PER DEVICE ----------------------------------------

const int DEVICE_ID = 1;  // *** CHANGE THIS: 1, 2, 3, or 4 ***

// ---- Config ----------------------------------------------------------------

const int   NUM_NODES             = 4;
const int   NODE_INDEX            = DEVICE_ID - 1;  // DO NOT CHANGE

const int   LED_PIN               = 4;   // DO NOT CHANGE
const int   NUM_LEDS              = 16;  // DO NOT CHANGE
const int   BUTTON_PIN            = 0;   // DO NOT CHANGE

const float BOUND                 = 1.0f;
const float DT                    = 0.06f;
const float K_MAX                 = 0.5f;
const float DIAGONAL_DAMPING      = 0.08f;  // max self-damping; actual is randomised per reconfig
const float PERTURB_MIN           = 5.0f;
const float PERTURB_MAX           = 10.0f;
const float DISPLAY_SCALE         = 1.0f;

const unsigned long BUTTON_DEBOUNCE_MS       = 400;
const unsigned long MIN_RECONFIG_INTERVAL_MS = 200;
const unsigned long PERTURB_FLASH_MS         = 250;
const unsigned long RECONFIG_FLASH_MS        = 300;
const unsigned long BOOT_DELAY_MS            = 600;

// Broadcast address
uint8_t broadcastAddress[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// ---- Message types ---------------------------------------------------------

typedef struct __attribute__((packed)) {
  uint8_t  msg_type;    // 2 = state update
  uint8_t  source_id;   // 1-4
  float    x_value;
} StateMessage;

typedef struct __attribute__((packed)) {
  uint8_t  msg_type;    // 1 = perturbation
  uint8_t  source_id;   // 0 = sensor node
  float    strength;
} KnockMessage;

// ---- Simulation state ------------------------------------------------------

float x[NUM_NODES]            = {0};
float k[NUM_NODES]            = {0};
float x_own                   = 0.0f;

unsigned long lastReconfig     = 0;
unsigned long lastButtonPress  = 0;
unsigned long perturbFlashUntil = 0;
unsigned long reconfigFlashUntil = 0;

bool inPerturbFlash  = false;
bool inReconfigFlash = false;

// ---- LED -------------------------------------------------------------------

CRGB leds[NUM_LEDS];

// ---- ESP-NOW callbacks -----------------------------------------------------

void onReceive(const esp_now_recv_info *info, const uint8_t *data, int len) {
  if (len < 1) return;
  uint8_t msg_type = data[0];

  if (msg_type == 1 && len == sizeof(KnockMessage)) {
    if (NODE_INDEX == 0) {
      KnockMessage *msg = (KnockMessage *)data;
      x_own += msg->strength;
      x[NODE_INDEX] = x_own;
      triggerPerturbFlash();
      Serial.print("[node] Perturbation received: ");
      Serial.println(msg->strength);
    }
  } else if (msg_type == 2 && len == sizeof(StateMessage)) {
    StateMessage *msg = (StateMessage *)data;
    int idx = msg->source_id - 1;
    if (idx >= 0 && idx < NUM_NODES && idx != NODE_INDEX) {
      x[idx] = msg->x_value;
    }
  }
}

void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // silent
}

// ---- S2: uniselector -------------------------------------------------------

// Randomises k row with variable diagonal damping -- each organisation
// genuinely different. System recovers through new dynamics, not reset.
void fireUniselector() {
  unsigned long now = millis();
  if (now - lastReconfig < MIN_RECONFIG_INTERVAL_MS) return;

  Serial.println("[S2] Reorganising");

  for (int j = 0; j < NUM_NODES; j++) {
    k[j] = randomFloat(-K_MAX, K_MAX);
  }
  // Diagonal: randomised within damping range -- no two organisations identical
  k[NODE_INDEX] = randomFloat(-DIAGONAL_DAMPING, -DIAGONAL_DAMPING * 0.1f);

  // NO x_own reset -- system recovers through its new dynamics
  lastReconfig = now;
  triggerReconfigFlash();

  Serial.print("[S2] New k: ");
  for (int j = 0; j < NUM_NODES; j++) {
    Serial.print(k[j]); Serial.print(" ");
  }
  Serial.println();
}

// ---- Button ----------------------------------------------------------------

void checkButton() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    unsigned long now = millis();
    if (now - lastButtonPress > BUTTON_DEBOUNCE_MS) {
      float strength = randomFloat(PERTURB_MIN, PERTURB_MAX);
      x_own += strength;
      x[NODE_INDEX] = x_own;
      triggerPerturbFlash();
      broadcastState();  // immediate broadcast before S2 can fire
      lastButtonPress = now;
      Serial.print("[node] Perturbation: ");
      Serial.println(strength);
    }
  }
}

// ---- Integration -----------------------------------------------------------

void stepSimulation() {
  float dx = 0.0f;
  for (int j = 0; j < NUM_NODES; j++) {
    dx += k[j] * x[j];
  }
  x_own += dx * DT;
  // Clamp to prevent runaway divergence
  x_own = constrain(x_own, -BOUND * 3.0f, BOUND * 3.0f);
  x[NODE_INDEX] = x_own;

  if (abs(x_own) > BOUND) fireUniselector();
}

// ---- Broadcast -------------------------------------------------------------

void broadcastState() {
  StateMessage msg;
  msg.msg_type  = 2;
  msg.source_id = DEVICE_ID;
  msg.x_value   = x_own;
  esp_now_send(broadcastAddress, (uint8_t *)&msg, sizeof(msg));
}

// ---- LED -------------------------------------------------------------------

void triggerPerturbFlash() {
  perturbFlashUntil = millis() + PERTURB_FLASH_MS;
  inPerturbFlash = true;
}

void triggerReconfigFlash() {
  reconfigFlashUntil = millis() + RECONFIG_FLASH_MS;
  inReconfigFlash = true;
}

void renderState() {
  unsigned long now = millis();

  // Perturbation: yellow
  if (inPerturbFlash && now < perturbFlashUntil) {
    fill_solid(leds, NUM_LEDS, CHSV(48, 255, 255));
    FastLED.show();
    return;
  }
  inPerturbFlash = false;

  // Reorganisation: white
  if (inReconfigFlash && now < reconfigFlashUntil) {
    fill_solid(leds, NUM_LEDS, CRGB::White);
    FastLED.show();
    return;
  }
  inReconfigFlash = false;

  // Normal: green fading to red/blue
  float magnitude = constrain(fabsf(x_own) * DISPLAY_SCALE, 0.0f, 1.0f);
  int litCount = NUM_LEDS - (int)(magnitude * (NUM_LEDS - 1));
  litCount = constrain(litCount, 1, NUM_LEDS);

  // Dead zone: green if near zero
  uint8_t hue = (x_own > 0.08f)  ? 0   // red: positive
              : (x_own < -0.08f) ? 160  // blue: negative
              : 96;                      // green: near zero

  for (int i = 0; i < NUM_LEDS; i++) {
    if (i < litCount) {
      leds[i] = CHSV(hue, 220, 200);
    } else {
      leds[i] = CRGB::Black;
    }
  }
  FastLED.show();
}

// ---- Plotter ---------------------------------------------------------------

void printPlotter() {
  for (int i = 0; i < NUM_NODES; i++) {
    Serial.print(x[i]);
    if (i < NUM_NODES - 1) Serial.print("\t");
  }
  Serial.println();
}

// ---- Helpers ---------------------------------------------------------------

float randomFloat(float low, float high) {
  return low + (high - low) * ((float)random(0, 10001) / 10000.0f);
}

// ---- Setup -----------------------------------------------------------------

void initK() {
  for (int j = 0; j < NUM_NODES; j++) {
    k[j] = randomFloat(-K_MAX, K_MAX);
  }
  k[NODE_INDEX] = randomFloat(-DIAGONAL_DAMPING, -DIAGONAL_DAMPING * 0.1f);
}

void setup() {
  delay(BOOT_DELAY_MS * DEVICE_ID);

  Serial.begin(115200);
  Serial.print("[node] Homeostat node ");
  Serial.print(DEVICE_ID);
  Serial.println(" booting...");

  randomSeed(esp_random());
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(180);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[node] ESP-NOW init failed. Halting.");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onReceive);
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  initK();

  Serial.print("[node] x[");
  Serial.print(NODE_INDEX);
  Serial.println("] ready. Listening to the room.");
}

// ---- Loop ------------------------------------------------------------------

void loop() {
  checkButton();
  stepSimulation();
  broadcastState();
  renderState();
  printPlotter();
  delay(60);
}

/*
 * FLASH INSTRUCTIONS:
 *   Board:    ESP32S3 Dev Module
 *   USB Mode: Hardware CDC and JTAG
 *   Libraries: FastLED
 * 
 *   Change DEVICE_ID only:
 *     Corner 1 -> DEVICE_ID = 1  (receives sensor perturbation)
 *     Corner 2 -> DEVICE_ID = 2
 *     Corner 3 -> DEVICE_ID = 3
 *     Corner 4 -> DEVICE_ID = 4
 */
