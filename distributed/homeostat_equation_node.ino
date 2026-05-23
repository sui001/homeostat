/*
 * PROJECT: Distributed Homeostat -- Room Installation
 * DEVICE:  Equation Node (one per corner, x[DEVICE_ID - 1])
 * 
 * This node is one unit of Ashby's homeostat, physically placed in
 * a corner of the room. It owns one state variable x[i] and one row
 * of the coupling matrix k[i][*]. It listens for the other three
 * nodes' x values via ESP-NOW, computes its own dx/dt, updates x[i],
 * and broadcasts its new value back into the room.
 * 
 * When |x[i]| exceeds BOUND, this node's S2 fires: it randomises its
 * own k row and resets x[i] to zero -- hunting for viability
 * independently. The room finds equilibrium, or it doesn't.
 * 
 * A knock from the sensor node hits x[0] only (node 1). The
 * disturbance propagates through coupling -- the room responds.
 * 
 * CONFIGURATION:
 *   DEVICE_ID: Set to 1, 2, 3, or 4 before flashing each node.
 *              Node 1 = corner 1 = x[0], etc.
 * 
 * WIRING:
 *   GPIO 4  -> WS2812 LED strip data
 *   GPIO 9  -> Button (other side to GND, internal pullup)
 *   LED strip VCC -> external 5V
 *   LED strip GND -> GND (shared with ESP32)
 */

#include <esp_now.h>
#include <WiFi.h>
#include <FastLED.h>

// ---- Identity -- CHANGE PER DEVICE ----------------------------------------

const int DEVICE_ID = 1;  // 1, 2, 3, or 4

// ---- Config ----------------------------------------------------------------

const int   NUM_NODES             = 4;
const int   NODE_INDEX            = DEVICE_ID - 1;  // 0-based index into x[]

const int   LED_PIN               = 48;
const int   NUM_LEDS              = 1;
const int   BUTTON_PIN            = 0;

const float BOUND                 = 1.0f;
const float DT                    = 0.06f;
const float K_MAX                 = 0.35f;
const float DIAGONAL_DAMPING      = 0.08f;
const float KNOCK_STRENGTH_MIN    = 2.0f;
const float KNOCK_STRENGTH_MAX    = 5.0f;
const float DISPLAY_SCALE         = 0.3f;

const unsigned long BUTTON_DEBOUNCE_MS       = 400;
const unsigned long MIN_RECONFIG_INTERVAL_MS = 200;
const unsigned long KNOCK_FLASH_MS           = 250;
const unsigned long PEER_TIMEOUT_MS          = 500;   // treat peer as stale after this
const unsigned long BOOT_DELAY_MS            = 600;   // stagger: node 1=600ms, 2=1200ms etc.

// Broadcast address
uint8_t broadcastAddress[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// ---- Message types ---------------------------------------------------------

// State broadcast: each node sends its current x value
typedef struct {
  uint8_t  msg_type;    // 2 = state update
  uint8_t  source_id;   // 1-4
  float    x_value;     // current x[source_id - 1]
} StateMessage;

// Knock from sensor node
typedef struct {
  uint8_t  msg_type;    // 1 = knock
  uint8_t  source_id;   // 0 = sensor
  float    strength;
} KnockMessage;

// ---- Simulation state ------------------------------------------------------

float x[NUM_NODES]          = {0};   // all four x values (own + received from peers)
float k[NUM_NODES]          = {0};   // this node's row of the coupling matrix
unsigned long lastReceived[NUM_NODES] = {0};  // timestamp of last update per peer

float x_own = 0.0f;  // canonical local value (also stored in x[NODE_INDEX])

unsigned long lastReconfig     = 0;
unsigned long lastButtonPress  = 0;
unsigned long knockFlashUntil  = 0;

bool inKnockFlash = false;

// ---- LED state -------------------------------------------------------------

CRGB leds[NUM_LEDS];

// ---- ESP-NOW callbacks -----------------------------------------------------

void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
  if (len < 1) return;
  uint8_t msg_type = data[0];

  if (msg_type == 1 && len == sizeof(KnockMessage)) {
    // Knock from sensor node -- only node 1 (x[0]) absorbs it
    if (NODE_INDEX == 0) {
      KnockMessage *msg = (KnockMessage *)data;
      x_own += msg->strength;
      x[NODE_INDEX] = x_own;
      triggerKnockFlash();
      Serial.print("[node] Knock received, strength: ");
      Serial.println(msg->strength);
    }

  } else if (msg_type == 2 && len == sizeof(StateMessage)) {
    StateMessage *msg = (StateMessage *)data;
    int idx = msg->source_id - 1;
    if (idx >= 0 && idx < NUM_NODES && idx != NODE_INDEX) {
      x[idx] = msg->x_value;
      lastReceived[idx] = millis();
    }
  }
}

void onSent(const uint8_t *mac, esp_now_send_status_t status) {
  // silent
}

// ---- S2: uniselector -------------------------------------------------------

// This node's S2 -- fires when x_own leaves the viable range.
// Randomises this node's k row and resets x_own.
// Each corner of the room makes this decision for itself.
void fireUniselector() {
  unsigned long now = millis();
  if (now - lastReconfig < MIN_RECONFIG_INTERVAL_MS) return;

  Serial.println("[S2] Out of viable range -- reconfiguring k row");

  for (int j = 0; j < NUM_NODES; j++) {
    k[j] = ((float)random(2000) / 1000.0f - 1.0f) * K_MAX;
  }
  k[NODE_INDEX] = -DIAGONAL_DAMPING;  // self-damping preserved

  x_own = 0.0f;
  x[NODE_INDEX] = 0.0f;
  lastReconfig = now;

  Serial.print("[S2] New k: ");
  for (int j = 0; j < NUM_NODES; j++) {
    Serial.print(k[j]); Serial.print(" ");
  }
  Serial.println();
}

// ---- Button (local perturbation, optional) ---------------------------------

void checkButton() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    unsigned long now = millis();
    if (now - lastButtonPress > BUTTON_DEBOUNCE_MS) {
      float strength = KNOCK_STRENGTH_MIN +
        (float)random(1000) / 1000.0f * (KNOCK_STRENGTH_MAX - KNOCK_STRENGTH_MIN);
      x_own += strength;
      x[NODE_INDEX] = x_own;
      triggerKnockFlash();
      lastButtonPress = now;
      Serial.print("[node] Local button knock: ");
      Serial.println(strength);
    }
  }
}

// ---- Integration step ------------------------------------------------------

void stepSimulation() {
  float dx = 0.0f;
  for (int j = 0; j < NUM_NODES; j++) {
    dx += k[j] * x[j];
  }
  x_own += dx * DT;
  x[NODE_INDEX] = x_own;

  if (abs(x_own) > BOUND) {
    fireUniselector();
  }
}

// ---- Broadcast own state ---------------------------------------------------

void broadcastState() {
  StateMessage msg;
  msg.msg_type  = 2;
  msg.source_id = DEVICE_ID;
  msg.x_value   = x_own;
  esp_now_send(broadcastAddress, (uint8_t *)&msg, sizeof(msg));
}

// ---- LED rendering ---------------------------------------------------------

void triggerKnockFlash() {
  knockFlashUntil = millis() + KNOCK_FLASH_MS;
  inKnockFlash = true;
}

void renderState() {
  if (inKnockFlash && millis() < knockFlashUntil) {
    fill_solid(leds, NUM_LEDS, CRGB(255, 200, 0));  // yellow flash
    FastLED.show();
    return;
  }
  inKnockFlash = false;

  float mag = abs(x_own) * DISPLAY_SCALE;
  int litCount = NUM_LEDS - (int)(mag * NUM_LEDS);
  litCount = constrain(litCount, 0, NUM_LEDS);

  CRGB colour;
  if (x_own >= 0) {
    colour = CRGB(200, 0, 0);    // red: positive
  } else {
    colour = CRGB(0, 0, 200);    // blue: negative
  }

  for (int i = 0; i < NUM_LEDS; i++) {
    if (i < litCount) {
      leds[i] = CRGB(0, 180, 0); // green: near equilibrium
    } else {
      leds[i] = colour;           // red/blue: displaced
    }
  }

  FastLED.show();
}

// ---- Serial plotter output -------------------------------------------------

void printPlotter() {
  // All four x values on one line -- readable in Arduino serial plotter
  for (int i = 0; i < NUM_NODES; i++) {
    Serial.print(x[i]);
    if (i < NUM_NODES - 1) Serial.print("\t");
  }
  Serial.println();
}

// ---- Setup -----------------------------------------------------------------

void initK() {
  for (int j = 0; j < NUM_NODES; j++) {
    k[j] = ((float)random(2000) / 1000.0f - 1.0f) * K_MAX;
  }
  k[NODE_INDEX] = -DIAGONAL_DAMPING;  // own diagonal: always damping
}

void setup() {
  delay(BOOT_DELAY_MS * DEVICE_ID);  // stagger boot across nodes

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

// ---- Main loop -------------------------------------------------------------

void loop() {
  checkButton();
  stepSimulation();
  broadcastState();
  renderState();
  printPlotter();

  delay(60);  // approx DT pacing
}

/*
 * FLASH INSTRUCTIONS:
 *   Board:    ESP32S3 Dev Module (or ESP32-S3 Supermini)
 *   USB Mode: Hardware CDC and JTAG
 *   Libraries: FastLED (via Library Manager)
 * 
 *   Before flashing each node, change DEVICE_ID:
 *     Corner 1 (x[0]) -> DEVICE_ID = 1  (receives knock from sensor)
 *     Corner 2 (x[1]) -> DEVICE_ID = 2
 *     Corner 3 (x[2]) -> DEVICE_ID = 3
 *     Corner 4 (x[3]) -> DEVICE_ID = 4
 * 
 *   Flash sensor node separately with homeostat_sensor_node.ino
 */
