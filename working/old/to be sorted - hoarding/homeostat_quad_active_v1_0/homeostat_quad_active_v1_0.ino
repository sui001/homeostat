/*
 * PROJECT: Homeostat -- Quad Active Buzzer
 * VERSION: v1.0.0
 * DEVICE:  ESP32-S3 Supermini
 *
 * Full four-node coupled ODE running locally. No ESP-NOW, no LEDs.
 * Four active buzzers, pulse rate tracks each x[] variable.
 * GPIO 4+5: low clickers (x[0], x[1])
 * GPIO 6+7: high tweeters (x[2], x[3])
 * Near zero: slow clicks. Agitated: rapid clicking. S2: double click.
 *
 * WIRING:
 *   GPIO 0  -> Boot button (other end to GND)
 *   GPIO 4  -> Buzzer 0 S pin   (x[0]) -- low clicker
 *   GPIO 5  -> Buzzer 1 S pin   (x[1]) -- low clicker
 *   GPIO 6  -> Buzzer 2 S pin   (x[2]) -- high tweeter
 *   GPIO 7  -> Buzzer 3 S pin   (x[3]) -- high tweeter
 *   3.3V    -> all buzzer + pins
 *   GND     -> all buzzer - pins
 */

#define VERSION "v1.0.0"

#define BUTTON_PIN  0

const int   N                   = 4;
const int   buzPins[N]          = {4, 5, 6, 7};

const float BOUND               = 1.0f;
const float DT                  = 0.06f;
const float K_MAX               = 0.25f;
const float DIAGONAL_DAMPING    = 0.08f;
const float PERTURB_MIN         = 3.0f;
const float PERTURB_MAX         = 6.0f;

// Pulse rate range per buzzer type
// clickers (0,1) -- slower, heavier
const unsigned long PULSE_SLOW_CLICK  = 1200;
const unsigned long PULSE_FAST_CLICK  = 80;
// tweeters (2,3) -- faster, lighter
const unsigned long PULSE_SLOW_TWEET  = 800;
const unsigned long PULSE_FAST_TWEET  = 40;

const unsigned long PULSE_ON_MS       = 8;
const unsigned long LOOP_MS           = 20;
const unsigned long DEBOUNCE_MS       = 400;
const unsigned long MIN_RECONFIG_MS   = 2000;

float x[N]   = { 0.30f, -0.20f, 0.15f, -0.25f };
float k[N][N];

unsigned long lastReconfig[N] = {0};
unsigned long lastButton      = 0;
unsigned long lastPulse[N]    = {0};
unsigned long buzOnUntil[N]   = {0};
bool          buzState[N]     = {false};
bool          s2Flag[N]       = {false};

float rf(float lo, float hi) {
  return lo + ((float)random(10000) / 10000.0f) * (hi - lo);
}

void randomiseK(int i) {
  for (int j = 0; j < N; j++)
    k[i][j] = (i == j) ? rf(-DIAGONAL_DAMPING, -DIAGONAL_DAMPING * 0.1f) : rf(-K_MAX, K_MAX);
}

void stepSimulation() {
  float nx[N];
  for (int i = 0; i < N; i++) {
    float dx = 0;
    for (int j = 0; j < N; j++) dx += k[i][j] * x[j];
    nx[i] = constrain(x[i] + dx * DT, -BOUND * 3.0f, BOUND * 3.0f);
  }
  for (int i = 0; i < N; i++) x[i] = nx[i];

  unsigned long now = millis();
  for (int i = 0; i < N; i++) {
    if (fabsf(x[i]) > BOUND && now - lastReconfig[i] > MIN_RECONFIG_MS) {
      randomiseK(i);
      lastReconfig[i] = now;
      s2Flag[i] = true;
      Serial.print("[S2] unit "); Serial.println(i);
    }
  }
}

void updateBuzzers() {
  unsigned long now = millis();

  for (int b = 0; b < N; b++) {
    // turn off if click duration elapsed
    if (buzState[b] && now >= buzOnUntil[b]) {
      digitalWrite(buzPins[b], LOW);
      buzState[b] = false;
    }

    if (!buzState[b]) {
      float norm = constrain(fabsf(x[b]) / BOUND, 0.0f, 1.0f);

      // pick timing based on buzzer type
      unsigned long slowMs = (b < 2) ? PULSE_SLOW_CLICK : PULSE_SLOW_TWEET;
      unsigned long fastMs = (b < 2) ? PULSE_FAST_CLICK : PULSE_FAST_TWEET;
      unsigned long interval = (unsigned long)(slowMs - norm * (slowMs - fastMs));

      if (now - lastPulse[b] >= interval) {
        if (s2Flag[b]) {
          // double click on S2
          digitalWrite(buzPins[b], HIGH); delay(12);
          digitalWrite(buzPins[b], LOW);  delay(60);
          digitalWrite(buzPins[b], HIGH); delay(12);
          digitalWrite(buzPins[b], LOW);
          s2Flag[b]    = false;
          lastPulse[b] = now;
        } else {
          digitalWrite(buzPins[b], HIGH);
          buzState[b]   = true;
          buzOnUntil[b] = now + PULSE_ON_MS;
          lastPulse[b]  = now;
        }
      }
    }
  }
}

void checkButton() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    unsigned long now = millis();
    if (now - lastButton > DEBOUNCE_MS) {
      float sign = (random(2) == 0) ? 1.0f : -1.0f;
      x[0] += sign * rf(PERTURB_MIN, PERTURB_MAX);
      lastButton = now;
      Serial.print("[knock] x[0]: "); Serial.println(x[0], 3);
    }
  }
}

void setup() {
  delay(4000);  // boot stabilisation -- do not remove

  Serial.begin(115200);
  Serial.print("=== HOMEOSTAT QUAD ACTIVE "); Serial.print(VERSION); Serial.println(" ===");

  randomSeed(esp_random());
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  for (int i = 0; i < N; i++) {
    pinMode(buzPins[i], OUTPUT);
    digitalWrite(buzPins[i], LOW);
    randomiseK(i);
  }

  // boot confirm -- each buzzer clicks twice in sequence
  for (int b = 0; b < N; b++) {
    for (int i = 0; i < 2; i++) {
      digitalWrite(buzPins[b], HIGH); delay(15);
      digitalWrite(buzPins[b], LOW);  delay(120);
    }
    delay(150);
  }

  Serial.println("[init] ready");
}

void loop() {
  checkButton();
  stepSimulation();
  updateBuzzers();

  for (int i = 0; i < N; i++) {
    Serial.print(x[i], 4);
    if (i < N-1) Serial.print("\t");
  }
  Serial.println();

  delay(LOOP_MS);
}

/*
 * TUNING:
 *   PULSE_SLOW_CLICK/TWEET  -- click interval at rest per buzzer type
 *   PULSE_FAST_CLICK/TWEET  -- click interval fully agitated
 *   PULSE_ON_MS             -- click duration
 *   K_MAX                   -- coupling strength
 *   BOUND                   -- viable range before S2 fires
 *
 * VERSION HISTORY:
 *   v1.0.0 -- four active buzzers, two clickers + two tweeters
 */
