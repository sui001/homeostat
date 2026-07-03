/*
 * PROJECT: Homeostat -- Dual Active Buzzer
 * VERSION: v1.0.0
 * DEVICE:  ESP32-S3 Supermini
 *
 * Full four-node coupled ODE running locally. No ESP-NOW, no LEDs.
 * Two active buzzers track x[0] and x[1] via pulse rate.
 * Near zero: slow clicks. Agitated: rapid clicking. S2: double click.
 *
 * WIRING:
 *   GPIO 0  -> Boot button (other end to GND)
 *   GPIO 4  -> Active buzzer 1 S pin (x[0])
 *   GPIO 5  -> Active buzzer 2 S pin (x[1])
 *   3.3V    -> both buzzer + pins
 *   GND     -> both buzzer - pins
 */

#define VERSION "v1.0.0"

#define BUTTON_PIN  0
#define BUZ_1       4   // x[0]
#define BUZ_2       5   // x[1]

const int   N                      = 4;
const float BOUND                  = 1.0f;
const float DT                     = 0.06f;
const float K_MAX                  = 0.25f;
const float DIAGONAL_DAMPING       = 0.08f;
const float PERTURB_MIN            = 3.0f;
const float PERTURB_MAX            = 6.0f;

// Pulse rate range -- ms between clicks
// calm (norm~0): PULSE_SLOW ms apart
// agitated (norm~1): PULSE_FAST ms apart
const unsigned long PULSE_SLOW     = 1200;
const unsigned long PULSE_FAST     = 80;
const unsigned long PULSE_ON_MS    = 8;    // click duration

const unsigned long LOOP_MS            = 20;   // tighter loop for click timing
const unsigned long BUTTON_DEBOUNCE_MS = 400;
const unsigned long MIN_RECONFIG_MS    = 2000;

float x[N];
float k[N][N];
unsigned long lastReconfig[N] = {0};
unsigned long lastButton      = 0;

// per-buzzer pulse state
unsigned long lastPulse[2]  = {0, 0};
bool          buzState[2]   = {false, false};
unsigned long buzOnUntil[2] = {0, 0};
bool          s2Flag[2]     = {false, false};

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
      if (i < 2) s2Flag[i] = true;  // flag double-click on S2
      Serial.print("[S2] unit "); Serial.println(i);
    }
  }
}

void updateBuzzers() {
  unsigned long now = millis();
  const int buzPins[2] = {BUZ_1, BUZ_2};

  for (int b = 0; b < 2; b++) {
    // turn off if click duration elapsed
    if (buzState[b] && now >= buzOnUntil[b]) {
      digitalWrite(buzPins[b], LOW);
      buzState[b] = false;
    }

    if (!buzState[b]) {
      float norm = constrain(fabsf(x[b]) / BOUND, 0.0f, 1.0f);
      // map norm to interval -- high norm = short interval
      unsigned long interval = (unsigned long)(PULSE_SLOW - norm * (PULSE_SLOW - PULSE_FAST));

      if (now - lastPulse[b] >= interval) {
        // S2 double-click: two rapid clicks
        if (s2Flag[b]) {
          digitalWrite(buzPins[b], HIGH); delay(12);
          digitalWrite(buzPins[b], LOW);  delay(60);
          digitalWrite(buzPins[b], HIGH); delay(12);
          digitalWrite(buzPins[b], LOW);
          s2Flag[b] = false;
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
    if (now - lastButton > BUTTON_DEBOUNCE_MS) {
      float sign = (random(2) == 0) ? 1.0f : -1.0f;
      x[0] += sign * rf(PERTURB_MIN, PERTURB_MAX);
      lastButton = now;
      Serial.print("[knock] x[0]: "); Serial.println(x[0], 3);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.print("=== HOMEOSTAT DUAL ACTIVE BUZZER "); Serial.print(VERSION); Serial.println(" ===");

  randomSeed(esp_random());
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZ_1, OUTPUT); digitalWrite(BUZ_1, LOW);
  pinMode(BUZ_2, OUTPUT); digitalWrite(BUZ_2, LOW);

  for (int i = 0; i < N; i++) randomiseK(i);

  // non-zero seed
  x[0] =  0.30f;
  x[1] = -0.20f;
  x[2] =  0.15f;
  x[3] = -0.25f;

  // boot confirm -- two clicks each
  for (int b = 0; b < 2; b++) {
    const int p = (b == 0) ? BUZ_1 : BUZ_2;
    for (int i = 0; i < 2; i++) {
      digitalWrite(p, HIGH); delay(15);
      digitalWrite(p, LOW);  delay(120);
    }
    delay(200);
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
 *   PULSE_SLOW  -- click interval at rest (ms)
 *   PULSE_FAST  -- click interval fully agitated (ms)
 *   PULSE_ON_MS -- click duration (ms)
 *   K_MAX       -- coupling strength
 *   BOUND       -- viable range before S2 fires
 *
 * VERSION HISTORY:
 *   v1.0.0 -- dual active buzzer, pulse rate tracks x[0] and x[1]
 */
