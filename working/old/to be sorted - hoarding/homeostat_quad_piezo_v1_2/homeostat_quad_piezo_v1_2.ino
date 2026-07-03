/*
 * PROJECT: Homeostat -- Quad Piezo, Single MCU
 * VERSION: v1.2.0
 * DEVICE:  ESP32-S3 Supermini
 *
 * v1.2.0: dropped ledcWriteTone, using ledcChangeFrequency + ledcWrite
 * directly to force independent channel control.
 *
 * WIRING:
 *   GPIO 0  -> Boot button (other end to GND)
 *   GPIO 4  -> Piezo 0 +   (x[0])
 *   GPIO 5  -> Piezo 1 +   (x[1])
 *   GPIO 6  -> Piezo 2 +   (x[2])
 *   GPIO 7  -> Piezo 3 +   (x[3])
 *   GND     -> all piezo -
 */

#define VERSION "v1.2.0"

#define BUTTON_PIN  0

const int   N                      = 4;
const int   pins[N]                = {4, 5, 6, 7};
const int   channels[N]            = {0, 1, 2, 3};

const float BOUND                  = 1.0f;
const float DT                     = 0.06f;
const float K_MAX                  = 0.25f;   // reduced -- single chip runs hotter
const float DIAGONAL_DAMPING       = 0.08f;
const float PERTURB_MIN            = 3.0f;
const float PERTURB_MAX            = 6.0f;
const float BASE_FREQ              = 150.0f;
const float FREQ_RANGE             = 700.0f;

const unsigned long LOOP_MS            = 60;
const unsigned long BUTTON_DEBOUNCE_MS = 400;
const unsigned long MIN_RECONFIG_MS    = 2000;

float x[N];
float k[N][N];
unsigned long lastReconfig[N] = {0};
unsigned long lastButton = 0;

float rf(float lo, float hi) {
  return lo + ((float)random(10000) / 10000.0f) * (hi - lo);
}

void randomiseK(int i) {
  for (int j = 0; j < N; j++)
    k[i][j] = (i == j) ? rf(-DIAGONAL_DAMPING, -DIAGONAL_DAMPING * 0.1f) : rf(-K_MAX, K_MAX);
}

void toneOn(int ch, uint32_t freq) {
  if (freq == 0) {
    ledcWrite(ch, 0);
    return;
  }
  ledcChangeFrequency(ch, freq, 8);
  ledcWrite(ch, 128);  // 50% duty -- square wave
}

void toneOff(int ch) {
  ledcWrite(ch, 0);
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
      Serial.print("[S2] unit "); Serial.println(i);
    }
  }
}

void updateAudio() {
  for (int i = 0; i < N; i++) {
    float norm = constrain(fabsf(x[i]) / BOUND, 0.0f, 1.0f);
    if (norm < 0.01f) {
      toneOff(channels[i]);
    } else {
      toneOn(channels[i], (uint32_t)(BASE_FREQ + norm * FREQ_RANGE));
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
  Serial.print("=== HOMEOSTAT QUAD PIEZO "); Serial.print(VERSION); Serial.println(" ===");

  randomSeed(esp_random());
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  for (int i = 0; i < N; i++) {
    ledcAttachChannel(pins[i], BASE_FREQ, 8, channels[i]);
    toneOff(channels[i]);
    randomiseK(i);
  }

  // diagnostic -- beep each piezo in sequence
  Serial.println("[init] diagnostic beep...");
  for (int i = 0; i < N; i++) {
    Serial.print("[init] piezo "); Serial.println(i);
    toneOn(channels[i], 440 + i * 110);  // different pitch per piezo
    delay(400);
    toneOff(channels[i]);
    delay(150);
  }
  Serial.println("[init] done.");

  // non-zero seed
  x[0] =  0.30f;
  x[1] = -0.20f;
  x[2] =  0.15f;
  x[3] = -0.25f;
}

void loop() {
  checkButton();
  stepSimulation();
  updateAudio();

  for (int i = 0; i < N; i++) {
    Serial.print(x[i], 4);
    if (i < N-1) Serial.print("\t");
  }
  Serial.println();

  delay(LOOP_MS);
}

/*
 * VERSION HISTORY:
 *   v1.0.0 -- four piezos, one per variable, single MCU
 *   v1.1.0 -- explicit LEDC channel assignment
 *   v1.1.1 -- non-zero initial conditions, slower S2
 *   v1.2.0 -- ledcChangeFrequency + ledcWrite replacing ledcWriteTone,
 *             diagnostic beep sequence on boot, reduced K_MAX
 */
