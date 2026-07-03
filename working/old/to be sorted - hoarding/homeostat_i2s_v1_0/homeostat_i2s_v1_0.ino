/*
 * PROJECT: Homeostat -- Four-node ODE + MAX98357A I2S Audio
 * VERSION: v1.0.0
 * DEVICE:  ESP32-S3 Supermini
 *
 * Full four-node coupled ODE running locally. No ESP-NOW, no LEDs.
 * Four sine voices mixed to mono, each tracking its own x[] variable.
 * At rest: consonant minor 7th chord. Hunting: frequencies drift and clash.
 * Button knocks x[0]. S2 fires per unit when |x[i]| exceeds BOUND.
 *
 * WIRING:
 *   MAX98357A BCLK  -> GPIO 5
 *   MAX98357A LRC   -> GPIO 6
 *   MAX98357A DIN   -> GPIO 4
 *   MAX98357A GND   -> GND
 *   MAX98357A VIN   -> 5V (VBUS)
 *   MAX98357A SD    -> floating
 *   GPIO 0          -> Boot button (other end to GND)
 */

#define VERSION "v1.0.0"

#include <driver/i2s.h>
#include <math.h>

// ---------------------------------------------------------------------------
// I2S
// ---------------------------------------------------------------------------

#define I2S_PORT    I2S_NUM_0
#define I2S_BCLK    5
#define I2S_LRC     6
#define I2S_DOUT    4
#define SAMPLE_RATE 22050
#define BUFFER_SIZE 256

// ---------------------------------------------------------------------------
// HARDWARE
// ---------------------------------------------------------------------------

#define BUTTON_PIN  0

// ---------------------------------------------------------------------------
// PHYSICS
// ---------------------------------------------------------------------------

const int   N                   = 4;
const float BOUND               = 1.0f;
const float DT                  = 0.06f;
const float K_MAX               = 0.25f;
const float DIAGONAL_DAMPING    = 0.08f;
const float PERTURB_MIN         = 3.0f;
const float PERTURB_MAX         = 6.0f;

const unsigned long LOOP_MS          = 60;
const unsigned long DEBOUNCE_MS      = 400;
const unsigned long MIN_RECONFIG_MS  = 2000;

// ---------------------------------------------------------------------------
// AUDIO
// ---------------------------------------------------------------------------

// Base chord -- minor 7th, sounds interesting when stable
const float BASE_FREQ[N]  = { 220.0f, 261.6f, 329.6f, 392.0f };
const float FREQ_SENS     = 120.0f;   // Hz shift per unit of x
const float VOICE_VOL     = 0.18f;    // per voice, four voices sum < 1.0

// ---------------------------------------------------------------------------
// STATE
// ---------------------------------------------------------------------------

float x[N]   = { 0.30f, -0.20f, 0.15f, -0.25f };  // non-zero seed
float k[N][N];
float phase[N] = { 0 };

unsigned long lastReconfig[N] = { 0 };
unsigned long lastButton      = 0;

// ---------------------------------------------------------------------------
// UTILITY
// ---------------------------------------------------------------------------

float rf(float lo, float hi) {
  return lo + ((float)random(10000) / 10000.0f) * (hi - lo);
}

void randomiseK(int i) {
  for (int j = 0; j < N; j++)
    k[i][j] = (i == j) ? rf(-DIAGONAL_DAMPING, -DIAGONAL_DAMPING * 0.1f) : rf(-K_MAX, K_MAX);
}

// ---------------------------------------------------------------------------
// SIMULATION
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// BUTTON
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// AUDIO
// ---------------------------------------------------------------------------

void renderAudio() {
  static int16_t buffer[BUFFER_SIZE * 2];

  for (int i = 0; i < BUFFER_SIZE; i++) {
    float sample = 0.0f;

    for (int v = 0; v < N; v++) {
      float freq = BASE_FREQ[v] + (x[v] * FREQ_SENS);
      freq = constrain(freq, 50.0f, 1200.0f);

      phase[v] += (2.0f * M_PI * freq) / SAMPLE_RATE;
      if (phase[v] > 2.0f * M_PI) phase[v] -= 2.0f * M_PI;

      sample += sinf(phase[v]) * VOICE_VOL;
    }

    sample = constrain(sample, -1.0f, 1.0f);
    int16_t s16 = (int16_t)(sample * 32767.0f);
    buffer[i * 2]     = s16;
    buffer[i * 2 + 1] = s16;
  }

  size_t written;
  i2s_write(I2S_PORT, buffer, sizeof(buffer), &written, portMAX_DELAY);
}

// ---------------------------------------------------------------------------
// I2S SETUP
// ---------------------------------------------------------------------------

void setupI2S() {
  i2s_config_t cfg = {
    .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate          = SAMPLE_RATE,
    .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count        = 4,
    .dma_buf_len          = BUFFER_SIZE,
    .use_apll             = false,
    .tx_desc_auto_clear   = true,
  };

  i2s_pin_config_t pins = {
    .bck_io_num   = I2S_BCLK,
    .ws_io_num    = I2S_LRC,
    .data_out_num = I2S_DOUT,
    .data_in_num  = I2S_PIN_NO_CHANGE,
  };

  i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
  i2s_set_pin(I2S_PORT, &pins);
  i2s_zero_dma_buffer(I2S_PORT);
  Serial.println("[i2s] ready");
}

// ---------------------------------------------------------------------------
// SETUP / LOOP
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.print("=== HOMEOSTAT I2S "); Serial.print(VERSION); Serial.println(" ===");

  randomSeed(esp_random());
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  for (int i = 0; i < N; i++) randomiseK(i);

  setupI2S();
}

void loop() {
  checkButton();
  stepSimulation();
  renderAudio();

  for (int i = 0; i < N; i++) {
    Serial.print(x[i], 4);
    if (i < N-1) Serial.print("\t");
  }
  Serial.println();
}

/*
 * TUNING:
 *   BASE_FREQ     -- chord at rest, one frequency per voice
 *   FREQ_SENS     -- how dramatically x[] shifts pitch
 *   VOICE_VOL     -- per-voice volume, four voices sum -- keep total < 1.0
 *   K_MAX         -- coupling strength
 *   PERTURB_MIN/MAX -- knock strength on button press
 *
 * VERSION HISTORY:
 *   v1.0.0 -- four-node ODE + MAX98357A I2S, button perturbation
 */
