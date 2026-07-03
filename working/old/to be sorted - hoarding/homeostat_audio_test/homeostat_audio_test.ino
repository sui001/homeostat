/*
 * PROJECT: Homeostat Audio Test -- MAX98357A I2S DAC
 * DEVICE:  ESP32-S3 Supermini
 *
 * Standalone test for MAX98357A I2S audio output.
 * Simulates the four homeostat x[] variables as oscillating values
 * and maps them to four audio sine wave voices mixed to mono output.
 * When at rest: consonant, calm chord.
 * When disturbed: frequencies drift and clash (hunting behaviour).
 *
 * Use this to confirm I2S wiring and audio behaviour before
 * integrating into homeostat_v3.
 *
 * WIRING:
 *   MAX98357A BCLK  -> GPIO 5
 *   MAX98357A LRC   -> GPIO 6
 *   MAX98357A DIN   -> GPIO 4
 *   MAX98357A GND   -> GND
 *   MAX98357A VIN   -> 3.3V or 5V
 *   MAX98357A SD    -> leave floating (always on)
 *
 * NO LIBRARIES NEEDED -- uses ESP32 Arduino core built-in I2S driver.
 *
 * FLASH SETTINGS:
 *   Board:    ESP32S3 Dev Module
 *   USB Mode: Hardware CDC and JTAG
 *   PSRAM:    Disabled (not needed)
 */

#include <driver/i2s.h>
#include <math.h>

// ---------------------------------------------------------------------------
// I2S CONFIG
// ---------------------------------------------------------------------------

#define I2S_PORT        I2S_NUM_0
#define I2S_BCLK        5
#define I2S_LRC         6
#define I2S_DOUT        4

#define SAMPLE_RATE     22050          // Hz -- 22k is fine for this
#define BUFFER_SIZE     256            // samples per DMA write

// ---------------------------------------------------------------------------
// AUDIO PARAMETERS
// ---------------------------------------------------------------------------

// Base frequencies for four voices (roughly a minor 7th chord -- interesting when stable)
const float BASE_FREQ[4] = { 220.0f, 261.6f, 329.6f, 392.0f };  // A3, C4, E4, G4

// How much x[] deviation shifts the frequency (Hz per unit)
const float FREQ_SENSITIVITY = 80.0f;

// Master volume: 0.0 - 1.0 per voice (sum should stay < 1.0 to avoid clipping)
const float VOICE_VOLUME = 0.18f;

// ---------------------------------------------------------------------------
// SIMULATED HOMEOSTAT STATE
// (replace these with real x[] from homeostat logic when integrating)
// ---------------------------------------------------------------------------

float x[4] = { 0.0f, 0.0f, 0.0f, 0.0f };   // system variables
float phase[4] = { 0.0f, 0.0f, 0.0f, 0.0f }; // per-voice phase accumulator

// Fake oscillation to demo audio behaviour without full homeostat
// In integration, remove this and use real x[] from the ODE
float demoTime = 0.0f;

void updateFakeHomeostat() {
  demoTime += 0.002f;
  // Gentle slow drift -- simulates post-knock hunting
  x[0] =  0.6f * sinf(demoTime * 0.7f);
  x[1] = -0.4f * sinf(demoTime * 1.1f + 0.5f);
  x[2] =  0.3f * sinf(demoTime * 0.9f + 1.2f);
  x[3] = -0.5f * sinf(demoTime * 0.5f + 2.0f);
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

  Serial.println("I2S initialised.");
}

// ---------------------------------------------------------------------------
// AUDIO RENDER
// ---------------------------------------------------------------------------

void renderAudio() {
  static int16_t buffer[BUFFER_SIZE * 2];  // *2 for stereo (L+R interleaved)

  for (int i = 0; i < BUFFER_SIZE; i++) {
    float sample = 0.0f;

    for (int v = 0; v < 4; v++) {
      // x[v] shifts frequency up or down from base
      float freq = BASE_FREQ[v] + (x[v] * FREQ_SENSITIVITY);
      freq = constrain(freq, 50.0f, 1000.0f);  // safety clamp

      phase[v] += (2.0f * M_PI * freq) / SAMPLE_RATE;
      if (phase[v] > 2.0f * M_PI) phase[v] -= 2.0f * M_PI;

      sample += sinf(phase[v]) * VOICE_VOLUME;
    }

    // Clamp and convert to 16-bit int
    sample = constrain(sample, -1.0f, 1.0f);
    int16_t s16 = (int16_t)(sample * 32767.0f);

    buffer[i * 2]     = s16;  // Left
    buffer[i * 2 + 1] = s16;  // Right (same -- mono mix)
  }

  size_t written;
  i2s_write(I2S_PORT, buffer, sizeof(buffer), &written, portMAX_DELAY);
}

// ---------------------------------------------------------------------------
// SETUP / LOOP
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("=== HOMEOSTAT AUDIO TEST ===");
  Serial.println("Four voices, frequencies modulated by simulated x[].");
  Serial.println("At rest: consonant chord. Disturbed: drift and clash.");

  setupI2S();
}

void loop() {
  updateFakeHomeostat();   // swap for real ODE when integrating
  renderAudio();           // fill I2S DMA buffer and send
}

/*
 * ---------------------------------------------------------------------------
 * INTEGRATION NOTES (for homeostat_v3 merge)
 * ---------------------------------------------------------------------------
 * 1. Remove updateFakeHomeostat() and demoTime
 * 2. Remove the float x[4] declaration here -- use the one from v3
 * 3. Call renderAudio() in the main loop alongside FastLED.show()
 * 4. Add setupI2S() call in setup()
 * 5. Tune FREQ_SENSITIVITY and BASE_FREQ to taste
 *    -- higher sensitivity = more dramatic pitch shifts on perturbation
 * ---------------------------------------------------------------------------
 */
