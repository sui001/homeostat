/*
 * PROJECT: Four-unit Ashby-style Homeostat Prototype
 * BOARD:   ESP32-S3 Supermini
 * LIBRARY: FastLED
 *
 * PURPOSE
 * -------
 * Faithful recreation of Ashby's homeostat (1947) as a visual display.
 * Four coupled differential equations, fully random coupling matrix.
 * Button knock disturbs x[0], propagates through coupling to all units.
 *
 * DISPLAY
 * -------
 * 4 green LEDs per strip = at rest (x near zero)
 * LEDs drop out as x moves away from zero
 * Red = positive, Blue = negative
 * Yellow flash (all 16) = knock
 *
 * WIRING
 * ------
 * GPIO 4  -> Strip 1 data (x[0])
 * GPIO 5  -> Strip 2 data (x[1])
 * GPIO 6  -> Strip 3 data (x[2])
 * GPIO 7  -> Strip 4 data (x[3])
 * GPIO 9  -> Button -> GND
 * 5V/GND  -> LED strips, external supply recommended
 */

#include <FastLED.h>

// ---------------------------------------------------------------------------
// HARDWARE CONFIG
// ---------------------------------------------------------------------------

#define NUM_UNITS       4
#define LEDS_PER_STRIP  4

#define LED_PIN_1       4
#define LED_PIN_2       5
#define LED_PIN_3       6
#define LED_PIN_4       7

#define BUTTON_PIN      9

CRGB strips[NUM_UNITS][LEDS_PER_STRIP];

// ---------------------------------------------------------------------------
// HOMEOSTAT PARAMETERS
// ---------------------------------------------------------------------------

const float BOUND                   = 1.0f;
const float DT                      = 0.06f;
const float K_MAX                   = 0.35f;
const float DIAGONAL_DAMPING        = 0.08f;

const float KNOCK_STRENGTH_MIN      = 2.0f;
const float KNOCK_STRENGTH_MAX      = 5.0f;

const unsigned long BUTTON_DEBOUNCE_MS       = 400;
const unsigned long MIN_RECONFIG_INTERVAL_MS = 200;
const unsigned long KNOCK_FLASH_MS           = 250;

// ---------------------------------------------------------------------------
// DISPLAY
// ---------------------------------------------------------------------------

const uint8_t HUE_NEGATIVE = 160;
const uint8_t HUE_ZERO     = 96;
const uint8_t HUE_POSITIVE = 0;
const uint8_t HUE_FLASH    = 48;

// ---------------------------------------------------------------------------
// STATE
// ---------------------------------------------------------------------------

float x[NUM_UNITS] = {0.0f, 0.0f, 0.0f, 0.0f};
float k[NUM_UNITS][NUM_UNITS];

unsigned long lastReconfigTime  = 0;
unsigned long lastButtonTime    = 0;
unsigned long knockFlashStart   = 0;
bool          lastButtonState   = false;
bool          knockFlashing     = false;

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  FastLED.addLeds<WS2812, LED_PIN_1, GRB>(strips[0], LEDS_PER_STRIP);
  FastLED.addLeds<WS2812, LED_PIN_2, GRB>(strips[1], LEDS_PER_STRIP);
  FastLED.addLeds<WS2812, LED_PIN_3, GRB>(strips[2], LEDS_PER_STRIP);
  FastLED.addLeds<WS2812, LED_PIN_4, GRB>(strips[3], LEDS_PER_STRIP);
  FastLED.setBrightness(160);

  randomSeed(analogRead(0) + micros());
  randomizeAllK();

  renderState();
  FastLED.show();

  Serial.println("=== HOMEOSTAT v3 ===");
  Serial.println("Starts balanced. Press button to knock.");
  Serial.println("x0\tx1\tx2\tx3");
}

// ---------------------------------------------------------------------------
// MAIN LOOP
// ---------------------------------------------------------------------------

void loop() {
  float knock = readButton();

  stepHomeostat(knock);
  checkViabilityAndReconfigure();
  renderState();
  FastLED.show();
  printDebug();

  delay((int)(DT * 1000.0f));
}

// ---------------------------------------------------------------------------
// BUTTON
// ---------------------------------------------------------------------------

float readButton() {
  bool  pressed = (digitalRead(BUTTON_PIN) == LOW);
  float knock   = 0.0f;
  unsigned long now = millis();

  if (pressed && !lastButtonState && (now - lastButtonTime > BUTTON_DEBOUNCE_MS)) {
    knock           = randomFloat(KNOCK_STRENGTH_MIN, KNOCK_STRENGTH_MAX);
    lastButtonTime  = now;
    knockFlashStart = now;
    knockFlashing   = true;
    Serial.print("KNOCK=");
    Serial.println(knock, 2);
  }

  lastButtonState = pressed;
  return knock;
}

// ---------------------------------------------------------------------------
// HOMEOSTAT EQUATIONS
// ---------------------------------------------------------------------------

void stepHomeostat(float knock) {
  float dx[NUM_UNITS] = {0, 0, 0, 0};

  for (int i = 0; i < NUM_UNITS; i++)
    for (int j = 0; j < NUM_UNITS; j++)
      dx[i] += k[i][j] * x[j];

  dx[0] += knock;

  for (int i = 0; i < NUM_UNITS; i++) {
    x[i] += DT * dx[i];
    x[i] = constrain(x[i], -BOUND * 3.0f, BOUND * 3.0f);
  }
}

// ---------------------------------------------------------------------------
// VIABILITY + ADAPTATION
// ---------------------------------------------------------------------------

void checkViabilityAndReconfigure() {
  unsigned long now = millis();
  if (now - lastReconfigTime < MIN_RECONFIG_INTERVAL_MS) return;

  for (int i = 0; i < NUM_UNITS; i++) {
    if (fabsf(x[i]) > BOUND) {
      randomizeRow(i);
      x[i] = 0.0f;
      lastReconfigTime = now;
      Serial.print("Reconfig unit ");
      Serial.println(i);
      return;
    }
  }
}

// ---------------------------------------------------------------------------
// COEFFICIENT RANDOMIZATION
// ---------------------------------------------------------------------------

void randomizeAllK() {
  for (int i = 0; i < NUM_UNITS; i++) randomizeRow(i);
}

void randomizeRow(int i) {
  for (int j = 0; j < NUM_UNITS; j++) {
    k[i][j] = (i == j)
      ? -DIAGONAL_DAMPING
      : randomFloat(-K_MAX, K_MAX);
  }
}

float randomFloat(float low, float high) {
  return low + (high - low) * ((float)random(0, 10001) / 10000.0f);
}

// ---------------------------------------------------------------------------
// LED DISPLAY
// ---------------------------------------------------------------------------

void renderState() {
  unsigned long now = millis();

  if (knockFlashing) {
    if (now - knockFlashStart < KNOCK_FLASH_MS) {
      for (int i = 0; i < NUM_UNITS; i++)
        for (int j = 0; j < LEDS_PER_STRIP; j++)
          strips[i][j] = CHSV(HUE_FLASH, 255, 255);
      return;
    } else {
      knockFlashing = false;
    }
  }

  for (int i = 0; i < NUM_UNITS; i++) {
    float magnitude = constrain(fabsf(x[i]) / 0.3f, 0.0f, 1.0f);
    uint8_t litCount = LEDS_PER_STRIP - (uint8_t)(magnitude * (LEDS_PER_STRIP - 1));
    litCount = constrain(litCount, 1, LEDS_PER_STRIP);

    uint8_t hue = (x[i] > 0.08f) ? HUE_POSITIVE
                : (x[i] < -0.08f) ? HUE_NEGATIVE
                : HUE_ZERO;

    for (int j = 0; j < LEDS_PER_STRIP; j++) {
      if (j < litCount) {
        strips[i][j] = CHSV(hue, 220, 200);
      } else {
        strips[i][j] = CRGB::Black;
      }
    }
  }
}

// ---------------------------------------------------------------------------
// SERIAL DEBUG
// ---------------------------------------------------------------------------

void printDebug() {
  static unsigned long lastPrint = 0;
  unsigned long now = millis();
  if (now - lastPrint < 50) return;
  lastPrint = now;

  Serial.print(x[0], 4); Serial.print('\t');
  Serial.print(x[1], 4); Serial.print('\t');
  Serial.print(x[2], 4); Serial.print('\t');
  Serial.println(x[3], 4);
}

/*
 * ---------------------------------------------------------------------------
 * FLASH INSTRUCTIONS
 * ---------------------------------------------------------------------------
 * Board:   ESP32S3 Dev Module (or ESP32-S3 Supermini)
 * USB:     CDC (Tools > USB Mode > Hardware CDC and JTAG)
 * Library: FastLED (Library Manager)
 *
 * TUNING:
 *   KNOCK_STRENGTH_MIN/MAX  -- random knock range
 *   BOUND                   -- viable range, triggers S2
 *   K_MAX                   -- coupling strength
 *   DIAGONAL_DAMPING        -- self damping, speed of return to green
 *   MIN_RECONFIG_INTERVAL_MS -- speed of S2 hunting
 * ---------------------------------------------------------------------------
 */
