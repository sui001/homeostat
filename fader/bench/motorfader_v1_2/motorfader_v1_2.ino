/*
 * PROJECT: Motorised fader bring-up
 * DEVICE:  ESP32-S3 SuperMini + L293D (or TB6612FNG) + dual B10K motorised fader
 *
 * Reads the fader's position (10k linear track on an ADC1 pin), drives its
 * little DC motor through an L293D (or TB6612FNG) H-bridge to reach a target, and lets go
 * the moment a finger touches the knob (if the fader has a touch-sense pin).
 * It's the basic "hand and machine share one slider" loop: the machine
 * proposes a position, a person can always overrule it.
 *
 * CONFIGURATION:
 *   DRIVER_L293D    1 = L293D, 0 = TB6612FNG
 *   HAS_TOUCH       1 only if the lever is isolated and you've wired it to GPIO4
 *   motorReversed   flip ('r' over serial) if the motor runs away from the target
 *   KP / MIN_DUTY   tune if it overshoots (lower KP) or stalls short (raise MIN_DUTY)
 *
 * WIRING (fader: use the unprimed gang 1-4; 1'-4' is a spare second track):
 *   Fader pin 3         -> 3V3
 *   Fader pin 1         -> GND
 *   Fader pin 2 (wiper) -> GPIO1  (ADC1_CH0) + 100nF to GND   (pin 4 unused)
 *   Fader lever         -> GPIO4  (TOUCH4)   [only if isolated from frame]
 *
 *   L293D (DIP-16, notch up, pin 1 top-left):
 *     1  EN1,2 -> GPIO5 (PWM)        16 VCC1  -> SuperMini 5V
 *     2  1A    -> GPIO6              8  VCC2  -> motor supply (5-12V)
 *     7  2A    -> GPIO7              4,5,12,13 -> GND (common with motor supply)
 *     3  1Y    -> motor +  (red dot) 6  2Y    -> motor -
 *     channel 3/4 (pins 9-15) unused: tie 9 to GND
 *
 *   or TB6612FNG:
 *   TB6612 PWMA -> GPIO5   AIN1 -> GPIO6   AIN2 -> GPIO7   STBY -> GPIO8
 *   TB6612 VCC  -> 3V3     GND  -> GND (common with motor supply)
 *   TB6612 VM   -> motor supply: 5V (USB) to test, ~9-10V for full speed
 *
 * SERIAL (115200):  a number 0-4095 = go there,  s = sweep demo,
 *                   ? = status,  t = recalibrate touch (hands off!)
 *                   + / - = jog motor full power 300 ms and print positions
 *                   r = toggle motor direction
 */

#define VERSION "1.2"

// ---- pins (avoid GPIO0/2/45/46 on the S3 SuperMini; GPIO48 = onboard RGB) ----
const int PIN_WIPER = 1;   // ADC1 - keeps working alongside WiFi / ESP-NOW
const int PIN_TOUCH = 4;
const int PIN_PWM   = 5;
const int PIN_IN1   = 6;
const int PIN_IN2   = 7;
const int PIN_STBY  = 8;     // TB6612 only

// ---- behaviour ----
#define DRIVER_L293D    1
#define HAS_TOUCH       0     // this fader has no touch pin
bool motorReversed = false;           // 'r' toggles at runtime
const uint32_t PWM_FREQ    = 20000;  // above hearing, so the motor doesn't whine
const uint8_t  PWM_BITS    = 10;     // duty 0-1023
const float    KP          = 1.2f;   // duty per ADC count of error
const int      MIN_DUTY    = 380;    // below this the motor just hums
const int      MAX_DUTY    = 1023;
const int      DEADBAND    = 25;     // ADC counts (~0.6% of travel) = "arrived"
const uint32_t MOVE_TIMEOUT_MS = 1200;  // give up rather than cook the motor
const float    TOUCH_RATIO = 1.3f;   // S3 touch values RISE when touched
const int      REPORT_STEP = 40;     // print when a hand moves it this much

enum FaderState { RESTING, SEEKING, HELD_BY_HAND };
FaderState state = RESTING;

int      target = 0;
uint32_t seekStartedAt = 0;
uint32_t touchBaseline = 0;
int      lastReported = -1000;

// ---------------------------------------------------------------------------

int readPosition() {
  // Average a few samples: the motor's brushes put spikes on the wiper.
  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) sum += analogRead(PIN_WIPER);
  return sum / 8;
}

bool fingerOnKnob() {
#if HAS_TOUCH
  return touchRead(PIN_TOUCH) > touchBaseline * TOUCH_RATIO;
#else
  return false;
#endif
}

void calibrateTouch() {
#if HAS_TOUCH
  uint32_t sum = 0;
  for (int i = 0; i < 16; i++) { sum += touchRead(PIN_TOUCH); delay(5); }
  touchBaseline = sum / 16;
  Serial.printf("touch baseline %lu (touched > %lu)\n",
                (unsigned long)touchBaseline,
                (unsigned long)(touchBaseline * TOUCH_RATIO));
#endif
}

// dir: +1 towards higher readings, -1 lower, 0 = brake
void driveMotor(int dir, int duty) {
  if (motorReversed) dir = -dir;
  if (dir == 0) {                       // short brake: both high, stops crisply
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, HIGH);
    // L293D only brakes while enabled; TB6612 brakes regardless of PWM
    ledcWrite(PIN_PWM, DRIVER_L293D ? MAX_DUTY : 0);
    return;
  }
  digitalWrite(PIN_IN1, dir > 0 ? HIGH : LOW);
  digitalWrite(PIN_IN2, dir > 0 ? LOW : HIGH);
  ledcWrite(PIN_PWM, duty);
}

void releaseMotor() {                   // coast: lets a hand move it freely
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  ledcWrite(PIN_PWM, 0);
}

void proposePosition(int t) {
  target = constrain(t, 0, 4095);
  seekStartedAt = millis();
  state = SEEKING;
  Serial.printf("-> seeking %d\n", target);
}

void seekTarget() {
  int pos = readPosition();
  int err = target - pos;

  if (abs(err) <= DEADBAND) {
    driveMotor(0, 0);
    delay(15);
    releaseMotor();
    state = RESTING;
    lastReported = pos;
    Serial.printf("arrived %d (target %d) in %lu ms\n",
                  pos, target, (unsigned long)(millis() - seekStartedAt));
    return;
  }
  if (millis() - seekStartedAt > MOVE_TIMEOUT_MS) {
    releaseMotor();
    state = RESTING;
    Serial.printf("TIMEOUT at %d (target %d) - motor wires reversed? "
                  "supply too low? raise MIN_DUTY?\n", pos, target);
    return;
  }
  int duty = constrain((int)(abs(err) * KP), MIN_DUTY, MAX_DUTY);
  driveMotor(err > 0 ? 1 : -1, duty);
}

void sweepDemo() {
  const int stops[] = {0, 4095, 2048, 1024, 3072, 0};
  for (int s : stops) {
    proposePosition(s);
    while (state == SEEKING) { seekTarget(); delay(1); }
    delay(400);
  }
}

// Raw test push: full power one way for 300 ms, printing where the slider goes.
void jogMotor(int dir) {
  Serial.printf("jog %c start pos %d\n", dir > 0 ? '+' : '-', readPosition());
  uint32_t t0 = millis();
  driveMotor(dir, MAX_DUTY);
  while (millis() - t0 < 300) {
    delay(50);
    Serial.printf("  %3lu ms pos %d\n", (unsigned long)(millis() - t0), readPosition());
  }
  releaseMotor();
  state = RESTING;
  lastReported = readPosition();
}

void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;
  if (line == "s") sweepDemo();
  else if (line == "t") calibrateTouch();
  else if (line == "+") jogMotor(1);
  else if (line == "-") jogMotor(-1);
  else if (line == "r") {
    motorReversed = !motorReversed;
    Serial.printf("motorReversed = %d\n", motorReversed);
  }
  else if (line == "?")
    Serial.printf("v%s pos %d target %d state %d rev %d touch %lu\n", VERSION,
                  readPosition(), target, state, motorReversed,
#if HAS_TOUCH
                  (unsigned long)touchRead(PIN_TOUCH));
#else
                  0UL);
#endif
  else if (isDigit(line[0])) proposePosition(line.toInt());
}

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(1500);                          // let USB CDC enumerate
  Serial.printf("motorfader v%s (built %s %s)\n", VERSION, __DATE__, __TIME__);

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  if (!DRIVER_L293D) {
    pinMode(PIN_STBY, OUTPUT);
    digitalWrite(PIN_STBY, HIGH);       // wake the TB6612
  }
  ledcAttach(PIN_PWM, PWM_FREQ, PWM_BITS);   // Arduino-ESP32 core 3.x API
  releaseMotor();

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_WIPER, ADC_11db);  // full 0-3.3V range
  calibrateTouch();                     // don't touch the knob during boot

  Serial.printf("position %d. Type 0-4095, s, ?, t\n", readPosition());
}

void loop() {
  handleSerial();

  // A finger always wins: the moment someone takes hold, let go.
  if (fingerOnKnob()) {
    if (state != HELD_BY_HAND) {
      releaseMotor();
      state = HELD_BY_HAND;
      Serial.println("hand on");
    }
  } else if (state == HELD_BY_HAND) {
    state = RESTING;
    Serial.println("hand off");
  }

  if (state == SEEKING) seekTarget();

  // Report hand movements (works without touch too, while resting)
  if (state != SEEKING) {
    int pos = readPosition();
    if (abs(pos - lastReported) >= REPORT_STEP) {
      lastReported = pos;
      Serial.printf("pos %d\n", pos);
    }
  }
  delay(1);
}

/*
 * FLASH (Arduino IDE, esp32 core 3.x):
 *   Board "ESP32S3 Dev Module", USB CDC On Boot: Enabled, upload over USB-C.
 *   Open Serial Monitor at 115200, line ending "Newline".
 *   First line should read "motorfader v1.0 (built ...)".
 *
 * FIRST RUN:
 *   1. Motor supply OFF. Slide by hand: "pos" should go ~0 -> ~4095.
 *      Goes backwards? swap the 3V3/GND track ends (or just accept it).
 *   2. (HAS_TOUCH 1 only) Touch the knob: "hand on"/"hand off".
 *      Always-on? type t with hands off.
 *   3. Motor supply ON, type 2048. If it runs to an end and TIMEOUTs,
 *      set MOTOR_REVERSED 1 (or swap the motor wires).
 *   4. Type s for the sweep. Overshoots/buzzes -> lower KP or widen DEADBAND.
 *      Stops short -> raise MIN_DUTY or the motor voltage (the L293D eats
 *      ~2V, so a 9V supply gives the motor ~7V). L293D warm is normal.
 */
