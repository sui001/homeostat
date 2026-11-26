/*
 * PROJECT: Homeostat pot node, bring-up
 * DEVICE:  ESP32-DevKitC-32E + Adafruit DRV8833 + Alps 16 mm motorised pot (100k)
 *
 * First light for one node. Reads the pot (the unit's needle and the hand's way in) and lets
 * the motor be nudged by hand command, so the wiring can be checked before any homeostat runs.
 * Motor is OFF at boot and only moves when told; every move has a time limit.
 *
 * CONFIGURATION:
 *   POS_LOW / POS_HIGH   counts at -50 / +50 deg (set live with 'lo' / 'hi', lost on reset)
 *   motorReversed        flip with 'r' if + jog makes the counts go DOWN
 *
 * WIRING (see node/img/node_wiring.svg):
 *   pot track ends -> 3V3 / GND (own GND pin), wiper -> GPIO34 + 100 nF to pot GND
 *   DRV8833 AIN1 -> GPIO25, AIN2 -> GPIO26, SLP -> GPIO27, FLT -> GPIO35 (+10k to 3V3)
 *   DRV8833 AOUT1/AOUT2 -> pot motor, VM/GND -> motor supply, GND common with ESP32
 *
 * SERIAL (115200):
 *   V            version
 *   ?            one status line now
 *   q            stream on/off (every 100 ms: raw, mV, deg, flt, motor)
 *   + / -        jog 150 ms at jog duty, prints counts before/after
 *   j N          jog duty 0-100 % (default 35)
 *   u N          speed test: run N % for 1 s towards the middle, print deg/s
 *   h D          hold at D degrees (-50..50) for 3 s, then coast
 *   s            stop (coast)      b  brake
 *   r            reverse motor direction
 *   lo / hi      take the current reading as -50 / +50 deg  (lo N / hi N: set the count)
 *   sleep / wake DRV8833 SLP low / high
 */

#define VERSION "1.3"
#if __has_include("version.h")
#include "version.h"            // written by node/flash.ps1 from git
#endif
#ifndef FW_VERSION
#define FW_VERSION "unversioned build"
#endif

// ---- pins ----
const int PIN_WIPER = 34;   // ADC1_CH6, input only: keeps working when the radio is on
const int PIN_AIN1  = 25;
const int PIN_AIN2  = 26;
const int PIN_SLP   = 27;
const int PIN_FLT   = 35;   // input only, external 10k pull-up; low = DRV8833 fault

// ---- motor ----
const uint32_t PWM_FREQ = 20000;   // above hearing
const uint8_t  PWM_BITS = 10;      // duty 0-1023
const int      FULL     = (1 << PWM_BITS) - 1;
int  jogPercent = 35;
bool motorReversed = true;          // v1.3: measured on the bench, + duty = counts up (towards CCW)
const uint32_t JOG_MS  = 150;
const uint32_t HOLD_MS = 8000;     // RK168 is slow by design: ~12 s per 300 deg
float KP = 1.5f;                   // duty per count of error while holding
int   MIN_DUTY = 500, MAX_DUTY = 1023; // 4.5 V motor on its own 5 V: full duty is in spec
const int DEADBAND = 15;           // counts (~1 deg)

// ---- the needle's window: 100 deg of the pot's ~300 ----
// v1.2: 3V3 on pot pin 2 (behind ~88k), GND on pin 3: wiper runs 0..~1.67 V, raw 0..~1925 over 300 deg.
// CCW = high counts. -50 deg is the CCW side. ~7 counts a degree; set from a hand sweep, refine with marks.
int POS_LOW  = 1224;               // -50 deg (CCW side)
int POS_HIGH = 522;                // +50 deg (CW side)
const int WINDOW_SLACK = 150;      // counts beyond the window the motor may still push outward

enum MotorMode { COAST, BRAKE, JOG, HOLD, FAULT };
MotorMode mode = COAST;
const char *modeName[] = {"coast", "brake", "jog", "hold", "fault"};

bool stream = true;
int holdTarget = 0;
uint32_t modeUntil = 0, lastReport = 0;
int lastDuty = 0;
String line;

// ---- reading the needle ----
int readRaw() {                    // 16 samples averaged: the 100k track is a soft source
  long sum = 0;
  for (int i = 0; i < 16; i++) sum += analogRead(PIN_WIPER);
  return sum / 16;
}

float rawToDeg(int raw) {
  return -50.0f + 100.0f * (raw - POS_LOW) / float(POS_HIGH - POS_LOW);
}

int degToRaw(float d) {
  return POS_LOW + int((d + 50.0f) / 100.0f * (POS_HIGH - POS_LOW));
}

// ---- driving it ----
// signed duty: + = forward. Slow decay (one input held high, the other PWM'd inverted):
// gentler at low speed than fast decay, which suits a small geared pot motor.
void drive(int duty) {
  duty = constrain(duty, -FULL, FULL);
  if (motorReversed) duty = -duty;
  lastDuty = duty;
  if (duty > 0) {
    ledcWrite(PIN_AIN1, FULL);
    ledcWrite(PIN_AIN2, FULL - duty);
  } else if (duty < 0) {
    ledcWrite(PIN_AIN1, FULL + duty);
    ledcWrite(PIN_AIN2, FULL);
  } else {
    ledcWrite(PIN_AIN1, 0);
    ledcWrite(PIN_AIN2, 0);
  }
}

void coast() { drive(0); mode = COAST; }
void brake() { ledcWrite(PIN_AIN1, FULL); ledcWrite(PIN_AIN2, FULL); lastDuty = 0; mode = BRAKE; }

// would this duty push the needle further out of its window?
bool pushingOut(int raw, int duty) {
  int lo = min(POS_LOW, POS_HIGH) - WINDOW_SLACK;
  int hi = max(POS_LOW, POS_HIGH) + WINDOW_SLACK;
  // duty > 0 is assumed to raise counts (that is what 'r' is for)
  return (raw > hi && duty > 0) || (raw < lo && duty < 0);
}

void report(const char *tag) {
  int raw = readRaw();
  Serial.printf("%s raw %4d  mV %4d  deg %6.1f  flt %s  motor %s %d\n", tag, raw,
                analogReadMilliVolts(PIN_WIPER), rawToDeg(raw),
                digitalRead(PIN_FLT) ? "ok" : "LOW", modeName[mode], lastDuty);
}

void jog(int sign) {
  int before = readRaw();
  if (pushingOut(before, sign)) { Serial.println("jog refused: already past the window that way"); return; }
  mode = JOG;
  drive(sign * jogPercent * FULL / 100);
  uint32_t t = millis();
  while (millis() - t < JOG_MS) {
    if (!digitalRead(PIN_FLT)) break;
    delay(2);
  }
  coast();
  delay(60);
  int after = readRaw();
  Serial.printf("jog %c %d%%: %d -> %d (%+d counts)%s\n", sign > 0 ? '+' : '-', jogPercent,
                before, after, after - before,
                abs(after - before) < 10 ? "  [didn't move: more duty, or check motor wiring/supply]" : "");
}

// how fast does the needle actually go? run at a fixed duty for 1 s, heading for the middle
void speedTest(int pct) {
  int before = readRaw();
  int mid = (POS_LOW + POS_HIGH) / 2;
  int sign = before > mid ? -1 : +1;
  mode = JOG;
  drive(sign * pct * FULL / 100);
  uint32_t t = millis();
  while (millis() - t < 1000) {
    int raw = readRaw();
    if (!digitalRead(PIN_FLT) || pushingOut(raw, sign)) break;
    delay(2);
  }
  uint32_t ms = millis() - t;
  coast();
  delay(80);
  int after = readRaw();
  float dps = (rawToDeg(after) - rawToDeg(before)) * 1000.0f / ms;
  Serial.printf("speed %d%%: %.1f -> %.1f deg in %lu ms = %.1f deg/s\n", pct,
                rawToDeg(before), rawToDeg(after), ms, dps);
}

void handleCommand(String c) {
  c.trim();
  if (c == "V") Serial.printf("node_v%s %s\n", VERSION, FW_VERSION);
  else if (c == "?") report("status");
  else if (c == "q") { stream = !stream; Serial.printf("stream %s\n", stream ? "on" : "off"); }
  else if (c == "+") jog(+1);
  else if (c == "-") jog(-1);
  else if (c.startsWith("j ")) { jogPercent = constrain(c.substring(2).toInt(), 0, 100); Serial.printf("jog duty %d%%\n", jogPercent); }
  else if (c.startsWith("h ")) {
    float d = constrain(c.substring(2).toFloat(), -50.0f, 50.0f);
    holdTarget = degToRaw(d); mode = HOLD; modeUntil = millis() + HOLD_MS;
    Serial.printf("hold %.1f deg (raw %d) for %lu ms\n", d, holdTarget, HOLD_MS);
  }
  else if (c.startsWith("u ")) speedTest(constrain(c.substring(2).toInt(), 0, 100));
  else if (c == "s") { coast(); Serial.println("coast"); }
  else if (c == "b") { brake(); Serial.println("brake"); }
  else if (c == "r") { motorReversed = !motorReversed; coast(); Serial.printf("motorReversed %d\n", motorReversed); }
  else if (c == "lo" || c.startsWith("lo ")) { POS_LOW = c.length() > 3 ? c.substring(3).toInt() : readRaw(); Serial.printf("POS_LOW = %d (-50 deg)\n", POS_LOW); }
  else if (c == "hi" || c.startsWith("hi ")) { POS_HIGH = c.length() > 3 ? c.substring(3).toInt() : readRaw(); Serial.printf("POS_HIGH = %d (+50 deg)\n", POS_HIGH); }
  else if (c == "sleep") { coast(); digitalWrite(PIN_SLP, LOW); Serial.println("driver asleep"); }
  else if (c == "wake") { digitalWrite(PIN_SLP, HIGH); Serial.println("driver awake"); }
  else if (c.length()) Serial.printf("? unknown '%s'\n", c.c_str());
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.printf("node_v%s %s\n", VERSION, FW_VERSION);   // first line: which code is this

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_WIPER, ADC_11db);          // 0..~3.1 V
  pinMode(PIN_FLT, INPUT);
  pinMode(PIN_SLP, OUTPUT);
  digitalWrite(PIN_SLP, HIGH);
  ledcAttach(PIN_AIN1, PWM_FREQ, PWM_BITS);
  ledcAttach(PIN_AIN2, PWM_FREQ, PWM_BITS);
  coast();

  Serial.println("motor off. turn the knob by hand and watch raw; '+' / '-' to jog. '?' for status");
  report("boot");
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { if (line.length()) handleCommand(line); line = ""; }
    else line += ch;
  }

  if (!digitalRead(PIN_FLT) && mode != FAULT && mode != COAST) {
    coast(); mode = FAULT;
    Serial.println("FAULT: DRV8833 FLT low (over-current or over-temperature). motor off");
  }

  if (mode == HOLD) {
    int raw = readRaw();
    int err = holdTarget - raw;
    if (millis() > modeUntil) { coast(); Serial.printf("hold done: target %d, at %d (%+d)\n", holdTarget, raw, -err); }
    else if (abs(err) < DEADBAND) drive(0);
    else {
      int duty = constrain(int(KP * err), -MAX_DUTY, MAX_DUTY);
      if (abs(duty) < MIN_DUTY) duty = duty > 0 ? MIN_DUTY : -MIN_DUTY;
      if (pushingOut(raw, duty)) { coast(); Serial.println("hold stopped: driving out of the window (try 'r')"); }
      else drive(duty);
    }
  }

  if (stream && millis() - lastReport >= 100) { lastReport = millis(); report("pot"); }
  delay(2);
}

/*
 * FLASH:  powershell -File node\flash.ps1 -Sketch bench\node_v1_2
 *   board "ESP32 Dev Module" (esp32:esp32:esp32), DevKitC shows as a CP210x COM port.
 *   If upload stalls at "Connecting...", hold BOOT until it starts writing.
 */
