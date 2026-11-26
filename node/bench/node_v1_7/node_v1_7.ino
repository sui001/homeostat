/*
 * PROJECT: Homeostat pot node, bring-up
 * DEVICE:  ESP32-DevKitC-32E + Adafruit DRV8833 + Alps 16 mm motorised pot (100k)
 *
 * v1.7: the hand. In 'follow' the motor holds the needle where it is told, with no time limit.
 * Take the knob and push against it: the motor goes slack and the hand has the needle. Leave
 * the knob still for STILL_MS and the motor takes it back, holding it where the hand left it.
 * The hand outranks the machine: it proposes, a person can overrule. Judged by displacement
 * (no touch track), as on the TL fader: off by more than HAND_ERR and not winning back
 * HAND_GAIN within HAND_MS while driving.
 * Also: a wiper jump filter (a reading the knob couldn't have made is ignored unless it stays)
 * and a fault when the wiper reads where this wiring can't go (a lost wiper reads ~4095).
 *
 * Motor is OFF at boot and only moves when told.
 *
 * CONFIGURATION:
 *   POS_LOW / POS_ZERO / POS_HIGH   counts at the -50 / 0 / +50 deg marks ('lo' / 'zero' / 'hi')
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
 *   f D          follow: hold D degrees (-50..50) with no time limit; the hand can take over
 *   f            follow from where the needle is now
 *   he N / hg N / hm N / still N   hand: error counts, win-back counts, window ms, release ms
 *   h D          hold at D degrees (-50..50) for 8 s, then coast (no hand detection)
 *   s            stop (coast)      b  brake
 *   r            reverse motor direction
 *   kp X / min N / max N   hold tuning: duty per count of error, duty floor, duty cap (0-1023)
 *   db N         hold deadband in counts (~5.5 counts a degree)
 *   lo / hi      take the current reading as -50 / +50 deg  (lo N / hi N: set the count)
 *   sleep / wake DRV8833 SLP low / high
 */

#define VERSION "1.7"
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
float KP = 4.0f;                   // duty per count of error while holding (v1.5: 1.5 was slow on the
                                   // real scale, ~5.5 counts a degree: most moves sat at MIN_DUTY)
int   MIN_DUTY = 500, MAX_DUTY = 1023; // 4.5 V motor on its own 5 V: full duty is in spec
int DEADBAND = 6;                  // counts: ~1 deg on the real scale (v1.6; 15 stopped ~2.7 deg short)

// ---- the needle's window: 100 deg of the pot's ~300 ----
// v1.2: 3V3 on pot pin 2 (behind ~88k), GND on pin 3: wiper runs 0..~1.67 V, raw 0..~1925 over 300 deg.
// CCW = high counts. -50 deg is the CCW side. ~7 counts a degree; set from a hand sweep, refine with marks.
// v1.4: read off the marks on the faceplate, knob resting, 2 s each (+-2 counts).
// The halves differ (260 vs 290 counts), so two straight lines meet at 0: every mark exact.
int POS_LOW  = 1779;               // -50 deg mark (CCW side)
int POS_ZERO = 1519;               //   0 deg mark
int POS_HIGH = 1229;               // +50 deg mark (CW side)
const int WINDOW_SLACK = 150;      // counts beyond the window the motor may still push outward

// ---- the hand (v1.7) ----
// ~5.5 counts a degree. The motor is slow (~28 deg/s flat out, a few deg/s near the duty floor),
// so the test is "not winning back", not "losing".
int HAND_ERR  = 22;                // counts (~4 deg) off target before a hand is suspected
int HAND_GAIN = 3;                 // counts the motor must win back per window to clear suspicion
uint32_t HAND_MS  = 250;           // the window
uint32_t STILL_MS = 700;           // knob still this long under a hand = let go
const int STILL_BAND = 6;          // counts (~1 deg): "still", above the +-2 count noise

// ---- the wiper's honesty ----
const int WIPER_MAX = 2300;        // this wiring tops out ~1925 (1.65 V); above = wiper lost
const int JUMP      = 120;         // counts (~22 deg) in one reading: not a knob, a contact
const int JUMP_KEEP = 8;           // ...unless it stays this many readings, then believe it
const uint32_t LOST_MS = 300;      // wiper out of range this long while driving = fault

enum MotorMode { COAST, BRAKE, JOG, HOLD, FOLLOW, HELD, FAULT };
MotorMode mode = COAST;
const char *modeName[] = {"coast", "brake", "jog", "hold", "follow", "held", "fault"};

bool stream = true;
int holdTarget = 0;
uint32_t modeUntil = 0, lastReport = 0;
int lastDuty = 0;
String line;

int pos = -1;                      // the needle, as believed (filtered)
int jumpRun = 0, jumpRef = 0;      // a run of readings far from pos, and where it is
uint32_t lostSince = 0;
uint32_t winStart = 0; int winErr = 0;                           // hand window
uint32_t driveSince = 0;           // when the push started: a motor starting from rest is slow,
                                   // so the first HAND_MS of any push is never judged
uint32_t stillSince = 0; int stillRef = 0;                       // held: is the knob still?

// ---- reading the needle ----
int readRaw() {                    // 16 samples averaged: the 100k track is a soft source
  long sum = 0;
  for (int i = 0; i < 16; i++) sum += analogRead(PIN_WIPER);
  return sum / 16;
}

// one reading through the filter. Returns false while the wiper reads where it can't be.
bool readNeedle() {
  int raw = readRaw();
  if (raw > WIPER_MAX) { if (!lostSince) lostSince = millis(); return false; }
  lostSince = 0;
  if (pos < 0) { pos = raw; return true; }
  if (abs(raw - pos) <= JUMP) { pos = raw; jumpRun = 0; return true; }
  // a jump: believe it only if it stays put
  if (jumpRun > 0 && abs(raw - jumpRef) <= JUMP) jumpRun++;
  else { jumpRun = 1; jumpRef = raw; }
  if (jumpRun >= JUMP_KEEP) {
    Serial.printf("needle re-anchored %d -> %d (a real move, or a contact that stayed)\n", pos, raw);
    pos = raw; jumpRun = 0;
  }
  return true;
}

float rawToDeg(int raw) {          // two lines, -50..0 and 0..+50, meeting at the 0 mark
  bool lowSide = (POS_LOW > POS_ZERO) ? raw >= POS_ZERO : raw <= POS_ZERO;
  if (lowSide) return -50.0f * (raw - POS_ZERO) / float(POS_LOW - POS_ZERO);
  return 50.0f * (raw - POS_ZERO) / float(POS_HIGH - POS_ZERO);
}

int degToRaw(float d) {
  if (d <= 0) return POS_ZERO + int(-d / 50.0f * (POS_LOW - POS_ZERO));
  return POS_ZERO + int(d / 50.0f * (POS_HIGH - POS_ZERO));
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
  bool tracking = (mode == HOLD || mode == FOLLOW);
  Serial.printf("%s raw %4d  mV %4d  deg %6.1f  tgt %6.1f  flt %s  motor %s %d\n", tag, raw,
                analogReadMilliVolts(PIN_WIPER), rawToDeg(pos >= 0 ? pos : raw),
                tracking ? rawToDeg(holdTarget) : NAN,
                digitalRead(PIN_FLT) ? "ok" : "LOW", modeName[mode], lastDuty);
}

// the motor's push towards a target: P with a floor, kept inside the window
int pushTowards(int target, int raw) {
  int err = target - raw;
  if (abs(err) < DEADBAND) return 0;
  int duty = constrain(int(KP * err), -MAX_DUTY, MAX_DUTY);
  if (abs(duty) < MIN_DUTY) duty = duty > 0 ? MIN_DUTY : -MIN_DUTY;
  if (pushingOut(raw, duty)) return 0;
  return duty;
}

void startFollow(int target) {
  // a target outside the needle's +-50 deg is brought to the nearest edge
  int lo = min(POS_LOW, POS_HIGH), hi = max(POS_LOW, POS_HIGH);
  holdTarget = constrain(target, lo, hi);
  mode = FOLLOW;
  winStart = 0; driveSince = 0;
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
  int mid = POS_ZERO;
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
  else if (c == "f") { startFollow(pos); Serial.printf("follow %.1f deg (here)\n", rawToDeg(holdTarget)); }
  else if (c.startsWith("f ")) {
    float d = constrain(c.substring(2).toFloat(), -50.0f, 50.0f);
    startFollow(degToRaw(d));
    Serial.printf("follow %.1f deg (raw %d)\n", d, holdTarget);
  }
  else if (c.startsWith("he ")) { HAND_ERR = c.substring(3).toInt(); Serial.printf("HAND_ERR %d\n", HAND_ERR); }
  else if (c.startsWith("hg ")) { HAND_GAIN = c.substring(3).toInt(); Serial.printf("HAND_GAIN %d\n", HAND_GAIN); }
  else if (c.startsWith("hm ")) { HAND_MS = c.substring(3).toInt(); Serial.printf("HAND_MS %lu\n", HAND_MS); }
  else if (c.startsWith("still ")) { STILL_MS = c.substring(6).toInt(); Serial.printf("STILL_MS %lu\n", STILL_MS); }
  else if (c.startsWith("u ")) speedTest(constrain(c.substring(2).toInt(), 0, 100));
  else if (c.startsWith("db ")) { DEADBAND = max(1, (int)c.substring(3).toInt()); Serial.printf("DEADBAND %d\n", DEADBAND); }
  else if (c.startsWith("kp ")) { KP = c.substring(3).toFloat(); Serial.printf("KP %.2f\n", KP); }
  else if (c.startsWith("min ")) { MIN_DUTY = constrain(c.substring(4).toInt(), 0, FULL); Serial.printf("MIN_DUTY %d\n", MIN_DUTY); }
  else if (c.startsWith("max ")) { MAX_DUTY = constrain(c.substring(4).toInt(), 0, FULL); Serial.printf("MAX_DUTY %d\n", MAX_DUTY); }
  else if (c == "s") { coast(); Serial.println("coast"); }
  else if (c == "b") { brake(); Serial.println("brake"); }
  else if (c == "r") { motorReversed = !motorReversed; coast(); Serial.printf("motorReversed %d\n", motorReversed); }
  else if (c == "lo" || c.startsWith("lo ")) { POS_LOW = c.length() > 3 ? c.substring(3).toInt() : readRaw(); Serial.printf("POS_LOW = %d (-50 deg)\n", POS_LOW); }
  else if (c == "zero" || c.startsWith("zero ")) { POS_ZERO = c.length() > 5 ? c.substring(5).toInt() : readRaw(); Serial.printf("POS_ZERO = %d (0 deg)\n", POS_ZERO); }
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

  Serial.println("motor off. 'f 0' to follow 0 deg; push the knob and the hand takes over. '?' for status");
  report("boot");
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { if (line.length()) handleCommand(line); line = ""; }
    else line += ch;
  }

  if (!digitalRead(PIN_FLT) && mode != FAULT && mode != COAST && mode != HELD) {
    coast(); mode = FAULT;
    Serial.println("FAULT: DRV8833 FLT low (over-current or over-temperature). motor off");
  }

  bool good = readNeedle();
  uint32_t now = millis();

  // a lost wiper: never drive on a reading this wiring can't make
  if (!good && (mode == HOLD || mode == FOLLOW) && now - lostSince > LOST_MS) {
    coast(); mode = FAULT;
    Serial.println("FAULT: wiper reads out of range (lost contact on pot pin 4 / GPIO34?). motor off");
  }

  if (mode == HOLD && good) {
    int err = holdTarget - pos;
    if (now > modeUntil) { coast(); Serial.printf("hold done: target %d, at %d (%+d)\n", holdTarget, pos, -err); }
    else drive(pushTowards(holdTarget, pos));
  }

  if (mode == FOLLOW && good) {
    int err = abs(holdTarget - pos);
    int duty = pushTowards(holdTarget, pos);
    drive(duty);
    // the hand: off by more than HAND_ERR, driving the whole window, and not winning back
    if (duty == 0) driveSince = 0;
    else if (driveSince == 0) driveSince = now;
    if (duty == 0 || err <= HAND_ERR || now - driveSince < HAND_MS) winStart = 0;
    else if (winStart == 0) { winStart = now; winErr = err; }
    else if (now - winStart >= HAND_MS) {
      if (winErr - err < HAND_GAIN) {
        coast(); mode = HELD;
        stillRef = pos; stillSince = now;
        Serial.printf("hand on at %.1f deg (%.1f off, won back %d counts in %lu ms). motor slack\n",
                      rawToDeg(pos), rawToDeg(pos) - rawToDeg(holdTarget), winErr - err, HAND_MS);
      } else { winStart = now; winErr = err; }
    }
  }

  if (mode == HELD && good) {
    if (abs(pos - stillRef) > STILL_BAND) { stillRef = pos; stillSince = now; }
    else if (now - stillSince >= STILL_MS) {
      startFollow(pos);
      Serial.printf("hand off at %.1f deg: holding there\n", rawToDeg(holdTarget));
    }
  }

  if (stream && millis() - lastReport >= 100) { lastReport = millis(); report("pot"); }
  delay(2);
}

/*
 * FLASH:  powershell -File node\flash.ps1 -Sketch bench\node_v1_7
 *   board "ESP32 Dev Module" (esp32:esp32:esp32), DevKitC shows as a CP210x COM port.
 *   If upload stalls at "Connecting...", hold BOOT until it starts writing.
 */
