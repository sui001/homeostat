/*
 * PROJECT: homeostat-arm: motorised fader as unit TL
 * DEVICE:  ESP32-S3 SuperMini + L293D + dual B10K motorised fader (track 1-4)
 *
 * The fader is TL's needle made physical. The sim (through editor.py, over USB
 * serial) streams TL's deflection and the fader follows it. When a hand takes
 * hold of the fader and pushes against it, the fader stops arguing: it lets go
 * of the motor, reports "held", and the sim holds TL wherever the hand puts it,
 * just as dragging the dial on the page does. When the hand lets go and the
 * slider sits still, the fader goes back to following.
 *
 * VERSION: version.h is written by fader/flash.ps1 (git commit, date, uncommitted
 *   changes). It is the first line on serial and is reported to the editor.
 *
 * WIRING (see the session notes / README):
 *   Fader pin 3 -> 3V3, pin 1 -> GND (own wire to the SuperMini), pin 2 (wiper) -> GPIO1
 *   L293D: 1 EN -> GPIO5, 2 1A -> GPIO6, 7 2A -> GPIO7, 3/6 -> motor,
 *          16 -> SuperMini 5V, 8 -> motor supply (9 V), 4/5/12/13 and 9 -> GND
 *
 * SERIAL PROTOCOL (115200, newline-terminated):
 *   in:  T <deg>    follow TL at this deflection (-50..+50)
 *        V          print version again
 *        kp X | kd X | min N | max N | freq N | speed N   live tuning (lost on reset)
 *   out: {"version": "..."}                         at boot and on V
 *        {"deg":..,"pos":..,"tgt":..,"mode":"follow|rest|held|fault"}   about 25 times a second
 *        anything not starting with '{' is a human note (the bridge logs it)
 */

#include "version.h"   // FW_VERSION, from flash.ps1

// ---- pins (avoid GPIO0/2/45/46 on the S3 SuperMini) ----
const int PIN_WIPER = 1;
const int PIN_PWM   = 5;   // L293D EN1,2
const int PIN_IN1   = 6;   // L293D 1A
const int PIN_IN2   = 7;   // L293D 2A

// ---- the dial: TL's -50..+50 degrees onto the track, clear of the end stops ----
const float DEG_STOP  = 50.0f;
const int   POS_LOW   = 60;     // counts at -50 deg
const int   POS_HIGH  = 4035;   // counts at +50 deg

// ---- motor control (tuned on the bench, 9 V on the L293D, see motorfader v1.9) ----
bool     motorReversed = true;
uint32_t pwmFreq   = 500;      // L293D can creep the motor slowly here; it hums
const uint8_t PWM_BITS = 10;
float    KP        = 1.0f;     // duty per count of error
float    KD        = 2.0f;     // duty per count/ms of speed difference
int      MIN_DUTY  = 380;      // below this the slider doesn't start moving
int      MAX_DUTY  = 850;       // ~83%: at 9 V full power is violent
float    followSpeed = 4.0f;   // counts/ms the setpoint may move: about 1 s end to end
const int   DEADBAND  = 20;    // counts: close enough, stop pushing
const int   WAKE      = 45;    // counts: resting until the target is this far away
const int   COAST_BELOW = 60;  // |u| under this: coast rather than kick

// ---- the hand ----
const int      HAND_ERR   = 160;   // counts (~4 deg) off where it should be...
const int      HAND_GAIN  = 60;    // ...gaining less than this (~1.5 deg) back...
const uint32_t HAND_MS    = 150;   // ...in this long = someone is holding it
const int      STILL_BAND = 25;    // counts: hand has let go when it stays within this...
const uint32_t STILL_MS   = 700;   // ...for this long
const uint32_t REPORT_MS  = 40;    // status line every 40 ms

// ---- guards ----
const uint32_t SETTLE_MS  = 300;   // TL still this long before the fader rests (else it coasts)
const int      STALL_MOVE = 20;    // counts it must move...
const uint32_t STALL_MS   = 800;   // ...in this long while pushing, or it's a fault
const int      RAIL_FAR   = 300;   // reading stuck at 0/4095 with TL this far away...
const uint32_t RAIL_MS    = 400;   // ...for this long = a wire is off
const uint32_t FAULT_MS   = 3000;  // motor off this long after a fault, then try again

enum Mode { FOLLOW, REST, HELD, FAULT };
Mode mode = REST;

float    setpoint = 2048;      // what the motor is steering to right now
int      target   = 2048;      // where TL is
float    speedEst = 0;         // counts/ms, + = towards higher readings
int      lastPos = 0;
uint32_t lastUs = 0, pushSince = 0, stillSince = 0, lastReport = 0;
int      stillRef = 0;
uint32_t moveAt = 0, railSince = 0, faultAt = 0, targetMovedAt = 0;
int      moveRef = 0, handRef = 0;
const int JUMP = 300;          // counts in one reading (~2 ms): no slider moves that fast, so it's a wire...
const int JUMP_KEEP = 10;      // ...if it stays that way for this many readings; else it's noise
int       jumpCount = 0, oddSinceReport = 0;   // odd readings skipped, reported as "odd"

// ---- smoothing TL ----
float     smoothMs  = 100;     // TL's 20 Hz steps glide into each other over about this long ('smooth N')
float     tSmooth   = 2048, tVel = 0;   // the glide, and its speed (counts/ms)
const float STILL_VEL   = 0.02f;  // counts/ms: TL slower than this counts as stopped
int       MIN_MOVING  = 220;   // duty floor once the slider is already moving ('minmove N')
const float MOVING_SPEED = 0.5f;  // counts/ms: "already moving"

// ---------------------------------------------------------------------------

// 16 samples spread over one PWM period (2 ms at 500 Hz): motor current pulsing through shared
// ground shifts the wiper while the drive is on, so a quick burst of samples lands all-on or
// all-off and the reading flips by hundreds of counts. Spread over the period, it averages out.
int readPosition() {
  uint32_t sum = 0;
  const uint32_t gapUs = 1000000UL / pwmFreq / 16;
  for (int i = 0; i < 16; i++) { sum += analogRead(PIN_WIPER); if (i < 15) delayMicroseconds(gapUs); }
  return sum / 16;
}
float posToDeg(float p) { return -DEG_STOP + (p - POS_LOW) * (2 * DEG_STOP) / (POS_HIGH - POS_LOW); }
int   degToPos(float d) {
  d = constrain(d, -DEG_STOP, DEG_STOP);
  return lroundf(POS_LOW + (d + DEG_STOP) * (POS_HIGH - POS_LOW) / (2 * DEG_STOP));
}

void driveMotor(int dir, int duty) {      // dir +1 = towards higher readings, 0 = brake
  if (motorReversed) dir = -dir;
  if (dir == 0) {
    digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, HIGH);
    ledcWrite(PIN_PWM, 1023);             // L293D only brakes while enabled
    return;
  }
  digitalWrite(PIN_IN1, dir > 0 ? HIGH : LOW);
  digitalWrite(PIN_IN2, dir > 0 ? LOW : HIGH);
  ledcWrite(PIN_PWM, duty);
}
void releaseMotor() {                     // coast: a hand can move it freely
  digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
  ledcWrite(PIN_PWM, 0);
}

void report(int pos) {
  const char *m = mode == HELD ? "held" : mode == FOLLOW ? "follow" : mode == FAULT ? "fault" : "rest";
  Serial.printf("{\"deg\":%.2f,\"pos\":%d,\"tgt\":%.2f,\"mode\":\"%s\",\"odd\":%d}\n",
                posToDeg(pos), pos, posToDeg(target), m, oddSinceReport + jumpCount);
  oddSinceReport = 0;
}
void printVersion() { Serial.printf("{\"version\":\"%s\"}\n", FW_VERSION); }

// The hand has let go: carry on from wherever it left the slider.
void handLetGo(int pos) {
  target = pos; setpoint = pos; tSmooth = pos; tVel = 0;
  mode = REST;
  Serial.println("hand off: following again");
}

void handleSerial() {
  static char buf[48]; static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c != '\n') { if (n < sizeof(buf) - 1) buf[n++] = c; continue; }
    buf[n] = 0; n = 0;
    String line(buf); line.trim();
    if (line.startsWith("T ")) {
      int t = degToPos(line.substring(2).toFloat());
      if (mode != HELD) {                     // a held fader ignores the sim
        if (abs(t - target) > 2) targetMovedAt = millis();
        target = t;
      }
    }
    else if (line == "V") printVersion();
    else if (line.startsWith("kp ")) KP = line.substring(3).toFloat();
    else if (line.startsWith("kd ")) KD = line.substring(3).toFloat();
    else if (line.startsWith("min ")) MIN_DUTY = constrain(line.substring(4).toInt(), 0, 1023);
    else if (line.startsWith("max ")) MAX_DUTY = constrain(line.substring(4).toInt(), 0, 1023);
    else if (line.startsWith("speed ")) followSpeed = max(0.1f, line.substring(6).toFloat() / 1000.0f);
    else if (line.startsWith("smooth ")) smoothMs = constrain(line.substring(7).toFloat(), 0.0f, 1000.0f);
    else if (line.startsWith("minmove ")) MIN_MOVING = constrain(line.substring(8).toInt(), 0, 1023);
    else if (line.startsWith("freq ")) {
      pwmFreq = constrain(line.substring(5).toInt(), 100, 40000);
      ledcChangeFrequency(PIN_PWM, pwmFreq, PWM_BITS);
    }
    bool tuned = line.startsWith("kp ") || line.startsWith("kd ") || line.startsWith("min ") ||
                 line.startsWith("max ") || line.startsWith("speed ") || line.startsWith("freq ") ||
                 line.startsWith("smooth ") || line.startsWith("minmove ");
    if (tuned)
      Serial.printf("tuning kp %.2f kd %.2f min %d minmove %d max %d freq %lu speed %.0f smooth %.0f\n",
                    KP, KD, MIN_DUTY, MIN_MOVING, MAX_DUTY, (unsigned long)pwmFreq, followSpeed * 1000, smoothMs);
  }
}

// Guards, so bad wiring or a jam can't make it thrash: a reading stuck at the end of its range while
// TL is far away, or the motor pushing hard without the slider moving, stops the motor (mode "fault")
// for FAULT_MS, says why, then it tries again.
void fault(const char *why, int pos) {
  releaseMotor();
  mode = FAULT; faultAt = millis();
  Serial.printf("FAULT: %s (reading %d, TL at %d). Motor off for %lu s.\n", why, pos, target,
                (unsigned long)(FAULT_MS / 1000));
}

void control() {
  int pos = readPosition();
  uint32_t now = micros();
  float dtMs = (now - lastUs) / 1000.0f;
  if (dtMs < 0.05f) return;
  uint32_t ms = millis();

  // The reading leapt further than any slider can move: a wire making and breaking contact (or just
  // reseated). Believe the new reading as where it is, and don't chase where it used to say it was.
  // One odd reading is noise: skip it and steer on. Only a jump that stays for JUMP_KEEP readings in
  // a row (~20 ms) is a real change, like a wire reseated.
  if (abs(pos - lastPos) > JUMP && dtMs < 50) {
    if (++jumpCount < JUMP_KEEP) return;
    jumpCount = 0;
    releaseMotor();
    Serial.printf("reading jumped %d -> %d and stayed: wiring? Staying put.\n", lastPos, pos);
    target = pos; setpoint = pos; tSmooth = pos; tVel = 0; speedEst = 0;
    lastPos = pos; lastUs = now;
    if (mode != FAULT) mode = REST;
    return;
  }
  oddSinceReport += jumpCount; jumpCount = 0;
  speedEst = 0.7f * speedEst + 0.3f * (pos - lastPos) / dtMs;
  lastPos = pos; lastUs = now;

  if (mode == FAULT) {
    releaseMotor();
    if (ms - faultAt > FAULT_MS) { setpoint = pos; tSmooth = pos; tVel = 0; mode = REST; railSince = 0; Serial.println("trying again"); }
    return;
  }

  if (mode == HELD) {                     // the hand has it: watch for it letting go
    if (abs(pos - stillRef) > STILL_BAND) { stillRef = pos; stillSince = ms; }
    else if (ms - stillSince > STILL_MS) handLetGo(pos);
    return;
  }

  // TL arrives as a 20 Hz staircase. Smooth it into a glide (first order, smoothMs) and take TL's
  // speed from the glide, so the motor steers to where TL is going instead of lurching from step to
  // step. Then glide the setpoint after it, no faster than followSpeed.
  float prevSmooth = tSmooth;
  tSmooth += dtMs / (smoothMs + dtMs) * (target - tSmooth);
  tVel = 0.8f * tVel + 0.2f * (tSmooth - prevSmooth) / dtMs;
  // The glide trails TL by its speed x smoothMs; aim that far ahead along TL's motion to take the lag
  // back out, without bringing the steps back.
  float aim = constrain(tSmooth + tVel * smoothMs, (float)POS_LOW, (float)POS_HIGH);
  if (fabsf(target - tSmooth) < DEADBAND) aim = tSmooth;              // TL has arrived: no leading
  float step = followSpeed * dtMs;
  if (aim > setpoint + step)      setpoint += step;
  else if (aim < setpoint - step) setpoint -= step;
  else                            setpoint = aim;
  float setVel = constrain(tVel, -followSpeed, followSpeed);
  if (fabsf(setVel) < STILL_VEL && fabsf(target - tSmooth) < DEADBAND) setVel = 0;   // TL has stopped

  int err = lroundf(setpoint) - pos;

  if (mode == REST) {
    if (abs(target - pos) <= WAKE && setVel == 0) return;   // close enough: stay quiet
    mode = FOLLOW; pushSince = 0; moveRef = pos; moveAt = ms;
  }

  // reading pinned at the very end of the range while TL is somewhere else: a wire off the fader
  if ((pos <= 2 || pos >= 4093) && abs(target - pos) > RAIL_FAR) {
    if (!railSince) railSince = ms;
    if (ms - railSince > RAIL_MS) { railSince = 0; fault("the wiper reads the end of its range: check fader pins 1-3", pos); return; }
  } else railSince = 0;

  if (setVel == 0 && abs(err) <= DEADBAND) {
    if (ms - targetMovedAt > SETTLE_MS) {               // TL has stopped: brake a moment, then rest
      driveMotor(0, 0); delay(15); releaseMotor();
      mode = REST;
    } else releaseMotor();                             // TL still moving: coast, don't stop-start
    moveRef = pos; moveAt = ms;
    return;
  }

  float u = KP * err - KD * (speedEst - setVel);
  bool pushing = !(setVel != 0 && fabsf(u) < COAST_BELOW);
  // Unsticking a still slider takes MIN_DUTY; keeping a moving one going takes much less, and using
  // the big floor for small corrections on the move is what made it kick back and forth.
  int floorDuty = fabsf(speedEst) > MOVING_SPEED ? MIN_MOVING : MIN_DUTY;
  if (pushing) driveMotor(u > 0 ? 1 : -1, constrain((int)fabsf(u), floorDuty, MAX_DUTY));
  else releaseMotor();

  // pushing without getting anywhere: a jam, the motor supply off, or the motor wires
  if (!pushing || abs(pos - moveRef) > STALL_MOVE) { moveRef = pos; moveAt = ms; }
  else if (ms - moveAt > STALL_MS && abs(err) <= HAND_ERR) { fault("pushing but the slider doesn't move: motor supply, wires or a jam", pos); return; }

  // A hand is on it when it is well off where it should be and makes no real progress back over
  // HAND_MS: the motor is pushing and something is pushing harder. Judged on distance covered, not on
  // the speed estimate (the 500 Hz drive makes that too jittery to stay under a threshold for long).
  // Lagging behind a fast-moving TL doesn't count: then it is covering ground towards the setpoint.
  if (abs(err) > HAND_ERR) {
    int gained = (err > 0 ? 1 : -1) * (pos - handRef);
    if (!pushSince || gained > HAND_GAIN) { pushSince = ms; handRef = pos; }
    else if (ms - pushSince > HAND_MS) {
      releaseMotor();
      mode = HELD; stillRef = pos; stillSince = ms; pushSince = 0;
      Serial.printf("hand on: letting go (%d counts off)\n", abs(err));
    }
  } else pushSince = 0;
}

void setup() {
  Serial.begin(115200);
  delay(1500);                                        // let USB CDC enumerate
  Serial.printf("homeostat_fader %s (built %s %s)\n", FW_VERSION, __DATE__, __TIME__);
  printVersion();

  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  ledcAttach(PIN_PWM, pwmFreq, PWM_BITS);
  releaseMotor();
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_WIPER, ADC_11db);

  int pos = readPosition();
  target = pos; setpoint = pos; tSmooth = pos; lastPos = pos; lastUs = micros();
  mode = REST;                                        // stay put until told otherwise
}

void loop() {
  handleSerial();
  control();
  if (millis() - lastReport >= REPORT_MS) { lastReport = millis(); report(lastPos); }
  delay(1);
}

/*
 * FLASH:  powershell -File fader\flash.ps1            (finds the SuperMini by its USB id)
 *         powershell -File fader\flash.ps1 -Port COM4
 * Writes version.h from git, compiles for "ESP32S3 Dev Module" with USB CDC on
 * boot, uploads, and prints the first serial line so you can see the version.
 * Close anything holding the port first (the editor's bridge lets go when the
 * port vanishes, and picks it up again afterwards).
 */
