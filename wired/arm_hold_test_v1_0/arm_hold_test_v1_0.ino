/*
 * PROJECT: Homeostat, wired mock-up
 * DEVICE:  Bench test, one unit's hard drive arm with position sensing
 *
 * The voice coil only pushes; it has no idea where the arm is, so any steady
 * push runs the arm into an end stop. Here a VL53L0X watches a flag on the arm
 * and the ESP32 pushes only as hard as the gap to a target asks, braking as it
 * closes in. The arm can then rest anywhere in its swing, which is what a
 * homeostat needle has to do. Still no homeostat logic: this proves the needle.
 *
 * CONFIGURATION:
 *   MAX_DRIVE_PCT: ceiling on the push, 50 is safe on USB power
 *   KP, KD:        how hard it chases the target, how hard it brakes. Live-tunable.
 *   STICTION_PCT:  extra push to break the bearing's grip. Find it with the
 *                  sweep test's hold command: the lowest number that moves it.
 *   Ends, gains and coil direction are saved in flash, so they survive a reset.
 *
 * WIRING:
 *   Shield: 3V3 -> 3v, GND -> Gnd, GPIO12 -> SDA, GPIO13 -> SCL, coil on M1
 *   VL53L0X: VIN -> 3V3, GND -> GND, SDA -> GPIO12, SCL -> GPIO13
 *   Matte flag on the arm tip, 50 to 120 mm from the sensor across the swing
 *
 * SERIAL (115200, send with Enter):
 *   w         watch: coil off, prints the distance. Push the arm by hand.
 *   a         this end of the swing is 0%
 *   z         this end of the swing is 100%
 *   0..100    hold the arm at that percent of its swing
 *   s         slow sweep of the target, 10% to 90% and back
 *   r         release, coil off
 *   f         flip coil direction (if the arm runs away from the target)
 *   kp 1.5    set KP     kd 0.4   set KD     st 15   set STICTION_PCT
 */

#include <Wire.h>
#include <Preferences.h>
#include <VL53L0X.h>
#include <Adafruit_MotorShield.h>

// ---------- config ----------
#define VERSION "1.0"

const int SDA_PIN = 12;
const int SCL_PIN = 13;
const uint8_t COIL_PORT = 1;
const uint16_t PWM_FREQ_HZ = 1600;
const uint32_t TOF_BUDGET_US = 20000;     // 20 ms per reading, about 50 a second

const int MAX_DRIVE_PCT = 50;
const float SMOOTHING = 0.35;             // 0..1, higher follows faster, jitters more
const float DEADBAND_PCT = 1.5;           // close enough: stop pushing
const unsigned long SWEEP_PERIOD_MS = 12000;
const unsigned long REPORT_EVERY_MS = 150;

float KP = 1.5;                           // drive % per % of error
float KD = 0.4;                           // drive % per (% per second) of speed... scaled below
int STICTION_PCT = 10;

// ---------- state ----------
enum NeedleMood { WATCHING, HOLDING, SWEEPING, RESTING };
NeedleMood mood = WATCHING;

Adafruit_MotorShield shield;
Adafruit_DCMotor *coil = nullptr;
VL53L0X tof;
Preferences memory;

float mmA = NAN, mmZ = NAN;               // the two ends of the swing
int coilSign = 1;
float smoothMm = NAN;
float posPct = NAN, lastPosPct = NAN, speedPct = 0;
float targetPct = 50;
int drivePct = 0;
unsigned long lastReadAt = 0, lastReportAt = 0, sweepStartedAt = 0;
char line[24];
uint8_t lineLen = 0;

void urgeArm(int pct) {
  pct = constrain(pct, -100, 100);
  drivePct = pct;
  long duty = (long)abs(pct) * MAX_DRIVE_PCT * 4095 / 10000;
  coil->run(pct * coilSign >= 0 ? FORWARD : BACKWARD);
  coil->setSpeedFine((uint16_t)duty);   // off-time shorts the coil: back-EMF braking
}

void releaseArm() {
  drivePct = 0;
  coil->run(RELEASE);
}

bool calibrated() { return !isnan(mmA) && !isnan(mmZ) && fabs(mmZ - mmA) > 5; }

void rememberSettings() {
  memory.putFloat("mmA", mmA);
  memory.putFloat("mmZ", mmZ);
  memory.putInt("sign", coilSign);
  memory.putFloat("kp", KP);
  memory.putFloat("kd", KD);
  memory.putInt("st", STICTION_PCT);
}

void recallSettings() {
  mmA = memory.getFloat("mmA", NAN);
  mmZ = memory.getFloat("mmZ", NAN);
  coilSign = memory.getInt("sign", 1);
  KP = memory.getFloat("kp", KP);
  KD = memory.getFloat("kd", KD);
  STICTION_PCT = memory.getInt("st", STICTION_PCT);
}

// Where is the needle? Returns false if the sensor saw nothing useful.
bool senseNeedle() {
  uint16_t mm = tof.readRangeContinuousMillimeters();
  if (tof.timeoutOccurred() || mm < 20 || mm > 2000) return false;  // 0 while it wakes, 8190 when it sees nothing
  unsigned long now = millis();
  smoothMm = isnan(smoothMm) ? mm : smoothMm + SMOOTHING * (mm - smoothMm);
  if (calibrated()) {
    posPct = (smoothMm - mmA) * 100.0 / (mmZ - mmA);
    if (!isnan(lastPosPct) && lastReadAt) {
      float dt = (now - lastReadAt) / 1000.0;
      if (dt > 0) speedPct += 0.5 * ((posPct - lastPosPct) / dt - speedPct);
    }
    lastPosPct = posPct;
  }
  lastReadAt = now;
  return true;
}

// Push toward the target, harder the further away, braking on the speed.
// The stiction kick is what stops the old build-up-then-flick.
void chaseTarget() {
  float error = targetPct - posPct;
  if (fabs(error) < DEADBAND_PCT && fabs(speedPct) < 5) {
    urgeArm(0);
    return;
  }
  float push = KP * error - KD * speedPct * 0.1;
  if (push > 0) push += STICTION_PCT;
  else if (push < 0) push -= STICTION_PCT;
  urgeArm((int)push);
}

void obeyCommand(char *cmd) {
  if (!strncmp(cmd, "kp", 2)) { KP = atof(cmd + 2); rememberSettings(); }
  else if (!strncmp(cmd, "kd", 2)) { KD = atof(cmd + 2); rememberSettings(); }
  else if (!strncmp(cmd, "st", 2)) { STICTION_PCT = atoi(cmd + 2); rememberSettings(); }
  else if (cmd[0] == 'w') { mood = WATCHING; releaseArm(); }
  else if (cmd[0] == 'r') { mood = RESTING; releaseArm(); }
  else if (cmd[0] == 'a') { mmA = smoothMm; rememberSettings(); }
  else if (cmd[0] == 'z') { mmZ = smoothMm; rememberSettings(); }
  else if (cmd[0] == 'f') { coilSign = -coilSign; rememberSettings(); }
  else if (cmd[0] == 's' || isdigit(cmd[0])) {
    if (!calibrated()) {
      Serial.println("mark the ends first: w, push to one end, a, push to the other, z");
      return;
    }
    if (cmd[0] == 's') { mood = SWEEPING; sweepStartedAt = millis(); }
    else { mood = HOLDING; targetPct = constrain(atof(cmd), 0, 100); }
  } else {
    Serial.println("? w a z s r f, 0..100, kp N, kd N, st N");
    return;
  }
  Serial.printf("> %s   (ends %.0f..%.0f mm, kp %.2f kd %.2f st %d, sign %d)\n",
                cmd, mmA, mmZ, KP, KD, STICTION_PCT, coilSign);
}

void listenForCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen) { line[lineLen] = 0; obeyCommand(line); lineLen = 0; }
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
    }
  }
}

const char *moodName() {
  switch (mood) {
    case WATCHING: return "watching";
    case HOLDING:  return "holding";
    case SWEEPING: return "sweeping";
    default:       return "resting";
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.printf("\narm hold test v%s\n", VERSION);

  Wire.setPins(SDA_PIN, SCL_PIN);
  while (!shield.begin(PWM_FREQ_HZ)) {
    Serial.println("motor shield not found (0x60)");
    delay(1000);
  }
  coil = shield.getMotor(COIL_PORT);
  releaseArm();

  tof.setBus(&Wire);
  tof.setTimeout(100);
  while (!tof.init()) {
    Serial.println("VL53L0X not found (0x29)");
    delay(1000);
  }
  tof.setMeasurementTimingBudget(TOF_BUDGET_US);
  tof.startContinuous();

  memory.begin("armhold", false);
  recallSettings();
  Serial.println(calibrated() ? "ends remembered. send a number to hold, s to sweep"
                              : "start: push the arm to one end, send a, other end, send z");
}

void loop() {
  listenForCommands();
  bool seen = senseNeedle();
  unsigned long now = millis();

  if (!seen) {
    if (mood == HOLDING || mood == SWEEPING) releaseArm();  // blind: don't push
  } else if (mood == HOLDING || mood == SWEEPING) {
    if (mood == SWEEPING) {
      unsigned long t = (now - sweepStartedAt) % SWEEP_PERIOD_MS;
      float half = SWEEP_PERIOD_MS / 2.0;
      float ramp = t < half ? t / half : (SWEEP_PERIOD_MS - t) / half;
      targetPct = 10 + 80 * ramp;
    }
    chaseTarget();
  }

  if (now - lastReportAt >= REPORT_EVERY_MS) {
    lastReportAt = now;
    if (calibrated())
      Serial.printf("%-8s %5.0f mm  pos %5.1f%%  target %5.1f%%  drive %4d%%\n",
                    moodName(), smoothMm, posPct, targetPct, drivePct);
    else
      Serial.printf("%-8s %5.0f mm\n", moodName(), smoothMm);
  }
}

/*
 * FLASHING
 *   Libraries: Adafruit Motor Shield V2 Library, VL53L0X (Pololu)
 *   arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc wired/arm_hold_test_v1_0
 *   arduino-cli upload  --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc -p COMxx wired/arm_hold_test_v1_0
 *
 * FIRST RUN
 *   1. w, then push the arm by hand end to end: the mm should change smoothly.
 *   2. Push to one end, a. Push to the other end, z.
 *   3. 50. If the arm runs to an end instead of settling, send f.
 *   4. If it hunts back and forth, lower kp or raise kd. If it stops short, raise st.
 *   5. s for a slow sweep under control.
 */
