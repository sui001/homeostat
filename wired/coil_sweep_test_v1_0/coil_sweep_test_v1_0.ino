/*
 * PROJECT: Homeostat, wired mock-up
 * DEVICE:  Bench test, one unit's hard drive arm
 *
 * First test of the needle. An ESP32-S3 Supermini drives the voice coil of a
 * salvaged hard drive arm through an Adafruit Motor Shield v2.3 (TB6612). It
 * sweeps the push slowly from one side to the other so you can see the arm
 * follow, and takes serial commands to hold, knock, brake or let it float.
 * No sensing and no homeostat yet: this only proves the arm answers.
 *
 * CONFIGURATION:
 *   MAX_DRIVE_PCT: ceiling on the push. 50 keeps a 10.3 ohm coil near 0.25 A,
 *                  safe on USB power. Raise it once the coil has its own 5 V.
 *   SWEEP_PERIOD_MS: one full there-and-back sweep.
 *
 * WIRING:
 *   Supermini 3V3    -> shield 3v pin (shield logic jumper set to 3v)
 *   Supermini GND    -> shield Gnd
 *   Supermini GPIO12 -> shield SDA
 *   Supermini GPIO13 -> shield SCL
 *   5 V supply       -> shield motor power terminal, 470 uF across it
 *   Voice coil       -> shield M1 (the two outer screws, not the middle GND)
 *
 * SERIAL (115200, send with Enter):
 *   s         sweep (the default)
 *   -100..100 hold that push, as a percent of MAX_DRIVE_PCT
 *   k         knock: a short shove, then brake, like Ashby's hand on the needle
 *   b         brake: coil shorted, the arm resists being moved
 *   r         release: coil open, the arm floats freely
 */

#include <Wire.h>
#include <Adafruit_MotorShield.h>

// ---------- config ----------
#define VERSION "1.0"

const int SDA_PIN = 12;
const int SCL_PIN = 13;
const uint8_t COIL_PORT = 1;           // M1
const uint16_t PWM_FREQ_HZ = 1600;     // the shield's top speed; the coil may hum

const int MAX_DRIVE_PCT = 50;
const unsigned long SWEEP_PERIOD_MS = 6000;
const unsigned long KNOCK_MS = 80;
const int KNOCK_PCT = 100;             // of MAX_DRIVE_PCT
const unsigned long REPORT_EVERY_MS = 200;

// ---------- state ----------
enum ArmMood { SWEEPING, HOLDING, KNOCKED, BRAKED, FLOATING };
ArmMood mood = SWEEPING;

Adafruit_MotorShield shield;
Adafruit_DCMotor *coil = nullptr;

int heldPct = 0;
int currentPct = 0;
unsigned long knockStartedAt = 0;
unsigned long lastReportAt = 0;
char line[16];
uint8_t lineLen = 0;

// Push the arm. pct is -100..100 of MAX_DRIVE_PCT; the sign is the direction.
// Between PWM pulses the TB6612 shorts the coil, so the arm's own motion is
// braked by its back-EMF. That is the damping the liquid used to give.
void urgeArm(int pct) {
  pct = constrain(pct, -100, 100);
  currentPct = pct;
  long duty = (long)abs(pct) * MAX_DRIVE_PCT * 4095 / 10000;
  coil->run(pct >= 0 ? FORWARD : BACKWARD);
  coil->setSpeedFine((uint16_t)duty);
}

// Coil shorted, no push: the arm can still be moved, but it drags.
void brakeArm() {
  currentPct = 0;
  coil->run(FORWARD);
  coil->setSpeedFine(0);
}

// Coil open: nothing resists, the arm drifts where it is pushed.
void releaseArm() {
  currentPct = 0;
  coil->run(RELEASE);
}

int sweepPct(unsigned long now) {
  // triangle wave, -100 -> 100 -> -100
  unsigned long t = now % SWEEP_PERIOD_MS;
  long half = SWEEP_PERIOD_MS / 2;
  long ramp = (t < (unsigned long)half) ? t : SWEEP_PERIOD_MS - t;
  return (int)(ramp * 200 / half) - 100;
}

void obeyCommand(const char *cmd) {
  if (cmd[0] == 's') {
    mood = SWEEPING;
  } else if (cmd[0] == 'k') {
    mood = KNOCKED;
    knockStartedAt = millis();
    urgeArm(currentPct >= 0 ? KNOCK_PCT : -KNOCK_PCT);
  } else if (cmd[0] == 'b') {
    mood = BRAKED;
    brakeArm();
  } else if (cmd[0] == 'r') {
    mood = FLOATING;
    releaseArm();
  } else if (cmd[0] == '-' || isdigit(cmd[0])) {
    heldPct = constrain(atoi(cmd), -100, 100);
    mood = HOLDING;
    urgeArm(heldPct);
  } else {
    Serial.println("? s, k, b, r or a number -100..100");
    return;
  }
  Serial.print("> ");
  Serial.println(cmd);
}

void listenForCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen) {
        line[lineLen] = 0;
        obeyCommand(line);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
    }
  }
}

const char *moodName() {
  switch (mood) {
    case SWEEPING: return "sweeping";
    case HOLDING:  return "holding";
    case KNOCKED:  return "knocked";
    case BRAKED:   return "braked";
    default:       return "floating";
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);  // let USB serial attach
  Serial.printf("\ncoil sweep test v%s\n", VERSION);

  Wire.setPins(SDA_PIN, SCL_PIN);
  while (!shield.begin(PWM_FREQ_HZ)) {
    Serial.println("motor shield not found on I2C (0x60). Check SDA/SCL, 3v, Gnd and the logic jumper.");
    delay(1000);
  }
  coil = shield.getMotor(COIL_PORT);
  Serial.printf("shield found. coil on M%d, max drive %d%%\n", COIL_PORT, MAX_DRIVE_PCT);
  Serial.println("commands: s sweep, -100..100 hold, k knock, b brake, r release");
}

void loop() {
  unsigned long now = millis();
  listenForCommands();

  if (mood == SWEEPING) {
    urgeArm(sweepPct(now));
  } else if (mood == KNOCKED && now - knockStartedAt >= KNOCK_MS) {
    mood = BRAKED;
    brakeArm();
  }

  if (now - lastReportAt >= REPORT_EVERY_MS) {
    lastReportAt = now;
    Serial.printf("%-8s drive %4d%%\n", moodName(), currentPct);
  }
  delay(10);
}

/*
 * FLASHING
 *   Board: ESP32S3 Dev Module (esp32:esp32:esp32s3), USB CDC On Boot: Enabled
 *   Library: Adafruit Motor Shield V2 Library (pulls in Adafruit BusIO)
 *
 *   arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc wired/coil_sweep_test_v1_0
 *   arduino-cli upload  --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc -p COMxx wired/coil_sweep_test_v1_0
 *
 *   If the port does not appear, hold BOOT, tap RESET, release BOOT, then upload.
 *   After uploading, tap RESET to leave boot mode and start the sketch.
 *
 *   One sketch, any unit: there is no DEVICE_ID yet because nothing is shared.
 */
