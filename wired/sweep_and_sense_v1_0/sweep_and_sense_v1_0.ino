/*
 * PROJECT: Homeostat, wired mock-up
 * DEVICE:  Bench test, coil sweep with the ToF watching
 *
 * Sweeps the voice coil open loop, like coil_sweep_test, while the VL53L0X
 * reads the arm, and prints both on every reading. Plotted together on the
 * range page they show how the arm answers a push: how much drive it takes to
 * unstick, how far it flicks, and whether it returns. The evidence for tuning
 * the closed-loop needle.
 *
 * OUTPUT, one line per ToF reading (about 50 a second):
 *   R,<mm>,<drive%>     mm is -1 when nothing is seen
 *
 * SERIAL (send with Enter): s sweep, r release, -100..100 hold
 *
 * WIRING:
 *   Shield: 3V3 -> 3v, GND -> Gnd, GPIO12 -> SDA, GPIO13 -> SCL, coil on M1
 *   VL53L0X: VIN -> 3V3, GND -> GND, SDA -> GPIO12, SCL -> GPIO13
 */

#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_MotorShield.h>

// ---------- config ----------
#define VERSION "1.0"

const int SDA_PIN = 12;
const int SCL_PIN = 13;
const uint8_t COIL_PORT = 1;
const uint16_t PWM_FREQ_HZ = 1600;
const uint32_t TOF_BUDGET_US = 20000;
const int MAX_DRIVE_PCT = 50;
const unsigned long SWEEP_PERIOD_MS = 6000;

// ---------- state ----------
enum ArmMood { SWEEPING, HOLDING, FLOATING };
ArmMood mood = SWEEPING;

Adafruit_MotorShield shield;
Adafruit_DCMotor *coil = nullptr;
VL53L0X tof;
int drivePct = 0;
char line[16];
uint8_t lineLen = 0;

void urgeArm(int pct) {
  pct = constrain(pct, -100, 100);
  drivePct = pct;
  long duty = (long)abs(pct) * MAX_DRIVE_PCT * 4095 / 10000;
  coil->run(pct >= 0 ? FORWARD : BACKWARD);
  coil->setSpeedFine((uint16_t)duty);
}

void releaseArm() {
  drivePct = 0;
  coil->run(RELEASE);
}

int sweepPct(unsigned long now) {
  unsigned long t = now % SWEEP_PERIOD_MS;
  long half = SWEEP_PERIOD_MS / 2;
  long ramp = (t < (unsigned long)half) ? t : SWEEP_PERIOD_MS - t;
  return (int)(ramp * 200 / half) - 100;
}

void listenForCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') {
      if (lineLen < sizeof(line) - 1) line[lineLen++] = c;
      continue;
    }
    if (!lineLen) continue;
    line[lineLen] = 0;
    lineLen = 0;
    if (line[0] == 's') mood = SWEEPING;
    else if (line[0] == 'r') { mood = FLOATING; releaseArm(); }
    else if (line[0] == '-' || isdigit(line[0])) { mood = HOLDING; urgeArm(atoi(line)); }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.printf("\nsweep and sense v%s\n", VERSION);

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
}

void loop() {
  listenForCommands();
  if (mood == SWEEPING) urgeArm(sweepPct(millis()));

  uint16_t mm = tof.readRangeContinuousMillimeters();   // paces the loop at ~50 Hz
  bool blind = tof.timeoutOccurred() || mm < 20 || mm > 2000;
  Serial.printf("R,%d,%d\n", blind ? -1 : mm, drivePct);
}

/*
 * FLASHING
 *   arduino-cli upload --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc -p COMxx wired/sweep_and_sense_v1_0
 *   Then: python wired/tools/range_view.py COMxx --log sweep.csv, open http://localhost:8765
 */
