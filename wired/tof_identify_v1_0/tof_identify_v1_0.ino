/*
 * PROJECT: Homeostat, wired mock-up
 * DEVICE:  Bench test, which ToF chip is this?
 *
 * The "VL53L0/1X V2" board fits either a VL53L0X or a VL53L1X, and both sit at
 * I2C address 0x29. They keep their model ID in different registers, so this
 * asks both ways and says which one answered. It also lists everything on the
 * bus, so the motor shield (0x60, plus 0x70 all-call) should show up too.
 *
 * WIRING:
 *   ToF VIN -> 3V3, GND -> GND, SDA -> GPIO12, SCL -> GPIO13
 *   XSHUT and GPIO1 unconnected
 */

#include <Wire.h>

// ---------- config ----------
#define VERSION "1.0"

const int SDA_PIN = 12;
const int SCL_PIN = 13;
const uint8_t TOF_ADDR = 0x29;

// VL53L0X: 8-bit register 0xC0 reads 0xEE
// VL53L1X: 16-bit register 0x010F reads 0xEA, then 0xCC
int readReg8(uint8_t reg) {
  Wire.beginTransmission(TOF_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(TOF_ADDR, (uint8_t)1) != 1) return -1;
  return Wire.read();
}

int readReg16Addr(uint16_t reg) {
  Wire.beginTransmission(TOF_ADDR);
  Wire.write(reg >> 8);
  Wire.write(reg & 0xFF);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(TOF_ADDR, (uint8_t)1) != 1) return -1;
  return Wire.read();
}

void listBus() {
  Serial.print("on the bus:");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf(" 0x%02X", a);
      found++;
    }
  }
  Serial.println(found ? "" : " nothing");
}

void nameTheChip() {
  int l1a = readReg16Addr(0x010F);
  int l1b = readReg16Addr(0x0110);
  int l0 = readReg8(0xC0);
  Serial.printf("L1X id 0x%02X%02X, L0X id 0x%02X -> ", l1a & 0xFF, l1b & 0xFF, l0 & 0xFF);
  if (l1a == 0xEA && l1b == 0xCC) Serial.println("VL53L1X");
  else if (l0 == 0xEE) Serial.println("VL53L0X");
  else if (l1a < 0 && l0 < 0) Serial.println("no answer at 0x29, check the wiring");
  else Serial.println("unrecognised");
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.printf("\ntof identify v%s\n", VERSION);
  Wire.begin(SDA_PIN, SCL_PIN);
}

void loop() {
  listBus();
  nameTheChip();
  delay(2000);
}

/*
 * FLASHING
 *   arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc wired/tof_identify_v1_0
 *   arduino-cli upload  --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc -p COMxx wired/tof_identify_v1_0
 *   Reflash coil_sweep_test afterwards to get the arm moving again.
 */
