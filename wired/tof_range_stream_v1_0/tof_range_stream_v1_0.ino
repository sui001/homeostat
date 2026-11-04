/*
 * PROJECT: Homeostat, wired mock-up
 * DEVICE:  Bench test, raw VL53L0X range stream
 *
 * Prints every reading the ToF makes, unsmoothed, about 50 a second, so the
 * sensor and the flag on the arm can be lined up by eye on the live range page
 * (wired/tools/range_view.py). The motor is never touched.
 *
 * OUTPUT, one line per reading:
 *   R,<mm>      a reading
 *   R,-1        nothing seen (out of range or no reflection)
 *
 * WIRING:
 *   VL53L0X: VIN -> 3V3, GND -> GND, SDA -> GPIO12, SCL -> GPIO13
 */

#include <Wire.h>
#include <VL53L0X.h>

// ---------- config ----------
#define VERSION "1.0"

const int SDA_PIN = 12;
const int SCL_PIN = 13;
const uint32_t TOF_BUDGET_US = 20000;   // 20 ms per reading

VL53L0X tof;

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.printf("\ntof range stream v%s\n", VERSION);
  Wire.begin(SDA_PIN, SCL_PIN);
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
  uint16_t mm = tof.readRangeContinuousMillimeters();
  bool blind = tof.timeoutOccurred() || mm < 20 || mm > 2000;
  Serial.printf("R,%d\n", blind ? -1 : mm);
}

/*
 * FLASHING
 *   arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc wired/tof_range_stream_v1_0
 *   arduino-cli upload  --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc -p COMxx wired/tof_range_stream_v1_0
 *   Then: python wired/tools/range_view.py COMxx, and open http://localhost:8765
 */
