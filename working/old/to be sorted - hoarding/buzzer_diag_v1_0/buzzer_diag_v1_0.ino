/*
 * PROJECT: Buzzer Diagnostic
 * VERSION: v1.0.0
 * DEVICE:  ESP32-S3 Supermini
 *
 * Tests active buzzers and bare piezos independently.
 * Watch serial monitor and listen carefully to each test.
 *
 * WIRING:
 *   GPIO 4  -> Active buzzer 1 (S pin)
 *   GPIO 5  -> Active buzzer 2 (S pin)
 *   GPIO 6  -> Bare piezo 1 (+)
 *   GPIO 7  -> Bare piezo 2 (+)
 *   GND     -> all - pins
 *   3.3V    -> all + pins on active buzzers
 */

#define VERSION "v1.0.0"

#define ACT_1   4
#define ACT_2   5
#define PIZ_1   6
#define PIZ_2   7

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("=== BUZZER DIAG v1.0.0 ===");

  pinMode(ACT_1, OUTPUT);
  pinMode(ACT_2, OUTPUT);
  digitalWrite(ACT_1, LOW);
  digitalWrite(ACT_2, LOW);

  ledcAttachChannel(PIZ_1, 440, 8, 0);
  ledcAttachChannel(PIZ_2, 440, 8, 1);
  ledcWrite(0, 0);
  ledcWrite(1, 0);

  delay(500);

  // TEST 1 -- active buzzer 1, simple on/off
  Serial.println("[test 1] active buzzer GPIO4 ON");
  digitalWrite(ACT_1, HIGH);
  delay(1000);
  digitalWrite(ACT_1, LOW);
  Serial.println("[test 1] GPIO4 OFF");
  delay(500);

  // TEST 2 -- active buzzer 2
  Serial.println("[test 2] active buzzer GPIO5 ON");
  digitalWrite(ACT_2, HIGH);
  delay(1000);
  digitalWrite(ACT_2, LOW);
  Serial.println("[test 2] GPIO5 OFF");
  delay(500);

  // TEST 3 -- active buzzer 1, rapid pulse (rhythm test)
  Serial.println("[test 3] GPIO4 rapid pulse");
  for (int i = 0; i < 10; i++) {
    digitalWrite(ACT_1, HIGH); delay(50);
    digitalWrite(ACT_1, LOW);  delay(50);
  }
  delay(500);

  // TEST 4 -- bare piezo 1, sweep frequencies
  Serial.println("[test 4] bare piezo GPIO6 freq sweep 100-1000Hz");
  ledcWrite(0, 128);
  for (int f = 100; f <= 1000; f += 50) {
    ledcChangeFrequency(0, f, 8);
    Serial.print("  freq: "); Serial.println(f);
    delay(150);
  }
  ledcWrite(0, 0);
  delay(500);

  // TEST 5 -- bare piezo 2, sweep frequencies
  Serial.println("[test 5] bare piezo GPIO7 freq sweep 100-1000Hz");
  ledcWrite(1, 128);
  for (int f = 100; f <= 1000; f += 50) {
    ledcChangeFrequency(1, f, 8);
    Serial.print("  freq: "); Serial.println(f);
    delay(150);
  }
  ledcWrite(1, 0);
  delay(500);

  // TEST 6 -- all four together
  Serial.println("[test 6] all four together 3 seconds");
  digitalWrite(ACT_1, HIGH);
  digitalWrite(ACT_2, HIGH);
  ledcChangeFrequency(0, 440, 8); ledcWrite(0, 128);
  ledcChangeFrequency(1, 660, 8); ledcWrite(1, 128);
  delay(3000);
  digitalWrite(ACT_1, LOW);
  digitalWrite(ACT_2, LOW);
  ledcWrite(0, 0);
  ledcWrite(1, 0);

  Serial.println("[done] report what you heard for each test");
}

void loop() {}
