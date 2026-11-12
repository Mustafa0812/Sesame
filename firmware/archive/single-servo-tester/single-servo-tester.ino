/*
 * Single servo tester - ESP32-S3 SuperMini
 *
 * Sweeps ONE servo back and forth forever. No serial commands.
 * Wiring: servo signal -> SERVO_PIN, servo V+ -> external 5-6 V supply,
 *         servo GND -> supply GND AND ESP32 GND (common ground).
 *
 * Library: ESP32Servo
 */

#include <ESP32Servo.h>

const int SERVO_PIN = 12;

// Pulse range used by the main firmware. If the servo buzzes at the
// extremes, narrow this to 500-2400.
const int MIN_US = 732;
const int MAX_US = 2929;

// Sweep limits and speed (ms per degree).
const int MIN_ANGLE = 0;
const int MAX_ANGLE = 180;
const int STEP_MS = 15;
const int PAUSE_MS = 500;  // pause at each end

Servo servo;

void sweep(int from, int to) {
  int dir = (to > from) ? 1 : -1;
  for (int a = from; a != to + dir; a += dir) {
    servo.write(a);
    delay(STEP_MS);
  }
  Serial.printf("Reached %d deg\n", to);
  delay(PAUSE_MS);
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.printf("Single servo tester: GPIO %d, %d-%d us\n", SERVO_PIN, MIN_US, MAX_US);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, MIN_US, MAX_US);
  servo.write(MIN_ANGLE);
  delay(1000);
}

void loop() {
  sweep(MIN_ANGLE, MAX_ANGLE);
  sweep(MAX_ANGLE, MIN_ANGLE);
}
