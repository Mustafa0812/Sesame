/*
 * servo-speed-tester.ino: Arduino Uno R3, 8 servos, 0 to 90 degrees in a loop
 *
 * 1. All 8 servos are driven to the same start angle (START_ANGLE) and held.
 * 2. When you send any character in the Serial Monitor, the loop starts: every
 *    servo is commanded to TARGET_ANGLE at the same instant, held for HOLD_MS,
 *    commanded back to START_ANGLE, held, and so on forever.
 * 3. Press the Uno's reset button to stop and start over.
 *
 * SPEED: a hobby servo has no position feedback, so the Arduino cannot measure
 * how fast it moves. All servos start together so you can compare them by eye.
 * For a real comparison, film the arms in slow motion on a phone and count
 * frames. Typical hobby servos need about 0.1 to 0.2 s per 60 degrees.
 *
 * WIRING
 *   Servo signal wires -> pins 2 to 9 (servo 0 on pin 2 ... servo 7 on pin 9)
 *   Servo power (red)  -> external 5 to 6 V supply, NOT the Uno's 5V pin
 *   Servo ground       -> external supply GND AND the Uno's GND (common ground)
 *
 * Arduino IDE: board "Arduino Uno", Serial Monitor at 9600 baud, any line ending.
 */

#include <Servo.h>

const uint8_t NUM_SERVOS = 8;
const uint8_t SERVO_PINS[NUM_SERVOS] = {2, 3, 4, 5, 6, 7, 8, 9};

const int START_ANGLE = 0;           // calibration position for every servo
const int TARGET_ANGLE = 90;         // where they all go
const unsigned long SETTLE_MS = 2000; // time to reach the start position
const unsigned long HOLD_MS = 1500;   // pause at each end so the servos arrive

Servo servos[NUM_SERVOS];

void setup() {
  Serial.begin(9600);

  Serial.println(F("Servo speed tester"));
  Serial.print(F("Moving all servos to start position "));
  Serial.print(START_ANGLE);
  Serial.println(F(" deg..."));

  // Attach one at a time so the start-up current surge is spread out a little.
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].write(START_ANGLE);
    delay(100);
  }
  delay(SETTLE_MS);

  Serial.println(F("All servos at start position."));
  Serial.print(F("Send any character to move them all to "));
  Serial.print(TARGET_ANGLE);
  Serial.println(F(" deg and back, forever."));

  while (!Serial.available()) { }   // wait for the trigger
  while (Serial.available()) Serial.read();
  Serial.println(F("Running. Press reset to stop."));
}

void moveAll(int angle) {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servos[i].write(angle);
  }
}

void loop() {
  moveAll(TARGET_ANGLE);
  delay(HOLD_MS);
  moveAll(START_ANGLE);
  delay(HOLD_MS);
}
