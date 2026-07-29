#include <Servo.h>

Servo testServo;

#define TEST_PIN  13     // pin of the servo under test

void setup() {
  testServo.attach(TEST_PIN);

  // Change this value and re-upload to observe the servo's behaviour.
  //   1507 = stop (neutral)   > 1507 = one direction   < 1507 = the other
  testServo.writeMicroseconds(1507);
}

void loop() {
  // Command is held; nothing repeats.
}
