#include <Servo.h>

Servo leftServo;
Servo rightServo;

#define LEFT_PIN   12
#define RIGHT_PIN  13
#define LEFT_STOP  1483
#define RIGHT_STOP 1515

#define PIVOT_SPEED   25
#define TURN_TIME_MS  1925   // right-turn duration

void setup() {
  leftServo.attach(LEFT_PIN);
  rightServo.attach(RIGHT_PIN);

  // Rest.
  leftServo.writeMicroseconds(LEFT_STOP);
  rightServo.writeMicroseconds(RIGHT_STOP);
  delay(2000);

  // Right pivot: left wheel forward, right wheel backward.
  leftServo.writeMicroseconds(LEFT_STOP - PIVOT_SPEED);
  rightServo.writeMicroseconds(RIGHT_STOP - PIVOT_SPEED);
  delay(TURN_TIME_MS);

  // Rest.
  leftServo.writeMicroseconds(LEFT_STOP);
  rightServo.writeMicroseconds(RIGHT_STOP);
}

void loop() {}
