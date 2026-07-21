#include <Servo.h>

Servo leftServo;
Servo rightServo;

#define LEFT_PIN   13
#define RIGHT_PIN  12
#define LEFT_STOP  1507
#define RIGHT_STOP 1507

#define LEFT_SENSOR_PIN  A0    // swap with A1 if you rearranged the sensors
#define RIGHT_SENSOR_PIN A1

#define ON_LINE_THRESH 550     // > this = on black line; < this = on white

#define BASE_SPEED  50         // forward speed (lowered to reduce overshoot)
#define TURN_BIAS   8          // baseline ellipse curve
#define TURN_DIV    75         // bigger = gentler proportional turn
#define TURN_CAP    15         // max proportional turn (soft states)
#define PIVOT_HARD  22         // opposite-wheel pivot for full-loss recovery

int lastDir = 1;               // last recovery direction (+1 = steer right)

unsigned long lastPrintTime = 0;
#define PRINT_INTERVAL_MS 100

void setup() {
  Serial.begin(9600);
  leftServo.attach(LEFT_PIN);
  rightServo.attach(RIGHT_PIN);
  leftServo.writeMicroseconds(LEFT_STOP);
  rightServo.writeMicroseconds(RIGHT_STOP);
  delay(3000);
}

int readSensor(int pin) {
  analogRead(pin);
  delayMicroseconds(50);
  return analogRead(pin);
}

void loop() {
  int left  = readSensor(LEFT_SENSOR_PIN);
  int right = readSensor(RIGHT_SENSOR_PIN);

  bool leftBlack  = (left  > ON_LINE_THRESH);
  bool rightBlack = (right > ON_LINE_THRESH);

  int leftSpeed, rightSpeed;
  const char* state;

  if (leftBlack && !rightBlack) {
    // ON TARGET: left rides the line, right on white. Track with proportional trim.
    int error = left - right;                 // how far onto the line we are
    int turn  = error / TURN_DIV;
    turn = constrain(turn, -TURN_CAP, TURN_CAP);
    leftSpeed  = BASE_SPEED + TURN_BIAS - turn;
    rightSpeed = BASE_SPEED - TURN_BIAS + turn;
    state = "TRACK";
  }
  else if (!leftBlack && !rightBlack) {
    // Both white: line slipped off left sensor toward the right. Steer RIGHT (soft).
    int turn = (ON_LINE_THRESH - min(left, right)) / TURN_DIV;
    turn = constrain(turn, 0, TURN_CAP);
    leftSpeed  = BASE_SPEED + turn;
    rightSpeed = BASE_SPEED - turn;
    lastDir = 1;
    state = "white->R";
  }
  else if (leftBlack && rightBlack) {
    // Both black: drifted right, line under both. Steer LEFT (soft).
    int turn = (min(left, right) - ON_LINE_THRESH) / TURN_DIV;
    turn = constrain(turn, 0, TURN_CAP);
    leftSpeed  = BASE_SPEED - turn;
    rightSpeed = BASE_SPEED + turn;
    lastDir = -1;
    state = "black->L";
  }
  else {
    // Right black, left white: line crossed fully over. Hard pivot LEFT to recover.
    leftSpeed  =  0;
    rightSpeed =  PIVOT_HARD;
    lastDir = -1;
    state = "R-only->L-hard";
  }

  leftSpeed  = constrain(leftSpeed,  -150, 150);
  rightSpeed = constrain(rightSpeed, -150, 150);

  leftServo.writeMicroseconds(LEFT_STOP  - leftSpeed);
  rightServo.writeMicroseconds(RIGHT_STOP + rightSpeed);

  unsigned long now = millis();
  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;
    Serial.print("L:");   Serial.print(left);
    Serial.print(" R:");  Serial.print(right);
    Serial.print(" ");    Serial.print(state);
    Serial.print(" Ls:"); Serial.print(leftSpeed);
    Serial.print(" Rs:"); Serial.println(rightSpeed);
  }
}