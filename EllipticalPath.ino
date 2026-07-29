#include <Servo.h>

Servo leftServo;
Servo rightServo;

#define LEFT_PIN   13
#define RIGHT_PIN  12
#define LEFT_STOP  1507        // servo-specific neutral pulse (calibrated)
#define RIGHT_STOP 1507        // servo-specific neutral pulse (calibrated)

#define LEFT_SENSOR_PIN  A0    // swap with A1 if you rearranged the sensors
#define RIGHT_SENSOR_PIN A1

#define ON_LINE_THRESH 550     // > this = on black line; < this = on white

// Tuning for the elliptical path: a constant turn bias produces the base
// curve, with gentle proportional trim on top.
#define BASE_SPEED  50         // forward speed (lowered to reduce overshoot)
#define TURN_BIAS   8          // baseline ellipse curve
#define TURN_DIV    75         // bigger = gentler proportional turn
#define TURN_CAP    15         // max proportional turn (soft states)
#define PIVOT_HARD  22         // opposite-wheel pivot for full-loss recovery

int lastDir = 1;               // last recovery direction (+1 = steer right)
unsigned long lastPrintTime = 0;
#define PRINT_INTERVAL_MS 100  // throttle Serial so prints don't slow the loop

void setup() {
  Serial.begin(9600);
  leftServo.attach(LEFT_PIN);
  rightServo.attach(RIGHT_PIN);
  leftServo.writeMicroseconds(LEFT_STOP);
  rightServo.writeMicroseconds(RIGHT_STOP);
  delay(3000);                 // pause to position the robot at the start
}

// Dummy read + settle delay before the real read for a stable ADC value.
int readSensor(int pin) {
  analogRead(pin);
  delayMicroseconds(50);
  return analogRead(pin);
}

// Edge-following with a constant curve bias for the ellipse. The TURN_BIAS
// term keeps the robot arcing along the ellipse; proportional trim corrects
// deviation, and a hard pivot recovers if the line is lost entirely.
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
    lastDir = -1;                             // line is toward the LEFT
    state = "TRACK";
  }
  else if (!leftBlack && !rightBlack) {
    // Both white: line lost. Hard-pivot toward the side it was last seen.
    leftSpeed  = BASE_SPEED + (lastDir * PIVOT_HARD);
    rightSpeed = BASE_SPEED - (lastDir * PIVOT_HARD);
    state = "RECOVER";
  }
  else if (leftBlack && rightBlack) {
    // Both black: squarely on the line. Ease forward with the base curve bias.
    leftSpeed  = BASE_SPEED + TURN_BIAS;
    rightSpeed = BASE_SPEED - TURN_BIAS;
    state = "BOTH";
  }
  else {
    // Right black, left white: line crossed to the right sensor. Trim the other way.
    int error = right - left;
    int turn  = error / TURN_DIV;
    turn = constrain(turn, -TURN_CAP, TURN_CAP);
    leftSpeed  = BASE_SPEED + TURN_BIAS + turn;
    rightSpeed = BASE_SPEED - TURN_BIAS - turn;
    lastDir = 1;                              // line is toward the RIGHT
    state = "R-TRACK";
  }

  leftSpeed  = constrain(leftSpeed,  -150, 150);
  rightSpeed = constrain(rightSpeed, -150, 150);

  // Mirrored servo mounting: LEFT minus, RIGHT plus so equal speeds go straight.
  leftServo.writeMicroseconds(LEFT_STOP  - leftSpeed);
  rightServo.writeMicroseconds(RIGHT_STOP + rightSpeed);

  // Throttled debug output.
  unsigned long now = millis();
  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;
    Serial.print("L:");   Serial.print(left);
    Serial.print(" R:");  Serial.print(right);
    Serial.print(" ");    Serial.print(state);
    Serial.print(" dir:"); Serial.print(lastDir);
    Serial.print(" Ls:"); Serial.print(leftSpeed);
    Serial.print(" Rs:"); Serial.println(rightSpeed);
  }
}    // Both white: line slipped off left sensor toward the right. Steer RIGHT (soft).
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
