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

// Tuning for the irregular course: slow and responsive so tight hairpins can
// be tracked without overshooting.
#define BASE_SPEED  5          // lowered: hairpins need time to turn
#define TURN_DIV    30         // proportional divisor; lower = more responsive
#define TURN_CAP    55         // correction limit; raised = authority on tight curves
#define PIVOT_HARD  40         // recovery pivot strength

int lastDir = 1;               // last side the line was seen on:
                               // +1 = toward RIGHT sensor, -1 = toward LEFT
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

// Single-sensor edge-following with a four-state controller. The active
// sensor rides the line edge; the difference between the two readings sets a
// proportional steering correction. lastDir remembers which way the line went
// so the robot can hunt back toward it when both sensors lose the line.
void loop() {
  int left  = readSensor(LEFT_SENSOR_PIN);
  int right = readSensor(RIGHT_SENSOR_PIN);

  bool leftBlack  = (left  > ON_LINE_THRESH);
  bool rightBlack = (right > ON_LINE_THRESH);

  int leftSpeed, rightSpeed;
  const char* state;

  if (leftBlack && !rightBlack) {
    // ON TARGET: left sensor rides the line, right on white. Steer proportionally.
    int turn = constrain((left - right) / TURN_DIV, -TURN_CAP, TURN_CAP);
    leftSpeed  = BASE_SPEED - turn;
    rightSpeed = BASE_SPEED + turn;
    lastDir = -1;                     // line is toward the LEFT sensor
    state = "TRACK";
  }
  else if (!leftBlack && !rightBlack) {
    // Both white: line has slipped away. Correct toward where it was last seen.
    int turn = constrain((ON_LINE_THRESH - min(left, right)) / TURN_DIV, 0, TURN_CAP);
    leftSpeed  = BASE_SPEED + (lastDir * turn);
    rightSpeed = BASE_SPEED - (lastDir * turn);
    state = "SEARCH";
  }
  else if (leftBlack && rightBlack) {
    // Both black: squarely on the line (or a tight corner). Ease forward.
    leftSpeed  = BASE_SPEED;
    rightSpeed = BASE_SPEED;
    state = "BOTH";
  }
  else {
    // Right black, left white: line has crossed to the right sensor.
    int turn = constrain((right - left) / TURN_DIV, -TURN_CAP, TURN_CAP);
    leftSpeed  = BASE_SPEED + turn;
    rightSpeed = BASE_SPEED - turn;
    lastDir = 1;                      // line is toward the RIGHT sensor
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
}
