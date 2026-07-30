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

// Base motion for the ellipse.
#define BASE_SPEED  50         // forward speed
#define TURN_BIAS   8          // constant curve bias that holds the elliptical arc

// PID gains for the edge-tracking correction (steer around the line edge).
// Start with KP only, then add KD to damp oscillation, then a little KI to
// remove steady offset. Retune on the actual track.
#define KP   0.15
#define KI   0.0
#define KD   0.08

#define TURN_CAP    15         // limit on the PID steering correction
#define I_CAP       30         // anti-windup clamp on the integral term
#define PIVOT_HARD  22         // opposite-wheel pivot for full-loss recovery

int lastDir = 1;               // last recovery direction (+1 = steer right)

// PID state.
float integral = 0;
float lastError = 0;
unsigned long lastTime = 0;

unsigned long lastPrintTime = 0;
#define PRINT_INTERVAL_MS 100  // throttle Serial so prints don't slow the loop

void setup() {
  Serial.begin(9600);
  leftServo.attach(LEFT_PIN);
  rightServo.attach(RIGHT_PIN);
  leftServo.writeMicroseconds(LEFT_STOP);
  rightServo.writeMicroseconds(RIGHT_STOP);
  lastTime = millis();
  delay(3000);                 // pause to position the robot at the start
}

// Dummy read + settle delay before the real read for a stable ADC value.
int readSensor(int pin) {
  analogRead(pin);
  delayMicroseconds(50);
  return analogRead(pin);
}

// PID controller for the edge-tracking error. Returns a steering correction,
// capped to TURN_CAP. dt is measured each call; a guard prevents the
// derivative term from spiking when dt is near zero.
float pidCorrection(float error) {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;   // seconds
  lastTime = now;
  if (dt <= 0) dt = 0.001;                // guard against divide-by-zero / D spike

  integral += error * dt;
  integral = constrain(integral, -I_CAP, I_CAP);   // anti-windup

  float derivative = (error - lastError) / dt;
  lastError = error;

  float output = KP * error + KI * integral + KD * derivative;
  return constrain(output, -TURN_CAP, TURN_CAP);
}

// Edge-following with a constant ellipse bias plus PID trim. TURN_BIAS keeps
// the robot arcing along the ellipse; the PID output corrects deviation from
// the line edge; a hard pivot recovers if the line is lost entirely.
void loop() {
  int left  = readSensor(LEFT_SENSOR_PIN);
  int right = readSensor(RIGHT_SENSOR_PIN);

  bool leftBlack  = (left  > ON_LINE_THRESH);
  bool rightBlack = (right > ON_LINE_THRESH);

  int leftSpeed, rightSpeed;
  const char* state;

  if (leftBlack && !rightBlack) {
    // ON TARGET: left rides the line, right on white. PID trim on the edge error.
    float error = left - right;             // how far onto the line we are
    float turn  = pidCorrection(error);
    leftSpeed  = BASE_SPEED + TURN_BIAS - turn;
    rightSpeed = BASE_SPEED - TURN_BIAS + turn;
    lastDir = -1;                           // line is toward the LEFT
    state = "TRACK";
  }
  else if (!leftBlack && !rightBlack) {
    // Both white: line lost. Reset PID and hard-pivot toward the last-seen side.
    integral = 0;
    lastError = 0;
    lastTime = millis();
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
    // Right black, left white: line crossed to the right sensor. PID trim, mirrored.
    float error = right - left;
    float turn  = pidCorrection(error);
    leftSpeed  = BASE_SPEED + TURN_BIAS + turn;
    rightSpeed = BASE_SPEED - TURN_BIAS - turn;
    lastDir = 1;                            // line is toward the RIGHT
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
