#include <Servo.h>

Servo leftServo;
Servo rightServo;

// ==================== CONFIGURATION ====================

#define START_X       0
#define START_Y       0
#define START_HEADING 0        // 0=North(+Y), 1=East(+X), 2=South(-Y), 3=West(-X)

// Target waypoints in grid coordinates; visited in order.
int waypoints[][2] = {
  {3, 2},
  {5, 5},
  {1, 4}
};
const int numWaypoints = sizeof(waypoints) / sizeof(waypoints[0]);

// ==================== HARDWARE ====================

#define LEFT_PIN   13
#define RIGHT_PIN  12
#define LEFT_STOP  1507        // servo-specific neutral pulse (calibrated)
#define RIGHT_STOP 1507        // servo-specific neutral pulse (calibrated)

#define LEFT_SENSOR_PIN  A0    // IR reflectance sensors on analog outputs
#define RIGHT_SENSOR_PIN A1

#define ON_LINE_THRESH 550     // reading above this = black line detected

// ==================== TUNING ====================

#define BASE_SPEED      15     // forward speed offset from neutral

#define TURN_DIV        90     // (reserved) proportional steering divisor
#define TURN_CAP        18     // (reserved) steering correction limit

#define PIVOT_SPEED          30
#define TURN_TIME_RIGHT_MS  2000   // calibrated 90-degree RIGHT turn from rest
#define TURN_TIME_LEFT_MS   1650   // calibrated 90-degree LEFT turn from rest
#define TURN_SETTLE_MS       500   // pause before a pivot to kill momentum
#define NUDGE_TIME_MS        1500  // creep past a crossing to align turning axis
#define CROSS_BLANK_MS       400   // debounce: ignore repeat crossings within this

#define SPIN_SPEED      30
#define SPIN_TIME_MS    8000

// ==================== STATE ====================

int curX = START_X;
int curY = START_Y;
int heading = START_HEADING;

bool prevAnyBlack = false;             // previous crossing state (for edge detect)
unsigned long lastCrossTime = 0;

const char* dirName(int h) {
  switch (h) {
    case 0: return "North";
    case 1: return "East";
    case 2: return "South";
    case 3: return "West";
  }
  return "?";
}

// ==================== LOW LEVEL ====================

// Dummy read + settle delay before the real read for a stable ADC value.
int readSensor(int pin) {
  analogRead(pin);
  delayMicroseconds(50);
  return analogRead(pin);
}

// Mirrored servo mounting: LEFT uses minus, RIGHT uses plus so that equal
// speeds drive the robot straight instead of spinning in place.
void drive(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed,  -150, 150);
  rightSpeed = constrain(rightSpeed, -150, 150);
  leftServo.writeMicroseconds(LEFT_STOP  - leftSpeed);
  rightServo.writeMicroseconds(RIGHT_STOP + rightSpeed);
}

void stopMotors() {
  leftServo.writeMicroseconds(LEFT_STOP);
  rightServo.writeMicroseconds(RIGHT_STOP);
}

// ==================== LINE FOLLOWING ====================
// The robot drives straight between intersections; alignment is handled by
// physical placement rather than active steering, so a crossing cannot pull
// the robot off course.

void followLineStep(int left, int right) {
  drive(BASE_SPEED, BASE_SPEED);
}

// Intersection = rising edge of EITHER sensor going black. Using "either"
// (not "both") also catches edge crossings where only the inner sensor sees
// the perpendicular line. Debounced by CROSS_BLANK_MS.
bool crossedIntersection(int left, int right) {
  bool anyBlack = (left > ON_LINE_THRESH) || (right > ON_LINE_THRESH);
  bool rising = anyBlack && !prevAnyBlack;
  prevAnyBlack = anyBlack;
  if (rising && (millis() - lastCrossTime > CROSS_BLANK_MS)) {
    lastCrossTime = millis();
    return true;
  }
  return false;
}

// ==================== MOVEMENT PRIMITIVES ====================

// Drive forward one grid cell: advance until the next intersection, nudge the
// turning axis onto it, stop, then update the tracked coordinate.
void driveOneCell() {
  prevAnyBlack = false;
  lastCrossTime = millis();

  while (true) {
    int left  = readSensor(LEFT_SENSOR_PIN);
    int right = readSensor(RIGHT_SENSOR_PIN);
    if (crossedIntersection(left, right)) break;
    followLineStep(left, right);
  }

  if (NUDGE_TIME_MS > 0) {
    drive(BASE_SPEED, BASE_SPEED);
    delay(NUDGE_TIME_MS);
  }
  stopMotors();
  delay(120);

  switch (heading) {
    case 0: curY++; break;
    case 1: curX++; break;
    case 2: curY--; break;
    case 3: curX--; break;
  }

  Serial.print("   crossed ("); Serial.print(curX);
  Serial.print(","); Serial.print(curY);
  Serial.print(")  heading "); Serial.println(dirName(heading));
}

// Timed open-loop 90-degree pivot from rest. dir = +1 right, -1 left.
// Left and right use separately calibrated durations because the two servos
// are not perfectly matched.
void turn90(int dir) {
  stopMotors();
  delay(TURN_SETTLE_MS);                 // settle so momentum doesn't overshoot

  int turnTime = (dir > 0) ? TURN_TIME_RIGHT_MS : TURN_TIME_LEFT_MS;

  drive(dir * PIVOT_SPEED, -dir * PIVOT_SPEED);
  delay(turnTime);

  stopMotors();
  delay(200);

  heading = (heading + (dir > 0 ? 1 : 3)) % 4;
}

// Rotate to face a target heading using the shortest turn.
void faceHeading(int target) {
  int diff = (target - heading + 4) % 4;
  if (diff == 1)      turn90(+1);
  else if (diff == 3) turn90(-1);
  else if (diff == 2) { turn90(+1); turn90(+1); }

  Serial.print("   turned to face "); Serial.println(dirName(heading));
}

// ==================== NAVIGATION ====================

// Manhattan path planning: resolve the X displacement first, then Y.
void goToWaypoint(int tx, int ty) {
  int dx = tx - curX;
  if (dx != 0) {
    faceHeading(dx > 0 ? 1 : 3);
    for (int i = 0; i < abs(dx); i++) driveOneCell();
  }
  int dy = ty - curY;
  if (dy != 0) {
    faceHeading(dy > 0 ? 0 : 2);
    for (int i = 0; i < abs(dy); i++) driveOneCell();
  }
}

// ==================== ROUTE PREVIEW ====================

// Print the full intersection-by-intersection route before driving, so the
// planned path can be verified against the physical grid.
void printFullRoute() {
  int px = START_X, py = START_Y;

  Serial.println("Full route (every intersection):");
  Serial.print("  ("); Serial.print(px);
  Serial.print(","); Serial.print(py); Serial.print(")  [start]");

  for (int w = 0; w < numWaypoints; w++) {
    int tx = waypoints[w][0];
    int ty = waypoints[w][1];

    int dx = tx - px;
    int stepX = (dx > 0) ? 1 : -1;
    for (int i = 0; i < abs(dx); i++) {
      px += stepX;
      Serial.print(" -> ("); Serial.print(px);
      Serial.print(","); Serial.print(py); Serial.print(")");
    }
    int dy = ty - py;
    int stepY = (dy > 0) ? 1 : -1;
    for (int i = 0; i < abs(dy); i++) {
      py += stepY;
      Serial.print(" -> ("); Serial.print(px);
      Serial.print(","); Serial.print(py); Serial.print(")");
    }
    Serial.print("  [wp "); Serial.print(w + 1); Serial.print("]");
  }
  Serial.println();
  Serial.println();
}

// Spin in place to signal the route is complete.
void spinFinish() {
  drive(SPIN_SPEED, -SPIN_SPEED);
  delay(SPIN_TIME_MS);
  stopMotors();
}

// ==================== MAIN ====================

void setup() {
  Serial.begin(9600);
  leftServo.attach(LEFT_PIN);
  rightServo.attach(RIGHT_PIN);
  stopMotors();

  Serial.println("=================================");
  Serial.print("Start: ("); Serial.print(START_X);
  Serial.print(","); Serial.print(START_Y);
  Serial.print(") facing "); Serial.println(dirName(START_HEADING));

  Serial.print("Given waypoints:");
  for (int w = 0; w < numWaypoints; w++) {
    Serial.print(" ("); Serial.print(waypoints[w][0]);
    Serial.print(","); Serial.print(waypoints[w][1]); Serial.print(")");
  }
  Serial.println();
  Serial.println("=================================");

  printFullRoute();

  delay(3000);   // pause so the robot can be positioned at the start

  for (int w = 0; w < numWaypoints; w++) {
    Serial.print("Heading to waypoint "); Serial.print(w + 1);
    Serial.print(": ("); Serial.print(waypoints[w][0]);
    Serial.print(","); Serial.print(waypoints[w][1]); Serial.println(")");

    goToWaypoint(waypoints[w][0], waypoints[w][1]);

    Serial.print(">> reached waypoint "); Serial.print(w + 1);
    Serial.print(" ("); Serial.print(curX);
    Serial.print(","); Serial.print(curY); Serial.println(")");
    Serial.println();
  }

  stopMotors();
  Serial.println("Route complete. Pausing 2s.");
  delay(2000);

  Serial.println("Spinning 720.");
  spinFinish();
  stopMotors();

  Serial.println("Done.");
}

void loop() {
  // Entire route runs once in setup(); nothing repeats.
}
