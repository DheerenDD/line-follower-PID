#include <Servo.h>

Servo leftServo;
Servo rightServo;

#define LEFT_PIN   13
#define RIGHT_PIN  12
#define LEFT_STOP  1507
#define RIGHT_STOP 1507
#define BASE_SPEED 50

#define SENSOR_PIN A0

#define BLACK_VALUE 900
#define WHITE_VALUE 100
#define TARGET      450

float Kp = 0.30;
float Ki = 0.002;
float Kd = 0.20;

float lastError = 0;
unsigned long lastTime = 0;
unsigned long lastPrintTime = 0;

#define PRINT_INTERVAL_MS 100

void setup() {
  Serial.begin(9600);

  leftServo.attach(LEFT_PIN);
  rightServo.attach(RIGHT_PIN);

  leftServo.writeMicroseconds(LEFT_STOP);
  rightServo.writeMicroseconds(RIGHT_STOP);

  delay(3000);
  lastTime = millis();
}

int fastRead() {
  return analogRead(SENSOR_PIN);
  delayMicroseconds(50);   // let it settle
  return analogRead(SENSOR_PIN);  // real read
}

void loop() {
  int val = fastRead();

  float error = (float)(val - TARGET);

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.001;

  float derivative = (error - lastError) / dt;
  float correction = (Kp * error) + (Kd * derivative);

  lastError = error;
  lastTime  = now;

  int leftSpeed  = BASE_SPEED + (int)(correction * 0.2);
  int rightSpeed = BASE_SPEED - (int)(correction * 0.2);

  leftSpeed  = constrain(leftSpeed,  -150, 150);
  rightSpeed = constrain(rightSpeed, -150, 150);

  leftServo.writeMicroseconds(LEFT_STOP  - leftSpeed);
  rightServo.writeMicroseconds(RIGHT_STOP + rightSpeed);

  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;
    Serial.println("Val:"); Serial.print(val);
    Serial.print(" Err:"); Serial.print(error);
    Serial.print(" Cor:"); Serial.print(correction);
    Serial.print(" Ls:"); Serial.print(leftSpeed);
    Serial.print(" Rs:"); Serial.println(rightSpeed);
  }
}