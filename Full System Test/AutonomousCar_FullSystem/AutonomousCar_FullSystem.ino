#include <Servo.h>

#define IR_SENSOR_RIGHT A0
#define IR_SENSOR_LEFT A1
#define MOTOR_SPEED 80

const int SAFE_DISTANCE = 15; // in cm
const int SCAN_ANGLE_LEFT = 150;
const int SCAN_ANGLE_RIGHT = 30;
const int CENTER_ANGLE = 90;
const int FORWARD_STEP = 200;
const int TURN_ANGLE_DURATION = 600;
const int SERVO_DELAY = 300;
const int SERVO_RESET_DELAY = 200;
const int LINE_RECHECK_DELAY = 50;
const int UTURN_DURATION = 1200;

// Ultrasonic sensor pins
const int trigPin = 6;
const int echoPin = 7;

// Right motor pins
int enableRightMotor = 2;
int rightMotorPin1 = 7;
int rightMotorPin2 = 5;

// Left motor pins
int enableLeftMotor = 3;
int leftMotorPin1 = 9;
int leftMotorPin2 = 8;

// Flags
bool red = false;

// Servo object
Servo servo;

// Function Declarations
float scanSide(bool left);
void followPerimeter(bool goLeft);
bool isLineCentered();
long readDistance();
void rotateMotor(int rightMotorSpeed, int leftMotorSpeed);
void moveForward(int speed, int duration);
void stopCar();
void turnLeft();
void turnRight();
void uTurn();
void lineFollow();

// Setup
void setup() {
  Serial.begin(9600);
  servo.attach(10);

  pinMode(IR_SENSOR_LEFT, INPUT);
  pinMode(IR_SENSOR_RIGHT, INPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  rotateMotor(0, 0);
  servo.write(CENTER_ANGLE);
}

// Main Loop
void loop() {
  if (red) {
    float leftClear = scanSide(true);
    float rightClear = scanSide(false);

    if (rightClear > SAFE_DISTANCE && rightClear >= leftClear) {
      followPerimeter(false); // Go right
    } else if (leftClear > SAFE_DISTANCE) {
      followPerimeter(true);  // Go left
    } else {
      uTurn();
    }

    red = false; // Reset for next detection
  } else {
    lineFollow();
  }
}

// Helper Functions
float scanSide(bool left) {
  int start = CENTER_ANGLE;
  int end = left ? SCAN_ANGLE_LEFT : SCAN_ANGLE_RIGHT;
  int step = left ? 5 : -5;

  float maxDistance = 0;
  for (int angle = start; angle != end; angle += step) {
    servo.write(angle);
    delay(SERVO_DELAY);
    float dist = readDistance();
    if (dist > maxDistance) maxDistance = dist;
  }
  servo.write(CENTER_ANGLE);
  delay(SERVO_RESET_DELAY);
  return maxDistance;
}

void followPerimeter(bool goLeft) {
  while (true) {
    float dist = readDistance();
    if (dist < SAFE_DISTANCE) {
      if (goLeft) turnLeft();
      else turnRight();
      delay(200);
      stopCar();
      continue;
    }

    moveForward(MOTOR_SPEED, FORWARD_STEP);
    if (isLineCentered()) {
      stopCar();
      break;
    }
  }
}

bool isLineCentered() {
  int left = digitalRead(IR_SENSOR_LEFT);
  int right = digitalRead(IR_SENSOR_RIGHT);
  return (left == HIGH && right == HIGH);
}

long readDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration / 58; // Convert to cm
}

void moveForward(int speed, int duration) {
  rotateMotor(speed, speed);
  delay(duration);
  stopCar();
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
  // Right motor control
  if (rightMotorSpeed < 0) {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  } else if (rightMotorSpeed > 0) {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  } else {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, HIGH);
  }

  // Left motor control
  if (leftMotorSpeed < 0) {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
  } else if (leftMotorSpeed > 0) {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
  } else {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);
  }

  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));
}

void stopCar() {
  rotateMotor(0, 0);
}

void turnLeft() {
  rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
  delay(TURN_ANGLE_DURATION);
  stopCar();
}

void turnRight() {
  rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);
  delay(TURN_ANGLE_DURATION);
  stopCar();
}

void uTurn() {
  rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);
  delay(UTURN_DURATION);
  stopCar();
}

void lineFollow() {
  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);

  if (rightIRSensorValue == LOW && leftIRSensorValue == LOW) {
    rotateMotor(MOTOR_SPEED, MOTOR_SPEED);  // Go straight
  } else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW) {
    rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);  // Turn right
  } else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH) {
    rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);  // Turn left
  } else {
    rotateMotor(0, 0);  // Stop
  }
}
