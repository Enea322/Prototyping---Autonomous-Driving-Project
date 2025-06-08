// Global Constants
const int MOTOR_SPEED = 150;

// Movement durations
const int FORWARD_DURATION_SHORT = 1500;
const int FORWARD_DURATION_LONG  = 2000;
const int FORWARD_DURATION_UTURN = 3000;
const int TURN_90_DURATION       = 600;
const int UTURN_DURATION         = 1200;
const int SERVO_DELAY            = 300;
const int SERVO_RESET_DELAY      = 200;

// IR Sensors
const int IR_SENSOR_LEFT  = A1;
const int IR_SENSOR_RIGHT = A0;

// Flags
bool red = false;
bool ambientChecked = false;

// Include Servo object
#include <Servo.h>
Servo servo;

// Function Declarations
bool isLeftClear();
bool isRightClear();
void moveForward(int speed, int duration);
void turnLeft90();
void turnRight90();
void uTurn();
void moveUntilLine();
void rotateMotor(int leftSpeed, int rightSpeed);

// Setup
void setup() {
  Serial.begin(9600);
  servo.attach(9);

  pinMode(IR_SENSOR_LEFT, INPUT);
  pinMode(IR_SENSOR_RIGHT, INPUT);

  // Other setup codes here
}

// Main Control Loop
void loop() {
  if (red) {
    Serial.println("Red detected");

    bool leftClear = isLeftClear();
    bool rightClear = isRightClear();

    if (rightClear && !leftClear) {
      // Case 1: Go around obstacle via right side
      turnRight90();
      moveForward(MOTOR_SPEED, FORWARD_DURATION_SHORT);
      turnLeft90();
      moveForward(MOTOR_SPEED, FORWARD_DURATION_LONG);
      turnLeft90();
      moveUntilLine();
    }
    else if (!rightClear && leftClear) {
      // Case 2: Go around obstacle via left side
      turnLeft90();
      moveForward(MOTOR_SPEED, FORWARD_DURATION_SHORT);
      turnRight90();
      moveForward(MOTOR_SPEED, FORWARD_DURATION_LONG);
      turnRight90();
      moveUntilLine();
    }
    else {
      // Case 3: U-turn and find alternate path
      uTurn();
      moveForward(MOTOR_SPEED, FORWARD_DURATION_SHORT);
      turnRight90();
      moveForward(MOTOR_SPEED, FORWARD_DURATION_UTURN);
      turnRight90();
      moveForward(MOTOR_SPEED, FORWARD_DURATION_UTURN);
      turnRight90();
      moveUntilLine();
    }

    Serial.println("Line reacquired, resuming line following.");
    ambientChecked = false; // Reset for next red detection
  }

  // Add the line following logic here
}

// === Function Definitions ===
bool isLeftClear() {
  servo.write(150);  // Look left
  delay(SERVO_DELAY);
  long distance = readDistance();
  servo.write(90);   // Reset
  delay(SERVO_RESET_DELAY);
  return distance > 10;
}

bool isRightClear() {
  servo.write(30);  // Look right
  delay(SERVO_DELAY);
  long distance = readDistance();
  servo.write(90);  // Reset
  delay(SERVO_RESET_DELAY);
  return distance > 10;
}

void moveForward(int speed, int duration) {
  rotateMotor(speed, speed);
  delay(duration);
  rotateMotor(0, 0);
}

void turnLeft90() {
  rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);
  delay(TURN_90_DURATION);  
  rotateMotor(0, 0);
}

void turnRight90() {
  rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
  delay(TURN_90_DURATION);  
  rotateMotor(0, 0);
}

void uTurn() {
  rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
  delay(UTURN_DURATION);  // 180° turn
  rotateMotor(0, 0);
}

void moveUntilLine() {
  while (true) {
    int rightIR = digitalRead(IR_SENSOR_RIGHT);
    int leftIR = digitalRead(IR_SENSOR_LEFT);
    if (rightIR == LOW || leftIR == LOW) break;
    rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
    delay(50);
  }
  rotateMotor(0, 0);
}

long readDistance() {
  // Return simulated or actual ultrasonic distance
  return 20; // For test purposes
}

void rotateMotor(int leftSpeed, int rightSpeed) {
  // Insert motor control code here
}
