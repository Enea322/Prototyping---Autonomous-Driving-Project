#include <Servo.h>
#include <math.h>

// ===== Pin Definitions =====
// IR line sensors
#define IR_SENSOR_RIGHT A0
#define IR_SENSOR_LEFT  A1

// Motors
#define MOTOR_SPEED     80
#define TURN_SPEED      65
int enableRightMotor = 2;
int rightMotorPin1   = 7;
int rightMotorPin2   = 5;
int enableLeftMotor  = 3;
int leftMotorPin1    = 9;
int leftMotorPin2    = 8;

// Ultrasonic
#define trigPin 13
#define echoPin 12

// Servo (for ultrasonic scan & existing avoid)
const int servo = 11;

// Color sensor (TCS230/TCS3200 style)
#define S0 A2
#define S1 A3
#define S2 A4
#define S3 A5
#define sensorOut 10

// ===== Globals & State =====
enum LastSeen { NONE, LEFT, RIGHT };
LastSeen lastSeen    = NONE;
unsigned long lostTimer = 0;
int safe_dis = 10; // in cm
int left_dis, right_dis, front_dis;
bool avoiding = false;
float redSum = 0;

// Color‐sensor calibration & measurement
float redAmbient = 0;
float redAvg     = 0;

// ===== Function Prototypes =====
void followLine();
void recoverLine();
void rotateMotor(int rightMotorSpeed, int leftMotorSpeed);
long readDistance();
void servoPulse(int pin, int angle);
void avoidObject();
void Check_side();
void moveForward();
void turnRight();
void turnLeft();
void stopCar();
void turnForward();
void turnBackward();

// New color‐sensor helpers
void calibrateColorSensor();
bool isRedObject();
void removeObject();

void setup() {
  Serial.begin(9600);

  // Motor pins
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  // Line sensors
  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT,  INPUT);

  // Ultrasonic
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Servo
  pinMode(servo, OUTPUT);

  // Color sensor
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  // set frequency scaling to 20%
  digitalWrite(S0, LOW);
  digitalWrite(S1, HIGH);

  // center ultrasonic servo
  servoPulse(servo, 90);
  delay(200);

  // ultrasonic warm‐up
  for (int i = 0; i < 3; i++) {
    readDistance();
    delay(50);
  }

  // motors off
  rotateMotor(0, 0);
  delay(500);

  // calibrate ambient red frequency
  calibrateColorSensor();
}

void loop() {
  front_dis = readDistance();

  if (!avoiding) {
    if (front_dis > 0 && front_dis <= safe_dis) {
      // object detected
      avoiding = true;
      stopCar();
      delay(200);

      // check color: red → avoid, otherwise remove
      if (isRedObject()) {
        stopCar();
        delay(200);
        avoidObject();
      } else {
        stopCar();
        delay(200);
        removeObject();
      }
      // after removeObject we set avoiding = false
    }
    else {
      // normal line follow
      followLine();
    }
  }
  else {
    // “clearing” mode: go forward until clear
    if (front_dis == 0 || front_dis > safe_dis) {
      avoiding = false;
      servoPulse(servo, 90);
      delay(200);
    }
    else {
      followLine();
    }
  }
}

// ===== Line Following & Recovery =====
void followLine() {
  bool rightIR = (digitalRead(IR_SENSOR_RIGHT) == HIGH);
  bool leftIR  = (digitalRead(IR_SENSOR_LEFT ) == HIGH);

  if ( rightIR && !leftIR ) {
    lastSeen   = RIGHT;
    turnRight();
    lostTimer  = 0;
  }
  else if ( !rightIR && leftIR ) {
    lastSeen   = LEFT;
    turnLeft();
    lostTimer  = 0;
  }
  else if ( !rightIR && !leftIR ) {
    if ( lostTimer == 0 ) lostTimer = millis();
    moveForward();
    if ( millis() - lostTimer > 80 ) {
      recoverLine();
      lostTimer = 0;
    }
  }
  else {
    // both HIGH — unlikely
    moveForward();
    lastSeen  = NONE;
    lostTimer = 0;
  }
}

void recoverLine() {
  stopCar();
  delay(100);

  if ( lastSeen == RIGHT ) {
    turnRight();
    while ( digitalRead(IR_SENSOR_RIGHT) == LOW ) { }
    stopCar(); delay(100);
    turnRight();
    while ( digitalRead(IR_SENSOR_RIGHT) == HIGH ) { }
    stopCar(); delay(100);
  }
  else if ( lastSeen == LEFT ) {
    turnLeft();
    while ( digitalRead(IR_SENSOR_LEFT) == LOW ) { }
    stopCar(); delay(100);
    turnLeft();
    while ( digitalRead(IR_SENSOR_LEFT) == HIGH ) { }
    stopCar(); delay(100);
  }
  else {
    turnBackward();
    delay(200);
    stopCar();
    delay(100);
  }

  moveForward();
  delay(100);
  stopCar();
}

// ===== Motor & Sensor Utilities =====
void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
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
  analogWrite(enableLeftMotor,  abs(leftMotorSpeed));
}

long readDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long t = pulseIn(echoPin, HIGH, 30000);
  return t ? t / 29 / 2 : 0;
}

void servoPulse(int pin, int angle) {
  int pwm = (angle * 11) + 500;
  digitalWrite(pin, HIGH);
  delayMicroseconds(pwm);
  digitalWrite(pin, LOW);
  delay(50);
}

// ===== Existing Obstacle Avoidance =====
void avoidObject() {
  Check_side();
  if (right_dis >= left_dis) {
    // steer right around
    turnBackward(); delay(500); stopCar(); delay(500);

    turnRight();    delay(300); stopCar(); delay(100);

    turnForward();  delay(650); stopCar(); delay(500);

    turnLeft();     delay(500); stopCar(); delay(500);

    turnForward();
    while (digitalRead(IR_SENSOR_RIGHT) == LOW) { }
    stopCar(); delay(500);
    
    turnRight();
    while (digitalRead(IR_SENSOR_RIGHT) == HIGH) { }
    stopCar(); delay(500);
  }
  else {
    // steer left around
    turnBackward(); delay(500); stopCar(); delay(500);

    turnLeft();     delay(300); stopCar(); delay(100);

    turnForward();  delay(650); stopCar(); delay(500);

    turnRight();    delay(500); stopCar(); delay(500);

    turnForward();
    while (digitalRead(IR_SENSOR_LEFT ) == LOW) { }
    stopCar(); delay(500);

    turnLeft();
    while (digitalRead(IR_SENSOR_LEFT ) == HIGH) { }
    stopCar(); delay(500);
  }
}

void Check_side() {
  stopCar(); delay(100);
  for (int angle = 70; angle <= 140; angle += 5) servoPulse(servo, angle);
  delay(300);
  right_dis = readDistance();
  delay(100);
  for (int angle = 140; angle >= 0; angle -= 5) servoPulse(servo, angle);
  delay(500);
  left_dis = readDistance();
  delay(100);
  for (int angle = 0; angle <= 70; angle += 5) servoPulse(servo, angle);
  delay(300);
  Serial.print("R="); Serial.print(right_dis);
  Serial.print("  L="); Serial.println(left_dis);
}

void moveForward()   { rotateMotor(MOTOR_SPEED,  MOTOR_SPEED); }
void turnRight()     { rotateMotor(-MOTOR_SPEED, MOTOR_SPEED); }
void turnLeft()      { rotateMotor(MOTOR_SPEED, -MOTOR_SPEED); }
void stopCar()       { rotateMotor(0, 0); }
void turnForward()   { rotateMotor(TURN_SPEED,  TURN_SPEED); }
void turnBackward()  { rotateMotor(-TURN_SPEED, -TURN_SPEED); }

// ===== New Remove‐Object Behavior =====
void removeObject() {
  // back up a bit
  turnBackward();
  delay(500);
  stopCar();
  delay(500);

  // push/remove object
  moveForward();
  delay(650);
  stopCar();
  delay(500);

  // resume line following
  avoiding = false;
}

// ===== Color Sensor Helpers =====
void calibrateColorSensor() {
  redSum = 0;
  for (int i = 0; i < 10; i++) {
    digitalWrite(S2, LOW);
    digitalWrite(S3, LOW);
    long edgeTime = pulseIn(sensorOut, HIGH) + pulseIn(sensorOut, LOW);
    float freq = edgeTime ? (1.0f / (edgeTime / 1000000.0f)) : 0;
    redSum += freq;
  }
  redAmbient = redSum / 10.0f;
}

bool isRedObject() {
  redSum = 0;
  for (int i = 0; i < 10; i++) {
    digitalWrite(S2, LOW);
    digitalWrite(S3, LOW);
    long edgeTime = pulseIn(sensorOut, HIGH) + pulseIn(sensorOut, LOW);
    float freq = edgeTime ? (1.0f / (edgeTime / 1000000.0f)) : 0;
    redSum += freq;
  }
  redAvg = redSum / 10.0f;
  Serial.print("Red avg: "); Serial.println(redAvg);
  return (redAvg > redAmbient);
}