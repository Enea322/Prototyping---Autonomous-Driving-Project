#include <Servo.h>

#define IR_SENSOR_RIGHT A0
#define IR_SENSOR_LEFT  A1
#define MOTOR_SPEED     80

// Right motor
int enableRightMotor = 2;
int rightMotorPin1   = 7;
int rightMotorPin2   = 5;

// Left motor
int enableLeftMotor  = 3;
int leftMotorPin1    = 9;
int leftMotorPin2    = 8;

// Ultrasonic pins
#define trigPin 13
#define echoPin 12

// Servo pin
const int servo = 11;

int safe_dis = 12; // in cm
int left_dis, right_dis, front_dis;

// New flag: have we already started avoiding this obstacle?
bool avoiding = false;

void setup() {
  Serial.begin(9600);

  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT,  INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(servo, OUTPUT);

  servoPulse(servo, 90);
  delay(200);

  // ultrasonic warm-up
  for (int i = 0; i < 3; i++) { readDistance(); delay(50); }

  rotateMotor(0, 0);
  delay(500);
}

void loop() {
  front_dis = readDistance();
  //Serial.print("Front: "); Serial.print(front_dis); Serial.println(" cm");

  // If we're not already avoiding...
  if (!avoiding) {
    // and we see a real obstacle
    if (front_dis > 0 && front_dis <= safe_dis) {
      avoiding = true;
      stopCar();
      delay(200);
      avoidObject();        // does Check_side() + turn
      delay(800);
      stopCar();
      // leave avoiding==true so we don’t re-scan immediately
    }
    else {
      // normal line follow
      int rightIR = digitalRead(IR_SENSOR_RIGHT);
      int leftIR  = digitalRead(IR_SENSOR_LEFT);
      if (rightIR == LOW && leftIR == LOW)      moveForward();
      else if (rightIR == HIGH && leftIR == LOW) turnRight();
      else if (rightIR == LOW && leftIR == HIGH) turnLeft();
      else                                       stopCar();
    }
  }
  else {
    // we’re in “clearing” mode: keep going forward until path is clear
    if (front_dis == 0 || front_dis > safe_dis) {
      avoiding = false;        // done avoiding
      servoPulse(servo, 90);   // re-center servo
      delay(200);
    }
    else {
      moveForward();
    }
  }
}

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

void avoidObject() {
  Check_side();
  
  turnRight();
  delay(300);
  //stopCar();
  //delay(500);

  moveForward();
  delay(5500);
  stopCar();
  delay(500);

  turnLeft();
  delay(250);
  stopCar();
  delay(500);

  moveForward();
  delay(150);
  stopCar();
  turnRight();
  delay(80);
  stopCar();
  delay(500);
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

void moveForward() { rotateMotor(MOTOR_SPEED,  MOTOR_SPEED); }
void turnRight()   { rotateMotor(-MOTOR_SPEED, MOTOR_SPEED); }
void turnLeft()    { rotateMotor(MOTOR_SPEED, -MOTOR_SPEED); }
void stopCar()     { rotateMotor(0, 0); }
