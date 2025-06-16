#include <Servo.h>

#define IR_SENSOR_RIGHT A0
#define IR_SENSOR_LEFT A1
#define MOTOR_SPEED 80

// Right motor
int enableRightMotor = 2;
int rightMotorPin1 = 7;
int rightMotorPin2 = 5;

// Left motor
int enableLeftMotor = 3;
int leftMotorPin1 = 9;
int leftMotorPin2 = 8;

// Ultrasonic pins
#define trigPin 13
#define echoPin 12

// Servo pin
const int servo = 11;
//Servo servo;

int safe_dis = 12; // in cm
int left_dis, right_dis, front_dis;

void setup() {
  Serial.begin(9600);

  // Motor setup
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  // IR sensor setup
  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);

  // Ultrasonic setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Servo setup
  pinMode(servo, OUTPUT);

  for (int angle = 70; angle <= 140; angle += 5)  {
   servoPulse(servo, angle);  
  }
  for (int angle = 140; angle >= 0; angle -= 5)  {
   servoPulse(servo, angle);  
  }

  for (int angle = 0; angle <= 70; angle += 5)  {
   servoPulse(servo, angle);  
  }

  front_dis = readDistance();

  // Initially stop
  rotateMotor(0, 0);

  delay(500);
}

void loop() {
  front_dis = readDistance();
  Serial.print("Distance: "); Serial.print(distance); Serial.println(" cm");

  //==============================================================
  // IR sensor reading
  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);
  if (rightIRSensorValue == LOW && leftIRSensorValue == LOW) {
      if(front_dis > safe_dis){
    moveForward();
    }
    else{
      stopCar();
      delay(2000);
      avoidObject();
    } 
  }

  //================================================================
  // Line following logic
  /*if (rightIRSensorValue == LOW && leftIRSensorValue == LOW) {
    rotateMotor(MOTOR_SPEED, MOTOR_SPEED);  // Go straight
  }*/
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW) {
    turnRight();  // Turn right
  }
  else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH) {
    turnLeft();  // Turn left
  }
  else {
    stopCar();  //stop
  }
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
  // Right motor
  if (rightMotorSpeed < 0) {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  }
  else if (rightMotorSpeed > 0) {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  }
  else {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, HIGH);
  }

  // Left motor
  if (leftMotorSpeed < 0) {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
  }
  else if (leftMotorSpeed > 0) {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
  }
  else {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);
  }

  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));
}

long readDistance() {
  digitalWrite(trigPin, LOW); 
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); 
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long time = pulseIn (echo, HIGH);
  return time / 29 / 2;
}

void servoPulse (int pin, int angle){
int pwm = (angle*11) + 500;      // Convert angle to microseconds
 digitalWrite(pin, HIGH);
 delayMicroseconds(pwm);
 digitalWrite(pin, LOW);
 delay(50); // Refresh cycle of servo
}

void avoidObject(){
  Check_side();
  turnLeft();
  delay(660);
  stopCar();
  delay(1000);
  moveForward();
  delay(500);

  turnRight();
  delay(560);
  stopCar();
  delay(1000);
  moveForward();
  delay(820);

  turnRight();
  delay(550);
  stopCar();
  delay(1000);
  moveForward();

  turnLeft();
  delay(630);
  stopCar();
  delay(1000);
  moveForward();
}

//=======================

void Check_side(){
    stopCar();
    delay(100);
 for (int angle = 70; angle <= 140; angle += 5)  {
   servoPulse(servo, angle);  }
    delay(300);
    right_dis = readDistance();
    // Serial.print("D R=");Serial.println(distance_R);
    delay(100);
  for (int angle = 140; angle >= 0; angle -= 5)  {
   servoPulse(servo, angle);  }
    delay(500);
    left_dis = readDistance();
    // Serial.print("D L=");Serial.println(distance_L);
    delay(100);
 for (int angle = 0; angle <= 70; angle += 5)  {
   servoPulse(servo, angle);  }
    delay(300);
}


void moveForward(){
  rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
}
void turnRight(){
  rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);
}
void turnLeft(){
  rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
}
void stopCar(){
  rotateMotor(0, 0);
}
