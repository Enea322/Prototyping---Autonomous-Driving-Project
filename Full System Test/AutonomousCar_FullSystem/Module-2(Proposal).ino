// Object avoidance logic credited to Felix

#include <Servo.h>

#define IR_SENSOR_RIGHT A0
#define IR_SENSOR_LEFT A1
int MOTOR_SPEED = 80;

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
const int servoPin = 11;
Servo servo;

void servoPulse(Servo &s, int angle) {
  s.write(angle);
}
int left_dis, right_dis, front_dis, angle;


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

  // Initially stop
  rotateMotor(0, 0);
}

void loop() {
  
  // IR sensor reading
  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);
  
  servo.attach(servoPin);
  long distance = readDistance();
  Serial.print("Distance: "); Serial.print(distance); Serial.println(" cm");

  if (distance <= 10 && distance > 0) {
    // Object detected too close
    checkSide();
    if(left_dis < right_dis){
      MOTOR_SPEED = 65;
      for(angle=0;angle <= 70;angle += 10){
        servo.write(angle);
      while(front_dis < 15){
        turnRight();
        delay(50);
        stopCar();
        front_dis = readDistance();
      }
      }
      rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
      while(rightIRSensorValue == LOW) {
      while(front_dis < 15){
        moveForward();
        delay(50);
        stopCar();
        front_dis = readDistance();
      }
      while(front_dis > 15){
        turnLeft();
        delay(50);
        stopCar();
        front_dis = readDistance();
      }
        rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
      }
      stopCar();
      leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);
      while(leftIRSensorValue == HIGH){
        rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
        while(rightIRSensorValue == HIGH){
          turnLeft();
          rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
        }
        stopCar();
        rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
        while(rightIRSensorValue == LOW){
          moveForward();
          rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
        }
        stopCar();
        leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);
      }
    }
    else{
      MOTOR_SPEED = 65;
      for(angle=0;angle <= 70;angle += 10){
        servo.write(angle);
      while(front_dis < 15){
        turnLeft();
        delay(50);
        stopCar();
        front_dis = readDistance();
      }
      }
      leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);
      while(leftIRSensorValue == LOW) {
      while(front_dis < 15){
        moveForward();
        delay(50);
        stopCar();
        front_dis = readDistance();
      }
      while(front_dis > 15){
        turnRight();
        delay(50);
        stopCar();
        front_dis = readDistance();
      }
        leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);
      }
      stopCar();
      rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
      while(rightIRSensorValue == HIGH){
        leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);
        while(leftIRSensorValue == HIGH){
          turnRight();
          leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);
        }
        stopCar();
        leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);
        while(leftIRSensorValue == LOW){
          moveForward();
          leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);
        }
        stopCar();
        rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
      }
    }
    rotateMotor(0, 0);
    delay(200);
    return; // skip line-following if object is ahead
    MOTOR_SPEED = 80;
    servo.write(90);
  }

  

  // Line following logic
  if (rightIRSensorValue == LOW && leftIRSensorValue == LOW) {
    rotateMotor(MOTOR_SPEED, MOTOR_SPEED);  // Go straight
  }
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW) {
    rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);  // Turn right
  }
  else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH) {
    rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);  // Turn left
  }
  else {
    rotateMotor(0, 0);  //stop
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
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);  // timeout
  if (duration == 0) return -1;  // sensor timeout
  return duration * 0.034 / 2;   // convert to cm
}

void checkSide(){
    stopCar();
    delay(100);
    for (int angle = 70; angle <= 140; angle += 5) {
    servo.write(angle);
    }
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
 for (int angle = 0; angle <= 70; angle += 5) {
        servo.write(angle);
    delay(20);
    }
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
