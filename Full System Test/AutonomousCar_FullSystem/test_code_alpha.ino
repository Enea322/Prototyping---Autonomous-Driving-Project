// Combined Arduino Code v4
// Includes: Line following, obstacle detection, color detection, and special maneuvers

#include <Servo.h>

// --- Pin Definitions ---

// Ultrasonic Sensor
#define trigPin 13
#define echoPin 12

// Servo Motor
#define servoPin 11
Servo servo;
int angle = 0;
int step = 5; // degrees per step

// IR Sensors
#define IR_SENSOR_RIGHT A0
#define IR_SENSOR_LEFT A1

// Motor Pins
int enableRightMotor = 2;
int rightMotorPin1 = 7;
int rightMotorPin2 = 5;
int enableLeftMotor = 3;
int leftMotorPin1 = 9;
int leftMotorPin2 = 8;

// Color Sensor
#define S0 A2
#define S1 A3
#define S2 A4
#define S3 A5
#define sensorOut 10

// Motor speed and timings
const int MOTOR_SPEED = 80;
const int FORWARD_DURATION_SHORT = 1500;
const int FORWARD_DURATION_LONG  = 2000;
const int FORWARD_DURATION_UTURN = 3000;
const int TURN_90_DURATION       = 250;
const int UTURN_DURATION         = 700;
const int SERVO_DELAY            = 300;
const int SERVO_RESET_DELAY      = 200;

// Color Sensor Calibration
int t = 0;
int redFrequency = 0;
int redAvg = 0;
int redAmbient = 0;
float redSum = 0;
bool red = false;
bool ambientChecked = false;

// --- Setup ---
void setup() {
  Serial.begin(9600);

  // Ultrasonic
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Motor pins
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  // IR Sensors
  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);

  // Servo
  servo.attach(servoPin);
  delay(500);

  // Color Sensor
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  digitalWrite(S0, LOW);
  digitalWrite(S1, HIGH);

}

// --- Loop ---
void loop() {
  
  if (!ambientChecked) {
      for (t = 0, redSum = 0; t <= 10; t++) {
        digitalWrite(S2, LOW);
        digitalWrite(S3, LOW);
        int redEdgeTime = pulseIn(sensorOut, HIGH) + pulseIn(sensorOut, LOW);
        redFrequency = (1 / (redEdgeTime / 1000000.0));
        redSum += redFrequency;
      }
      redAmbient = redSum / 10;
      ambientChecked = true;
    }
  
  long distance = readDistance();
  Serial.print("Distance: "); Serial.print(distance); Serial.println(" cm");

  if (distance > 0 && distance <= 10) {
    
    rotateMotor(0, 0);
    
    Serial.println("------------------------------");
    

    for (t = 0, redSum = 0; t <= 10; t++) {
      digitalWrite(S2, LOW);
      digitalWrite(S3, LOW);
      int redEdgeTime = pulseIn(sensorOut, HIGH) + pulseIn(sensorOut, LOW);
      redFrequency = (1 / (redEdgeTime / 1000000.0));
      redSum += redFrequency;
    }
    redAvg = redSum / 10;
    red = redAvg > redAmbient;

    Serial.print("Average red detected: ");
    Serial.println(redAvg);
    delay(100);

    if (red) {
      Serial.println("Red detected");

      bool leftClear = isLeftClear();
      bool rightClear = isRightClear();

      if (rightClear && !leftClear) {
        turnRight90();
        moveForward(MOTOR_SPEED, FORWARD_DURATION_SHORT);
        turnLeft90();
        moveForward(MOTOR_SPEED, FORWARD_DURATION_LONG);
        turnLeft90();
        moveUntilLine();
      }
      else if (!rightClear && leftClear) {
        turnLeft90();
        moveForward(MOTOR_SPEED, FORWARD_DURATION_SHORT);
        turnRight90();
        moveForward(MOTOR_SPEED, FORWARD_DURATION_LONG);
        turnRight90();
        moveUntilLine();
      }
      else {
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
      ambientChecked = false;
    }
    else {
      Serial.println("Green detected");
    }
    Serial.println("------------------------------");
    delay(1000);
    rotateMotor(0, 0);
  }
    
  

  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);

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
    rotateMotor(0, 0);  // Stop

    
  delay(100);
}
}

// --- Function Definitions ---
long readDistance() {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  return duration * 0.034 / 2;
}

bool isLeftClear() {
  servo.write(150);
  delay(SERVO_DELAY);
  long distance = readDistance();
  servo.write(90);
  delay(SERVO_RESET_DELAY);
  return distance > 10;
}

bool isRightClear() {
  servo.write(30);
  delay(SERVO_DELAY);
  long distance = readDistance();
  servo.write(90);
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
  millis(TURN_90_DURATION);
  rotateMotor(0, 0);
}

void turnRight90() {
  rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
  millis(TURN_90_DURATION);
  rotateMotor(0, 0);
}

void uTurn() {
  rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
  while(true){
    if(rightIR == LOW) break;
  }
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
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));
}
