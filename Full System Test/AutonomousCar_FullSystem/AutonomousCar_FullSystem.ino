// Combined Arduino Code v3
// Includes: Ultrasonic obstacle detection, servo motor, IR line detection, and color sensor

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
#define MOTOR_SPEED 80

// Right motor
int enableRightMotor = 2;
int rightMotorPin1 = 7;
int rightMotorPin2 = 5;

// Left motor
int enableLeftMotor = 3;
int leftMotorPin1 = 9;
int leftMotorPin2 = 8;

// Color Sensor
#define S0 A2
#define S1 A3
#define S2 A4
#define S3 A5
#define sensorOut 10

// Color Sensor Calibration Values
int t = 0;
int redMin = 0;
int redMax = 1;
int redColor = 0;
int redFrequency = 0;
int redAvg = 0;
int redAmbient = 0;
int redEdgeTime = 0;
float redSum = 0;
bool ambientChecked = false;
bool red = false;

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

  rotateMotor(0, 0);
}

// --- Functions ---
long readDistance() {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  return duration * 0.034 / 2;
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
  // Right motor control
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

  // Left motor control
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

void scanWithServo() {
  for (angle = 0; angle <= 180; angle += step) {
    servo.write(angle);
    delay(10);
  }
  for (angle = 180; angle >= 0; angle -= step) {
    servo.write(angle);
    delay(10);
  }
}

// --- Loop ---
void loop() {
  long distance = readDistance();
  Serial.print("Distance: "); Serial.print(distance); Serial.println(" cm");

  if (distance > 0 && distance <= 10) rotateMotor(0, 0);

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

    Serial.println("------------------------------");
    if (!ambientChecked) {
      for (t = 0, redFrequency = 0, redSum = 0; t <= 10; t++) {
        digitalWrite(S2, LOW);
        digitalWrite(S3, LOW);
        redEdgeTime = pulseIn(sensorOut, HIGH) + pulseIn(sensorOut, LOW);
        redFrequency = (1 / (redEdgeTime / 1000000.0));
        redSum += redFrequency;
      }
      redAmbient = redSum / 10;
      ambientChecked = true;
    }

    for (t = 0, redFrequency = 0, redSum = 0; t <= 10; t++) {
      digitalWrite(S2, LOW);
      digitalWrite(S3, LOW);
      redEdgeTime = pulseIn(sensorOut, HIGH) + pulseIn(sensorOut, LOW);
      redFrequency = (1 / (redEdgeTime / 1000000.0));
      redSum += redFrequency;
    }
    redAvg = redSum / 10;
    red = redAvg > redAmbient;

    Serial.print("Average red detected: ");
    Serial.println(redAvg);
    Serial.println("------------------------------");
    delay(100);

    if (red) Serial.println("Red detected");
    else Serial.println("Green detected");

    Serial.println("------------------------------");
    delay(1000);
  }

  delay(100);
}
