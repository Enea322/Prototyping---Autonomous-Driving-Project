// Combined Arduino Code v1
// Includes: Ultrasonic obstacle detection, servo motor, IR line detection, and color sensor

#include <Servo.h>

// --- Pin Definitions ---

// Ultrasonic Sensor
#define trigPin 13
#define echoPin 12

// Motor Driver
#define IN1 9
#define IN2 8
#define ENA 6
#define IN3 7
#define IN4 5
#define ENB 4

// Servo Motor
#define servoPin 11
Servo servo;
int angle = 0;
int step = 5;

// IR Sensors
#define IR_LEFT_PIN A0
#define IR_RIGHT_PIN A1

// Color Sensor
#define S0 A2
#define S1 A3
#define S2 A4
#define S3 A5
#define sensorOut 10

// Color Sensor Calibration Values (adjust as needed)
int redMin = 0, redMax = 1;
int greenMin = 0, greenMax = 1;
int redColor = 0, greenColor = 0;

enum Color { RED, GREEN, NONE };
Color currentColor = NONE;

// --- Setup ---
void setup() {
  Serial.begin(9600);

  // Ultrasonic pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Motor pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);

  // Servo
  servo.attach(servoPin);

  // IR Sensors
  pinMode(IR_LEFT_PIN, INPUT);
  pinMode(IR_RIGHT_PIN, INPUT);

  // Color Sensor
  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, HIGH);
}

// --- Functions ---

long readDistance() {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH) * 0.034 / 2;
}

void moveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, 100);  // Speed 0-255
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, 100);
}

void stopMotors() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
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

void readColorSensor() {
  digitalWrite(S2, LOW); digitalWrite(S3, LOW);
  float redEdgeTime = pulseIn(sensorOut, HIGH) + pulseIn(sensorOut, LOW);
  float redFreq = 1 / (redEdgeTime / 1000000);
  redColor = map(redFreq, redMax, redMin, 255, 0);
  redColor = constrain(redColor, 0, 255);

  digitalWrite(S2, HIGH); digitalWrite(S3, HIGH);
  float greenEdgeTime = pulseIn(sensorOut, HIGH) + pulseIn(sensorOut, LOW);
  float greenFreq = 1 / (greenEdgeTime / 1000000);
  greenColor = map(greenFreq, greenMax, greenMin, 255, 0);
  greenColor = constrain(greenColor, 0, 255);

  if (redColor > greenColor) currentColor = RED;
  else if (greenColor > redColor) currentColor = GREEN;
  else currentColor = NONE;
}

bool isLineDetected() {
  int left = digitalRead(IR_LEFT_PIN);
  int right = digitalRead(IR_RIGHT_PIN);
  return (left == LOW || right == LOW);  // Detects black line
}

bool isEndOfLine() {
  int left = digitalRead(IR_LEFT_PIN);
  int right = digitalRead(IR_RIGHT_PIN);
  return (left == HIGH && right == HIGH);  // White surface
}

// --- Loop ---
void loop() {
  long distance = readDistance();
  scanWithServo();
  readColorSensor();

  Serial.print("Distance: "); Serial.println(distance);
  Serial.print("Color: ");
  if (currentColor == RED) Serial.println("RED");
  else if (currentColor == GREEN) Serial.println("GREEN");
  else Serial.println("NONE");

  Serial.print("Line: ");
  if (isLineDetected()) Serial.println("Detected");
  else if (isEndOfLine()) Serial.println("End of Line");
  else Serial.println("No line");

  if (distance <= 10) stopMotors();
  else moveForward();

  delay(100);
}