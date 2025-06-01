// Combined Arduino Code v2
// Includes: Ultrasonic obstacle detection, servo motor, IR line detection, and color sensor

#include <Servo.h>

// --- Pin Definitions ---

// Ultrasonic Sensor
#define trigPin 13
#define echoPin 12

// Motor Driver
#define IN1 9
#define IN2 8
#define ENA 3
#define IN3 7
#define IN4 5
#define ENB 2

// Servo Motor
#define servoPin 11
Servo servo;
int angle = 0;
int step = 5; // degrees per step

// IR Sensors
#define IR_LEFT A0
#define IR_RIGHT A1

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
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Servo
  servo.attach(servoPin);

  // IR Sensors
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
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

// --- Functions ---
long readDistance() {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  return duration * 0.034 / 2;
}

void moveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, 50);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, 50);
}

void turnLeft() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA, 0);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, 50);
}

void turnRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, 50);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(ENB, 0);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA, 0);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(ENB, 0);
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

  if (distance > 0 && distance <= 10) stopMotors();
  else moveForward();

  int left = digitalRead(IR_LEFT);
  int right = digitalRead(IR_RIGHT);
  int state = (left << 1) | right;

  switch(state) {
    case 0b11:
      Serial.println("move forward");
      moveForward();
      break;
    case 0b10:
      Serial.println("turning left for Right to find the line");
      turnLeft();
      break;
    case 0b01:
      Serial.println("turning right for Left to find the line");
      turnRight();
      break;
    case 0b00:
      Serial.println("No line detected");
      stopMotors();

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
      break;
  }

  delay(100);
}
