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
#define IR_LEFT_PIN A0
#define IR_RIGHT_PIN A1

// Color Sensor
#define S0 A2
#define S1 A3
#define S2 A4
#define S3 A5
#define sensorOut 10

// Color Sensor Variables
int t = 0;
int redMin = 0;
int redMax = 1;
int redColor = 0;
float redFrequency = 0;
float redAvg = 0;
int redAmbient = 0;
float redEdgeTime = 0;
float redSum = 0;
bool ambientChecked = false;
bool red = false;

void setup() {
  Serial.begin(9600);

  // Ultrasonic
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Motors
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);

  // Servo
  servo.attach(servoPin);

  // IR
  pinMode(IR_LEFT_PIN, INPUT);
  pinMode(IR_RIGHT_PIN, INPUT);

  // Color Sensor
  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  digitalWrite(S0, LOW);
  digitalWrite(S1, HIGH); // 2% scaling
}

long readDistance() {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  return duration * 0.034 / 2;
}

void moveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, 50);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(ENB, 50);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN3, LOW);
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

void loop() {
  long distance = readDistance();
  Serial.print("Distance: "); Serial.print(distance); Serial.println(" cm");

  if (distance > 0 && distance <= 10) stopMotors();
  else moveForward();

  int left = digitalRead(IR_LEFT_PIN);
  int right = digitalRead(IR_RIGHT_PIN);
  
  if (left == 1 && right == 1) Serial.println("Both sensors on line");
  else if (left == 1 && right == 0) Serial.println("Left sensor on line");
  else if (left == 0 && right == 1) Serial.println("Right sensor on line");
  else if (left == 0 && right == 0) {
    Serial.println("Line not detected");

    Serial.println("------------------------------");

    if (!ambientChecked) {
      for (t = 0, redSum = 0; t <= 10; t++) {
        digitalWrite(S2, LOW); digitalWrite(S3, LOW);
        redEdgeTime = pulseIn(sensorOut, HIGH) + pulseIn(sensorOut, LOW);
        redFrequency = 1.0 / (redEdgeTime / 1000000.0);
        redSum += redFrequency;
      }
      redAmbient = redSum / 10;
      ambientChecked = true;
    }

    for (t = 0, redSum = 0; t <= 10; t++) {
      digitalWrite(S2, LOW); digitalWrite(S3, LOW);
      redEdgeTime = pulseIn(sensorOut, HIGH) + pulseIn(sensorOut, LOW);
      redFrequency = 1.0 / (redEdgeTime / 1000000.0);
      redSum += redFrequency;
    }
    redAvg = redSum / 10;

    red = (redAvg > redAmbient);

    Serial.print("Average red detected: ");
    Serial.println(redAvg);
    Serial.println("------------------------------");
    delay(100);

    if (red) Serial.println("Red detected");
    else Serial.println("Green detected");

    Serial.println("------------------------------");
    delay(1000);
  }
}
