/*
Ultrasonic Sensor code v1, where the ultrasonic sensor decides when to stop/spin the motors.

-ENA, ENB (enable A/B): for speed (PWM)
-IN1, IN2: motor A direction
-IN3, IN4: motor B direction
*/

// C++ Code
#define trigPin 13
#define echoPin 12

#define IN1 7
#define IN2 6
#define ENA 5

#define IN3 4
#define IN4 3
#define ENB 2

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
}

long readDistance() {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 100);  // Speed 0-255

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 100);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void loop() {
  long distance = readDistance();
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance <= 10) {
    stopMotors();
  } else {
    moveForward();
  }

  delay(100);
}
