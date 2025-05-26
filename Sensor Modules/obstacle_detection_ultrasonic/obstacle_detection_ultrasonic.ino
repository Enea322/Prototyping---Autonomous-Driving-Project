/*
Ultrasonic Sensor code v2, where the ultrasonic sensor decides when to stop/spin the motors.

-ENA, ENB (enable A/B): for speed (PWM)
-IN1, IN2: motor A direction
-IN3, IN4: motor B direction
*/

#define trigPin 13
#define echoPin 12

#define IN1 9
#define IN2 8
#define ENA 3

#define IN3 7
#define IN4 5
#define ENB 2

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
}

long readDistance() {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000); // Timeout after 30ms
  return duration * 0.034 / 2;
}

void moveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, 50);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); analogWrite(ENB, 50);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN3, LOW);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
}

void loop() {
  long distance = readDistance();
  Serial.print("Distance: "); Serial.print(distance); Serial.println(" cm");

  if (distance > 0 && distance <= 10) stopMotors();
  else moveForward();
}
