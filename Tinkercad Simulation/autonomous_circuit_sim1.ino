// C++ Code


/*
#############################################################################
For more details please go to the file CODE_EXPLANATION.md on the repository.
#############################################################################
*/

// Define pins
const int sensorPin = A0;      // IR sensor (photoresistor)
const int ledPin = 4;          // LED pin for IR sensor feedback
const int trigPin = 7;         
const int echoPin = 6;         
const int IN1 = 9;
const int IN2 = 8;
const int IN3 = 11;
const int IN4 = 10;
const int EN1_2 = 2; // Enable 1&2
const int EN3_4 = 3; // Enable 3&4

// 
int sensorValue = 0;
long duration;
int distance;

void setup() {
  // Set up motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(EN1_2, OUTPUT);
  pinMode(EN3_4, OUTPUT);

  // Enable motors
  digitalWrite(EN1_2, HIGH);
  digitalWrite(EN3_4, HIGH);

  // Set up IR sensor and LED
  pinMode(sensorPin, INPUT);
  pinMode(ledPin, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.begin(9600);
}

void loop() {
  // IR sensor for black/white surface detection
  sensorValue = analogRead(sensorPin);
  Serial.println(sensorValue);  

  if (sensorValue < 400) {
    digitalWrite(ledPin, LOW);  // Black surface
  } else {
    digitalWrite(ledPin, HIGH); // White surface
  }

  // Ultrasonic distance measurement
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 30000);
if (duration == 0) {
  distance = 999; // Sensor failed, no reading
} else {
  distance = duration * 0.034 / 2;
}


  Serial.print("Distance: ");
  Serial.println(distance);

  // Motor 1 control (bottom)
  if (distance < 20) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }

  // Motor 2 always runs (top)
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  delay(100);
}
