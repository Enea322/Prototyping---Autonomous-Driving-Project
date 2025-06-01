// IR Sensor Pins
#define IR_LEFT A0    
#define IR_RIGHT A1   

// Motor Driver Pins
// Left Motor
#define IN1 9   
#define IN2 8  
#define ENA 3
// Right Motor
#define IN3 7  
#define IN4 5  
#define ENB 2  

void setup() {
  // Set IR pins
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  // Set motor control pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  Serial.begin(9600);
  delay(500);
}

// Movement Functions
void moveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, 100);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, 100);
}

void turnLeft() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA, 0);     // Stop left motor
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, 100);  // Move right motor forward
}

void turnRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, 100);  // Move left motor forward
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(ENB, 0);     // Stop right motor
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA, 0);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(ENB, 0);
}

void loop() {
  int left = digitalRead(IR_LEFT);
  int right = digitalRead(IR_RIGHT);

  // Combining readings into a 2-bit code for better logical switch case
  int state = (left << 1) | right;

  switch(state) {
    case 0b11: // 3: both sensors on the line
      Serial.println("move forward");
      moveForward();
      break;

    case 0b10: // 2: left on line, right not
      Serial.println("turning left for Right to find the line");
      turnLeft();
      break;

    case 0b01: // 1: right on line, left not
      Serial.println("turning right for Left to find the line");
      turnRight());
      break;

    case 0b00: // 0: both off line
      Serial.println("No line detected");
      stopMotors();
      break;
  }

  delay(100); // avoiding rapid switching
}
