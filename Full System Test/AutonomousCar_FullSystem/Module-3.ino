#include <Servo.h>

#define IR_SENSOR_RIGHT A0
#define IR_SENSOR_LEFT  A1
#define MOTOR_SPEED     80
#define TURN_SPEED      65

// Right motor
int enableRightMotor = 2;
int rightMotorPin1   = 7;
int rightMotorPin2   = 5;

// Left motor
int enableLeftMotor  = 3;
int leftMotorPin1    = 9;
int leftMotorPin2    = 8;

// Ultrasonic pins
#define trigPin 13
#define echoPin 12

// Servo pin
const int servo = 11;

// ===== Globals & State =====
enum LastSeen { NONE, LEFT, RIGHT };
LastSeen lastSeen    = NONE;
unsigned long lostTimer = 0;

int safe_dis = 10; // in cm
int left_dis, right_dis, front_dis;

// New flag: have we already started avoiding this obstacle?
bool avoiding = false;

void setup() {
  Serial.begin(9600);

  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT,  INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(servo, OUTPUT);

  servoPulse(servo, 90);
  delay(200);

  // ultrasonic warm-up
  for (int i = 0; i < 3; i++) { readDistance(); delay(50); }

  rotateMotor(0, 0);
  delay(500);
}

void loop() {
  front_dis = readDistance();
  //Serial.print("Front: "); Serial.print(front_dis); Serial.println(" cm");

  // If we're not already avoiding...
  if (!avoiding) {
    // and we see a real obstacle
    if (front_dis > 0 && front_dis <= safe_dis) {
      avoiding = true;
      stopCar();
      delay(200);
      avoidObject();        // does Check_side() + turn
      // leave avoiding==true so we don’t re-scan immediately
    }
    else {
      // normal line follow
      followLine();
    }
  }
  else {
    // we’re in “clearing” mode: keep going forward until path is clear
    if (front_dis == 0 || front_dis > safe_dis) {
      avoiding = false;        // done avoiding
      servoPulse(servo, 90);   // re-center servo
      delay(200);
    }
    else {
      followLine();
    }
  }
}

// ===== followLine() with overshoot detection & recovery =====
void followLine() {
  bool rightIR = (digitalRead(IR_SENSOR_RIGHT) == HIGH);
  bool leftIR  = (digitalRead(IR_SENSOR_LEFT ) == HIGH);

  // 1) Right sensor on black - steer right
  if ( rightIR && !leftIR ) {
      lastSeen   = RIGHT;
      turnRight();
      lostTimer  = 0;                // reset coast timer
  }
  // 2) Left sensor on black - steer left
  else if ( !rightIR && leftIR ) {
      lastSeen   = LEFT;
      turnLeft();
      lostTimer  = 0;
  }
  // 3) Both sensors see white - could be centred or overshoot
  else if ( !rightIR && !leftIR ) {
    // start the short “coast” timer on first entry
    if ( lostTimer == 0 ) lostTimer = millis();

        moveForward();                // coast straight

    // if coast exceeds threshold, we’ve truly lost the line
    if ( millis() - lostTimer > 80 ) {
        recoverLine();
        lostTimer = 0;
    }
  }
  // 4) (both HIGH — unlikely) just go forward
  else {
    moveForward();
    lastSeen  = NONE;
    lostTimer = 0;
  }
}

// ===== recoverLine() to re-acquire the track =====
void recoverLine() {
  stopCar();
  delay(100);

  if ( lastSeen == RIGHT ) {
    // Case: overshot to the left - pivot right back onto line
    turnRight();
    while ( digitalRead(IR_SENSOR_RIGHT) == LOW ) { }
    stopCar();
    delay(100);

    turnRight();
    while ( digitalRead(IR_SENSOR_RIGHT) == HIGH ) { }
    stopCar();
    delay(100);
  }
  else if ( lastSeen == LEFT ) {
    // Case: overshot to the right - pivot left back onto line
    turnLeft();
    while ( digitalRead(IR_SENSOR_LEFT) == LOW ) { }
    stopCar();
    delay(100);

    turnLeft();
    while ( digitalRead(IR_SENSOR_LEFT) == HIGH ) { }
    stopCar();
    delay(100);
  }
  else {
    // No history: back up a bit then try again
    turnBackward();
    delay(200);
    stopCar();
    delay(100);
  }

  // now centered again - go forward to adjust onto the line
  moveForward();
  delay(100);
  stopCar();
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
  analogWrite(enableLeftMotor,  abs(leftMotorSpeed));
}

long readDistance() {
  digitalWrite(trigPin, LOW); 
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); 
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long t = pulseIn(echoPin, HIGH, 30000);
  return t ? t / 29 / 2 : 0;
}

void servoPulse(int pin, int angle) {
  int pwm = (angle * 11) + 500;
  digitalWrite(pin, HIGH);
  delayMicroseconds(pwm);
  digitalWrite(pin, LOW);
  delay(50);
}

void avoidObject() {
  // 1) scan both sides
  Check_side();

  // 2) if right side is clearer than left, steer right
  if (right_dis >= left_dis) {
    // back up a bit
    turnBackward();
    delay(500);
    stopCar();
    delay(500);

    // pivot right
    turnRight();
    delay(300);
    stopCar();
    delay(1000);

    // move forward past the obstacle
    turnForward();
    delay(650);
    stopCar();
    delay(500);

    // swing back left to rejoin the line
    turnLeft();
    delay(500);
    stopCar();
    delay(500);

    // go forward until right IR sees BLACK (HIGH)
    turnForward();
    while (digitalRead(IR_SENSOR_RIGHT) == LOW) { }
    stopCar();
    delay(500);

    // pivot right until right IR sees WHITE (LOW) again
    turnRight();
    while (digitalRead(IR_SENSOR_RIGHT) == HIGH) { }
    stopCar();
    delay(500);
  }
  // 3) otherwise obstacle is on right, so steer left
  else {
    // back up a bit
    turnBackward();
    delay(500);
    stopCar();
    delay(500);

    // pivot left
    turnLeft();
    delay(300);
    stopCar();
    delay(1000);

    // move forward past the obstacle
    turnForward();
    delay(650);
    stopCar();
    delay(500);

    // swing back right to rejoin the line
    turnRight();
    delay(500);
    stopCar();
    delay(500);

    // go forward until left IR sees BLACK (HIGH)
    turnForward();
    while (digitalRead(IR_SENSOR_LEFT) == LOW) { }
    stopCar();
    delay(500);

    // pivot left until left IR sees WHITE (LOW) again
    turnLeft();
    while (digitalRead(IR_SENSOR_LEFT) == HIGH) { }
    stopCar();
    delay(500);
  }
}

void Check_side() {
  stopCar(); delay(100);
  for (int angle = 70; angle <= 140; angle += 5) servoPulse(servo, angle);
  delay(300);
  right_dis = readDistance();
  delay(100);
  for (int angle = 140; angle >= 0; angle -= 5) servoPulse(servo, angle);
  delay(500);
  left_dis = readDistance();
  delay(100);
  for (int angle = 0; angle <= 70; angle += 5) servoPulse(servo, angle);
  delay(300);
  Serial.print("R="); Serial.print(right_dis);
  Serial.print("  L="); Serial.println(left_dis);
}

void moveForward() { rotateMotor(MOTOR_SPEED,  MOTOR_SPEED); }
void turnRight()   { rotateMotor(-MOTOR_SPEED, MOTOR_SPEED); }
void turnLeft()    { rotateMotor(MOTOR_SPEED, -MOTOR_SPEED); }
void stopCar()     { rotateMotor(0, 0); }

void turnForward() { rotateMotor(TURN_SPEED,  TURN_SPEED); }
void turnBackward() { rotateMotor(-TURN_SPEED,  -TURN_SPEED); }


