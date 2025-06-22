// ===== Stable Line‑Following & Object‑Handling =====
// Re‑worked sketch implementing the feedback recommendations:
//   • Uses the Servo library (non‑blocking refresh)
//   • Adds timeouts to all sensor‑wait loops
//   • Reduces blocking delays
//   • Uses const/constexpr for pins & parameters
//   • Adds margin to colour detection
//   • Consistent naming & organisation
// ------------------------------------------------------------

#include <Servo.h>

// ===== Compile‑time Configuration ======
// IR sensors (digital‑level outputs)
constexpr uint8_t IR_SENSOR_RIGHT = A0;
constexpr uint8_t IR_SENSOR_LEFT  = A1;

// Motor driver (EN ‑ IN1 ‑ IN2)
// Right
constexpr uint8_t EN_RIGHT  = 2;
constexpr uint8_t R_IN1     = 7;
constexpr uint8_t R_IN2     = 5;
// Left
constexpr uint8_t EN_LEFT   = 3;
constexpr uint8_t L_IN1     = 9;
constexpr uint8_t L_IN2     = 8;

// Motion parameters (PWM)
constexpr uint8_t CRUISE_PWM = 80;   // straight line speed
constexpr uint8_t TURN_PWM   = 65;   // turn speed

// Ultrasonic
constexpr uint8_t TRIG_PIN = 13;
constexpr uint8_t ECHO_PIN = 12;
constexpr uint16_t SONIC_TIMEOUT_US = 30000;   // 5 m

// Servo (for ultrasonic scan)
constexpr uint8_t SERVO_PIN = 11;
constexpr uint8_t SERVO_FWD = 90;    // centre
constexpr uint8_t SERVO_LEFT = 40;
constexpr uint8_t SERVO_RIGHT = 140;

// TCS3200 colour sensor
constexpr uint8_t S0 = A2;
constexpr uint8_t S1 = A3;
constexpr uint8_t S2 = A4;
constexpr uint8_t S3 = A5;
constexpr uint8_t TCS_OUT = 10;

// Safety / behaviour constants
constexpr uint8_t SAFE_DISTANCE_CM    = 10;     // stop / avoid threshold
constexpr float   RED_THRESHOLD_FACTOR = 1.30f; // % above ambient red
constexpr uint16_t LOST_LINE_MS       = 80;     // how long both sensors LOW before recovery
constexpr uint16_t SENSOR_WAIT_MS     = 1000;   // timeout for while‑loops

// ===== State =====
enum LastSeen : uint8_t { NONE, LEFT, RIGHT };
LastSeen lastSeen = NONE;
unsigned long lostTimer = 0;
bool avoiding = false;

int  frontDist = 0;
int  leftDist  = 0;
int  rightDist = 0;

// Colour sensor baseline
float redAmbient = 0.0f;

// ===== Objects =====
Servo sonarServo;

// ===== Forward Declarations =====
void followLine();
void recoverLine();
void rotateMotors(int8_t rightPWM, int8_t leftPWM);
int  readDistanceCm();
void avoidObject();
void removeObject();
void scanSides();
bool isRedObject();
void calibrateColourSensor();

// ===== Arduino Setup =======
void setup() {
  Serial.begin(115200);

  // GPIO setup
  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT,  INPUT);

  pinMode(EN_RIGHT, OUTPUT);
  pinMode(R_IN1,    OUTPUT);
  pinMode(R_IN2,    OUTPUT);
  pinMode(EN_LEFT,  OUTPUT);
  pinMode(L_IN1,    OUTPUT);
  pinMode(L_IN2,    OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(S0, OUTPUT);  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);  pinMode(S3, OUTPUT);
  pinMode(TCS_OUT, INPUT);
  
  digitalWrite(S0, LOW);
  digitalWrite(S1, HIGH);

  // Attach & centre servo
  sonarServo.attach(SERVO_PIN);
  sonarServo.write(SERVO_FWD);
  delay(300);

  // Warm‑up ultrasonic
  for (uint8_t i = 0; i < 3; ++i) {
    readDistanceCm();
    delay(30);
  }

  // Stop motors
  rotateMotors(0, 0);

  // Calibrate colour sensor baseline
  calibrateColourSensor();

  Serial.println(F("Robot ready."));
}

// ===== Arduino Loop ======
void loop() {
  frontDist = readDistanceCm();

  if (!avoiding) {
    if (frontDist > 0 && frontDist <= SAFE_DISTANCE_CM) {
      // Possible obstacle ahead
      rotateMotors(0, 0);
      delay(100);
      avoiding = true;      // tentative

      if (isRedObject()) {
        avoidObject();      // leave obstacle in place
      } else {
        removeObject();     // push obstacle away
      }
      avoiding = false;     // either branch ends with clear path attempt
    } else {
      followLine();
    }
  } else {
    // In avoidance manoeuvre - keep following line until clear
    if (frontDist == 0 || frontDist > SAFE_DISTANCE_CM) {
      avoiding = false;
    }
    followLine();
  }
}

// ===== Line Following ======
void followLine() {
  bool rightHigh = digitalRead(IR_SENSOR_RIGHT);
  bool leftHigh  = digitalRead(IR_SENSOR_LEFT);

  if (rightHigh && !leftHigh) {
    lastSeen = RIGHT;
    rotateMotors(-TURN_PWM, TURN_PWM);   // pivot right
    lostTimer = 0;
  } else if (!rightHigh && leftHigh) {
    lastSeen = LEFT;
    rotateMotors(TURN_PWM, -TURN_PWM);   // pivot left
    lostTimer = 0;
  } else if (!rightHigh && !leftHigh) {
    if (lostTimer == 0) lostTimer = millis();
    rotateMotors(CRUISE_PWM, CRUISE_PWM);

    if (millis() - lostTimer > LOST_LINE_MS) {
      recoverLine();
      lostTimer = 0;
    }
  } else {
    // Both sensors see line – go straight
    rotateMotors(CRUISE_PWM, CRUISE_PWM);
    lastSeen = NONE;
    lostTimer = 0;
  }
}

void recoverLine() {
  rotateMotors(0, 0);
  delay(40);
  unsigned long start;

  if (lastSeen == RIGHT) {
    // Steer right until sensor re‑detects line
    start = millis();
    while (!digitalRead(IR_SENSOR_RIGHT) && millis() - start < SENSOR_WAIT_MS) {
      rotateMotors(-TURN_PWM, TURN_PWM);
    }
  } else if (lastSeen == LEFT) {
    // Steer left until sensor re‑detects line
    start = millis();
    while (!digitalRead(IR_SENSOR_LEFT) && millis() - start < SENSOR_WAIT_MS) {
      rotateMotors(TURN_PWM, -TURN_PWM);
    }
  } else {
    // Unknown direction – small reverse & retry
    rotateMotors(-TURN_PWM, -TURN_PWM);
    delay(200);
  }
  rotateMotors(0, 0);
  delay(60);
}

// ===== Motors (signed PWM helpers) =======
void rotateMotors(int8_t rightPWM, int8_t leftPWM) {
  // Right wheel direction
  digitalWrite(R_IN1, rightPWM < 0);
  digitalWrite(R_IN2, rightPWM >= 0);
  // Left wheel direction
  digitalWrite(L_IN1, leftPWM >= 0);
  digitalWrite(L_IN2, leftPWM < 0);

  analogWrite(EN_RIGHT, abs(rightPWM));
  analogWrite(EN_LEFT,  abs(leftPWM));
}

// ===== Ultrasonic ======
int readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long echoTime = pulseIn(ECHO_PIN, HIGH, SONIC_TIMEOUT_US);
  if (echoTime == 0) return 0;                  // timeout
  return static_cast<int>(echoTime / 29 / 2);   // convert to cm
}

// ===== Obstacle Handling ======
void avoidObject() {
  scanSides();
  // Pick the side with more clearance
  if (rightDist >= leftDist) {
    // Path: back – pivot right – forward flank – pivot left – merge
    rotateMotors(-TURN_PWM, -TURN_PWM); delay(400);
    rotateMotors(0, 0);

    rotateMotors(-TURN_PWM, TURN_PWM);  delay(250);
    rotateMotors(0, 0);

    rotateMotors(CRUISE_PWM, CRUISE_PWM); delay(500);
    rotateMotors(0, 0);

    rotateMotors(TURN_PWM, -TURN_PWM);  delay(350);
    rotateMotors(0, 0);
  } else {
    // Mirror to the left
    rotateMotors(-TURN_PWM, -TURN_PWM); delay(400);
    rotateMotors(0, 0);

    rotateMotors(TURN_PWM, -TURN_PWM);  delay(250);
    rotateMotors(0, 0);

    rotateMotors(CRUISE_PWM, CRUISE_PWM); delay(500);
    rotateMotors(0, 0);

    rotateMotors(-TURN_PWM, TURN_PWM);  delay(350);
    rotateMotors(0, 0);
  }
}

void removeObject() {
  // Push obstacle away then resume
  rotateMotors(-TURN_PWM, -TURN_PWM);  delay(300);
  rotateMotors(0, 0);

  rotateMotors(CRUISE_PWM + 30, CRUISE_PWM + 30); delay(600);
  rotateMotors(0, 0);
}

// ===== Side Scan (servo) ======
void scanSides() {
  // Right side scan
  sonarServo.write(SERVO_RIGHT);
  delay(250);
  rightDist = readDistanceCm();

  // Left side scan
  sonarServo.write(SERVO_LEFT);
  delay(250);
  leftDist = readDistanceCm();

  // Return forward
  sonarServo.write(SERVO_FWD);
  delay(100);

  Serial.print(F("Side scan – R:")); Serial.print(rightDist);
  Serial.print(F(" cm  L:")); Serial.print(leftDist); Serial.println(F(" cm"));
}

// ===== Colour Sensor ======
void calibrateColourSensor() {
  float sum = 0.0f;
  for (uint8_t i = 0; i < 10; ++i) {
    // RED filter S2=S3=LOW
    digitalWrite(S2, LOW);
    digitalWrite(S3, LOW);
    unsigned long period = pulseIn(TCS_OUT, HIGH) + pulseIn(TCS_OUT, LOW);
    if (period == 0) continue;
    sum += 1.0e6f / period;   // convert period to Hz
  }
  redAmbient = sum / 10.0f;
  Serial.print(F("Ambient red freq: ")); Serial.println(redAmbient);
}

bool isRedObject() {
  float sum = 0.0f;
  for (uint8_t i = 0; i < 10; ++i) {
    digitalWrite(S2, LOW);
    digitalWrite(S3, LOW);
    unsigned long period = pulseIn(TCS_OUT, HIGH) + pulseIn(TCS_OUT, LOW);
    if (period == 0) continue;
    sum += 1.0e6f / period;
  }
  float redAvg = sum / 10.0f;
  Serial.print(F("Red avg: ")); Serial.println(redAvg);
  return (redAvg > redAmbient * RED_THRESHOLD_FACTOR);
}
