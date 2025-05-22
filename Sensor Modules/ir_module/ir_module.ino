//IR Sensor Pin Definitions
#define IR_LEFT_PIN 7    // Left IR sensor OUT
#define IR_RIGHT_PIN 8   // Right IR sensor OUT

//IR Sensor Initialization
void initIR() {
  pinMode(IR_LEFT_PIN, INPUT);
  pinMode(IR_RIGHT_PIN, INPUT);
}

//Check if line is detected under either sensor
bool isLineDetected() {
  int left = digitalRead(IR_LEFT_PIN);
  int right = digitalRead(IR_RIGHT_PIN);

  //Black line: LOW reading when detected
  return (left == LOW || right == LOW);
}

//Check if both sensors are off the line
bool isEndOfLine() {
  int left = digitalRead(IR_LEFT_PIN);
  int right = digitalRead(IR_RIGHT_PIN);

  // Both HIGH: white surface; no line detected
  return (left == HIGH && right == HIGH);
}
