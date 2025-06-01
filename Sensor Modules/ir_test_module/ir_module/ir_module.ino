//IR Sensor Pin Definitions
#define IR_LEFT_PIN A0    // Left IR sensor OUT
#define IR_RIGHT_PIN A1   // Right IR sensor OUT

//IR Sensor Initialization
void setup() {
  pinMode(IR_LEFT_PIN, INPUT);
  pinMode(IR_RIGHT_PIN, INPUT);
  Serial.begin(9600);
  delay(500);
}

void loop() {
  int left = digitalRead(IR_LEFT_PIN);
  int right = digitalRead(IR_RIGHT_PIN);
  
  if (left == 1 && right == 1){
    Serial.println("Bothe sensor under line");
  }
  else if (left == 1 && right == 0){
    Serial.println("Left sensor under line");
  }
  else if (left == 0 && right == 1){
    Serial.println("Right sensor under line");
  }
  else if (left == 0 && right == 0) {
    Serial.println("Line is not detected");
  } 
}
