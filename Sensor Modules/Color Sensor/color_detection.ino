#include <math.h>
#define noPaverageseInterrupts 1
// Assignment of the sensor pins
#define S0 A2
#define S1 A3
#define S2 A4
#define S3 A5
#define sensorOut 10
/*Calibration values (must be updated before
updated before each use)*/
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

void setup()
{
/*definition of the sensor pins*/
pinMode(S0, OUTPUT);
pinMode(S1, OUTPUT);
pinMode(S2, OUTPUT);
pinMode(S3, OUTPUT);
pinMode(sensorOut, INPUT);
/*Scaling the output frequency
S0/S1
LOW/LOW=AUS, LOW/HIGH=2%,
HIGH/LOW=20%, HIGH/HIGH=100%*/
digitalWrite(S0, LOW);
digitalWrite(S1, HIGH);
Serial.begin(9600);
}
//Initialization of the loop
void loop() {
Serial.println("------------------------------");
  if(ambientChecked == false){
    for(t=0,redFrequency=0;t<=10;t++){
      digitalWrite(S2, LOW);
      digitalWrite(S3, LOW);
      /*Frequency measurement of the red color*/
      float(redEdgeTime) = pulseIn(sensorOut, HIGH) + pulseIn(sensorOut, LOW);
      float(redFrequency) = (1 / (redEdgeTime / 1000000));
      redSum += redFrequency;
    }
    redAmbient = redSum/10;
    ambientChecked = true;
  }

for(t=0,redFrequency=0,redAvg=0;t<=10;t++){
/*Determination of the photodiode type during measurement
S2/S3
LOW/LOW=RED, LOW/HIGH=BLUE,
HIGH/HIGH=GREEN, HIGH/LOW=CLEAR*/
digitalWrite(S2, LOW);
digitalWrite(S3, LOW);
/*Frequency measurement of the red color*/
float(redEdgeTime) = pulseIn(sensorOut, HIGH) + pulseIn(sensorOut, LOW);
float(redFrequency) = (1 / (redEdgeTime / 1000000));
redSum += redFrequency;
}
redAvg = redSum/10;
  if(redAvg > redAmbient){
    red = true;
  }
  else{
    red = false;
  }
Serial.print("Average red detected: ");
Serial.println(redAvg);
Serial.println("------------------------------");
delay(100);
/*Determination of the measured color by comparison with the ambient average*/
if (red == true) {
Serial.println("Red detected ");
delay(100);
}
if (red == false) {
Serial.println("Green detected ");
delay(100);
}
{
Serial.println("------------------------------");
delay(1000);
}
}
