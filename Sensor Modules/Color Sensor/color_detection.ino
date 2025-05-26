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
int redMin = 0;
int redMax = 1;
int greenMin = 0;
int greenMax = 1;
int redColor = 0;
int greenColor = 0;
int redFrequency = 0;
int redEdgeTime = 0;
int greenFrequency = 0;
int greenEdgeTime = 0;
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
{
/*Determination of the photodiode type during measurement
S2/S3
LOW/LOW=RED, LOW/HIGH=BLUE,
HIGH/HIGH=GREEN, HIGH/LOW=CLEAR*/
digitalWrite(S2, LOW);
digitalWrite(S3, LOW);
/*Frequency measurement of the specified color and its as-
signment to an RGB value between 0-255*/
float(redEdgeTime) = pulseIn(sensorOut, HIGH) + pulseIn
(sensorOut, LOW);
float(redFrequency) = (1 / (redEdgeTime / 1000000));
redColor = map(redFrequency, redMax, redMin, 255, 0);
if (redColor > 255) {
redColor = 255;
}
if (redColor < 0) {
redColor = 0;
}
/*Output of frequency mapped to 0-255*/
Serial.print("Red Frequency: ");
Serial.println(redFrequency);
Serial.print("R = ");
Serial.println(redColor);
delay(100);
}
{
/*Output of frequency mapped to 0-255*/
Serial.print("Green Frequency: ");
Serial.println(greenFrequency);
Serial.print("G = ");
Serial.println(greenColor);
delay(100);
}
Serial.println("------------------------------");
delay(100);
/*Determination of the measured color by comparison
with the values of the other measured colors*/
if (redColor > 700) {
Serial.println("Red detected ");
delay(100);
}
if (700 > redColor) {
Serial.println("Green detected ");
delay(100);
}
if (greenColor > 0 and redColor > 0) {
Serial.println("Please place an Object in front of the Sensor");
delay(100);
}
{
Serial.println("------------------------------");
delay(1000);
}
}
