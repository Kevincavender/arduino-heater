#include "max6675.h"
#include <PID_v1.h>

int RELAY_PIN=11;
//DEFINE PID VARIABLES
//Define Variables we'll be connecting to

double Setpoint, Input, Output;
//Specify the links and initial tuning parameters

double Kp=14.684, Ki=0.03194, Kd=0;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int WindowSize = 10000;//Sets the PWM relay period (1000 = 1 sec)
unsigned long windowStartTime;
bool relay = LOW;

//DEFINE TEMPERATURE SENSOR VARIABLES
float sensorValue;
float N=5; //number of averaged temperature samples
float avgValue;
int thermoSO = 9;
int thermoCS = 5;
int thermoSCK = 6;
MAX6675 thermocouple(thermoSCK, thermoCS, thermoSO);
int vccPin = 3;
int gndPin = 2;

//amplification of the temperature reading and setpoint
//this is a random constant that made things work correctly
float amp=77;
// the setup function runs once when you press reset or power the board

void setup() 
{
  // initialize digital pin 12 as an output.
  pinMode(RELAY_PIN, OUTPUT);//this is the relay pin
  Serial.begin(9600);

  pinMode(vccPin, OUTPUT); digitalWrite(vccPin, HIGH);
  pinMode(gndPin, OUTPUT); digitalWrite(gndPin, LOW);

  delay(500);//give the a/d converter chip time so startup
  windowStartTime = millis();
  //initialize the variables we're linked to
  
  float Setsetpoint = 70;     //TEMPERATURE SETPOINT (degrees C)

  WindowSize = 10000;
  Setpoint=Setsetpoint*amp;
  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

// the loop function runs over and over again forever
void loop() 
{
delay(1000);

float T = thermocouple.readCelsius();
Input = T*amp;
myPID.Compute();

/************************************************
* turn the output pin on/off based on pid output
************************************************/

if (millis()-windowStartTime > WindowSize)
  { //time to shift the Relay Window
     windowStartTime += WindowSize;
  }
if (Output < millis()-windowStartTime)
  {
    
    relay = LOW;
  }

else relay = HIGH;

digitalWrite(RELAY_PIN, relay);

Serial.print(T);
Serial.print(", ");
Serial.print(Output);
Serial.print(", ");
Serial.print(relay);
Serial.print(", ");
Serial.println(millis());

}
