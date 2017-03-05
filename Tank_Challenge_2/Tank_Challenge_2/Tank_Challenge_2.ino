#include <dfr_tank.h>
#include "enes100.h"
#include "math.h"

// Pins
const int TRIG_PIN = 12;
const int ECHO_PIN = 13;

// Anything over 400 cm (23200 us pulse) is "out of range"
const unsigned int MAX_DIST = 23200;

// Variables for ultrasonic sensor
unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

SoftwareSerial mySerial(8,9); 
Marker marker(108); // Will have to replace # with our team's marker #
RF_Comm rf(&mySerial, &marker);


DFRTank tank;


void setup() {
  Serial.begin(9600);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  
  rf.startMission(); // Lets the Vision System (which is timing you) know that you are starting your mission

  tank.init();

}

void loop() {
  rf.updateLocation();
  
  while(angleDiff(marker.theta, (3.14159653/2.0)) > 0.25){
    tank.setLeftMotorPWM(-125);
    tank.setRightMotorPWM(125);
    rf.updateLocation();
  }
    
  tank.turnOffMotors();
}

float angleDiff(float theta, float phi){
  return (theta - phi) >= 0 ? theta - phi : -(theta - phi);
}

float getUltrasonic(){
  
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  while ( digitalRead(ECHO_PIN) == 0 );

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min
  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 1);
  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed 
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Wait at least 60ms before next measurement
  delay(60);

  return cm;
}

