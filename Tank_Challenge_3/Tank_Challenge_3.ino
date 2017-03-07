#include <dfr_tank.h>
#include "enes100.h"
#include "math.h"
#include <PID_v1.h>

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

//Define PID variables
double tTheta, motorOutput;

PID turnPID(&marker.theta, &motorOutput, &tTheta, 200, 20, 0, DIRECT);

boolean isTopPath = false;
boolean isFinished = false;

DFRTank tank;


void setup() {
  Serial.begin(9600);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  
  rf.startMission(); // Lets the Vision System (which is timing you) know that you are starting your mission

  tank.init();

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
  myPID.SetOutputLimits(-255,255);
}

void loop() {
  if(!isFinished){
    rf.updateLocation();
  
    if(marker.y > 1){ //if top half turn down
        isTopPath = true;
    } else { //if bottom half, turn up
        isTopPath = false;
    }
  
    turnToAngle(isTopPath ? -PI/2 : PI/2);

    driveToPositionY(isTopPath ? 0.2 : 1.8);

    turnToAngle(0);

    if(getUltrasonic() > 100){
      driveToPositionX(1.5);
    } else {
      turnToAngle(isTopPath ? PI/2 : -PI/2);
      driveToPositionY(isTopPath ? 1.8 : 0.2);
      turnToAngle(0);
      driveToPositionX(1.5);
    }

    isFinished = true;
    
    tank.turnOffMotors();
  }
}

void turnToAngle(float theta){
  tTheta = theta;
  rf.updateLocation();
  
  while(angleDiff(marker.theta, theta) > 0.25){
    myPID.Compute();
    tank.setLeftMotorPWM(-motorOutput);
    tank.setRightMotorPWM(motorOutput);
    rf.updateLocation();
  }
}

void driveToPositionY(float target){
  rf.updateLocation();
  
  if(target > marker.y){
    while(target > marker.y){
      drive(.6, .6);
      rf.updateLocation();
    }
  } else {
    while(marker.y > target){
      drive(.6, .6);
      rf.updateLocation();
    }
  }
}

void driveToPositionX(float target){
  rf.updateLocation();
  
  if(target > marker.x){
    while(target > marker.x){
      drive(.6, .6);
      rf.updateLocation();
    }
  } else {
    while(marker.x > target){
      drive(.6, .6);
      rf.updateLocation();
    }
  }
}

void drive(float left, float right){
  tank.setLeftMotorPWM(left * 255);
  tank.setRightMotorPWM(right * 255);
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

