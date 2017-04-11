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
Marker marker(118); // Will have to replace # with our team's marker #
RF_Comm rf(&mySerial, &marker);

//Define PID variables
double tTheta, motorOutput;

PID turnPID((double*)&marker.theta, &motorOutput, &tTheta, 350, 600, 0, DIRECT);

boolean isTopPath = false;
boolean isFinished = false;

DFRTank tank;


void setup() {

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  
  rf.transmitData(START_MISSION, NO_DATA); // Lets the Vision System (which is timing you) know that you are starting your mission
  rf.println("START");
  rf.transmitData(NAV, CHEMICAL);
  tank.init();

  turnPID.SetMode(AUTOMATIC);
  turnPID.SetSampleTime(10);
  turnPID.SetOutputLimits(-255,255);
}

void loop() {
  rf.println("LOOP");
  if(!isFinished){
    rf.updateLocation();

    rf.println(marker.x);
    rf.println(marker.y);
    
    if(marker.y > 1){ //if top half turn down
        isTopPath = true;
    } else { //if bottom half, turn up
        isTopPath = false;
    }

    rf.println(marker.x);
    rf.println(marker.y);
  
    turnToAngle(isTopPath ? -PI/2 : PI/2);

    rf.println(marker.x);
    rf.println(marker.y);

    driveToPositionY(.5, isTopPath ? 0.3 : 1.7);

    rf.println(marker.x);
    rf.println(marker.y);

    turnToAngle(0);

    rf.println(marker.x);
    rf.println(marker.y);

    //Pool is located at (2.0, 1.5)
    rf.print("Distance to wall: ");
    rf.println(getUltrasonic());
    if(getUltrasonic() > 30){
      driveToPositionX(1.5, (isTopPath ? 1.7 : 0.3));
      turnToAngle(atan2(1.5 - marker.y, 2.0 - marker.x));
      driveToPositionX(2.0, 1.5);
    } else {
      turnToAngle(isTopPath ? PI/2 : -PI/2);
      driveToPositionY(.5, isTopPath ? 1.7 : 0.3);
      turnToAngle(0);
      driveToPositionX(1.5, isTopPath ? 1.7 : 0.3);
      turnToAngle(atan2(1.5 - marker.y, 2.0 - marker.x));
      driveToPositionY(2, 1.5);
    }

    isFinished = true;
    
    tank.turnOffMotors();
  }
}

void turnToAngle(float theta){
  tTheta = theta;
  rf.updateLocation();
  
  while(angleDiff(marker.theta, theta) > 0.15){
    turnPID.Compute();
    tank.setLeftMotorPWM(-motorOutput);
    tank.setRightMotorPWM(motorOutput);
    rf.updateLocation();
  }
}

void driveToPositionY(float targetX, float targetY){
  rf.updateLocation();
    if(targetY > marker.y){
      while(targetY > marker.y){
        drive(.5, .5);
        rf.updateLocation();
      }
    } else {
      while(marker.y > targetY){
        drive(.5, .5);
        rf.updateLocation();
      }
    }
}

void driveToPositionX(float targetX, float targetY){
  rf.updateLocation();
  
  if(angleDiff(atan2(targetY - marker.y, targetX - marker.x), marker.theta) < 0.15){
    if(targetX > marker.x){
      while(targetX > marker.x){
        drive(.5, .5);
        rf.updateLocation();
      }
    } else {
      while(marker.x > targetX){
        drive(.5, .5);
        rf.updateLocation();
      }
    }
  } else {
    turnToAngle(atan2(targetY - marker.y, targetX - marker.x));
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
  unsigned long timer1 = millis();
  float sum = 0;
  int count = 0;

  while(millis() - timer1 < 1500){
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
  
    sum += cm;
    count++;
  }
  return sum / count;
}

