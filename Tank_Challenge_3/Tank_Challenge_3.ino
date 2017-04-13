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
Marker marker(7); // Will have to replace # with our team's marker #
RF_Comm rf(&mySerial, &marker);

//Define PID variables
double tTheta, motorOutput;

PID turnPID((double*)&marker.theta, &motorOutput, &tTheta, 450, 3000, 0, DIRECT);

boolean isTopPath = false;
boolean isFinished = false;

DFRTank tank;


void setup() {

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  
  //rf.transmitData(START_MISSION, NO_DATA); // Lets the Vision System (which is timing you) know that you are starting your mission
  rf.println("BASIC TEST");
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

    rf.print("Turn marker to angle: ");
    rf.println(isTopPath ? -PI/2 : PI/2);
  
    turnToAngle(isTopPath ? -PI/2 : PI/2);

    rf.println(marker.x);
    rf.println(marker.y);

    rf.print("Move marker to position 0.5, ");
    rf.println(isTopPath ? 0.3 : 1.7);
    
    driveToPositionY(.5, isTopPath ? 0.3 : 1.7);

    rf.println(marker.x);
    rf.println(marker.y);

    rf.println("Turn marker to 0 position");
    
    turnToAngle(0);

    rf.println(marker.x);
    rf.println(marker.y);

    //Pool is located at (2.0, 1.5)
    rf.print("Distance to wall: ");

    float ultra = getUltrasonic();
    
    rf.println(ultra);
    if(ultra > 30.0){
      rf.print("Move marker to 1.5, ");
      rf.println(isTopPath ? 1.7 : 0.3);
      driveToPositionX(1.5, (isTopPath ? 0.3 : 1.7));
      rf.print("Turn to angle: ");
      rf.println(atan2(1.5 - marker.y, 2.0 - marker.x));
      turnToAngle(atan2(1.5 - marker.y, 2.0 - marker.x));
      rf.print("Move marker to position 2.0, 1.5");
      driveToPositionX(2.0, 1.5);
    } else {
      rf.print("Turn to angle: ");
      rf.println(isTopPath ? PI/2 : -PI/2);
      turnToAngle(isTopPath ? PI/2 : -PI/2);
      rf.print("Move marker to: 0.5, ");
      rf.println(isTopPath ? 1.7 : 0.3);
      driveToPositionY(.5, isTopPath ? 1.7 : 0.3);
      rf.println("Turn to zero position");
      turnToAngle(0);
      rf.print("Move to position: 1.5, ");
      rf.println(isTopPath ? 1.7 : 0.3);
      driveToPositionX(1.5, isTopPath ? 1.7 : 0.3);
      rf.print("Turn to angle: ");
      rf.println(atan2(1.5 - marker.y, 2.0 - marker.x));
      turnToAngle(atan2(1.5 - marker.y, 2.0 - marker.x));
      rf.println("Move to location 2, 1.5");
      driveToPositionY(2, 1.5);
    }

    isFinished = true;
    
    tank.turnOffMotors();
  }
}

void turnToAngle(float theta){
  tTheta = theta;
  rf.updateLocation();
  
  while(angleDiff(marker.theta, theta) > 0.08){
    turnPID.Compute();
    tank.setLeftMotorPWM(-motorOutput);
    tank.setRightMotorPWM(motorOutput);
    rf.updateLocation();
  }
  tank.turnOffMotors();
}

void driveToPositionY(float targetX, float targetY){
  rf.updateLocation();
      if(targetY > marker.y){
        while(targetY > marker.y){
          if(angleDiff(marker.theta, atan2(targetY - marker.y, targetX - marker.x)) < 0.4){
            drive(.5, .5);
          } else {
            turnToAngle(atan2(targetY - marker.y, targetX - marker.x));
          }
          rf.updateLocation();
        }
      } else {
        while(marker.y > targetY){
          if(angleDiff(marker.theta, atan2(targetY - marker.y, targetX - marker.x)) < 0.4){
            drive(.5, .5);
          } else {
            turnToAngle(atan2(targetY - marker.y, targetX - marker.x));
          }
          rf.updateLocation();
        }
      }
}

void driveToPositionX(float targetX, float targetY){
  rf.updateLocation();
  
  if(angleDiff(marker.theta, atan2(targetY - marker.y, targetX - marker.x)) < 0.4){
    if(targetX > marker.x){
      while(targetX > marker.x){
          if(angleDiff(marker.theta, atan2(targetY - marker.y, targetX - marker.x)) < 0.4){
            drive(.5, .5);
          } else {
            turnToAngle(atan2(targetY - marker.y, targetX - marker.x));
          }        
          rf.updateLocation();
      }
    } else {
      while(marker.x > targetX){
          if(angleDiff(marker.theta, atan2(targetY - marker.y, targetX - marker.x)) < 0.4){
            drive(.5, .5);
          } else {
            turnToAngle(atan2(targetY - marker.y, targetX - marker.x));
          }
          rf.updateLocation();
      }
    }
  } else {
    turnToAngle(atan2(targetY - marker.y, targetX - marker.x));
  }
}

void drive(float left, float right){
  tank.setLeftMotorPWM(1 * (left * 255));
  tank.setRightMotorPWM(1 * (right * 255));
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

