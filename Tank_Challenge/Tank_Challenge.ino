#include <dfr_tank.h>
#include "enes100.h"
#include "math.h"

/**
 * HC-SR04 Ultrasonic Sensor
 * 
 * Description:
 *  Connect the ultrasonic sensor to the Arduino as per the
 *  hardware connections below.
 * 
 * Hardware Connections:
 *  Arduino | HC-SR04 
 *  -------------------
 *    5V    |   VCC     
 *    7     |   Trig     
 *    8     |   Echo     
 *    GND   |   GND
 */

// Pins
const int TRIG_PIN = 7;
const int ECHO_PIN = 6  ;

// Anything over 400 cm (23200 us pulse) is "out of range"
const unsigned int MAX_DIST = 23200;


/*
 * Replace 8 and 9 with the pins you plan on using for RX and TX 
 * respectively (recall that your RX pin connects to the APC’s TX 
 * pin and vice versa).
 */
SoftwareSerial mySerial(8,9); 
Marker marker(112); // Will have to replace # with our team's marker #
RF_Comm rf(&mySerial, &marker);

DFRTank tank;

//Tracks state of FSM
int state = 1; 
//Pin for distance
const int dSense=A6;

//Drives Robot to center of staging area
float tLocX = .5;
float tLocY = 1;
float tTheta = 0;

// Variables for ultrasonic sensor
unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

void setup() {
  rf.startMission(); // Lets the Vision System (which is timing you) know that you are starting your mission
  pinMode(dSense,INPUT);
  Serial.begin(9600);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  tank.init();

  rf.updateLocation();

  //Finds vector from starting position to center of staging area
  tTheta = atan2(tLocY - marker.y, tLocX - marker.x);
}

void loop() {
    rf.updateLocation(); // Boolean. Requests an update of your OSV's location & stores it in marker object.
    
    //Naive implementation of a FSM
    switch (state) {
      //State 1 turns robot to correct angle
      case 1:
        mySerial.println("Case 1");
        mySerial.print("tTheta:");
        mySerial.println(tTheta);
        turnToTarget();
        break;
       Drive towards the center point
      case 2: 
        {
          mySerial.println("Case 2");
          mySerial.print("Marker pos: ");
          mySerial.print(marker.x);
          mySerial.println(marker.y);
          // Check if the error of the angle is greater than 2 degrees
          float err = marker.theta - atan2(tLocY - marker.y, tLocX - marker.x);
          mySerial.println(err);
          if (abs(err) > 0.01) {
            state = 1; // Go back and fix the angle
          } else { // Calculate & drive the distance
            float distance = findDistance();
            drive(.6,.6);
            if (distance < 0.03) {
              state++;
            }
          }
        }
        break;
      //Turn perpendicular to wall
      case 3:
        {
          mySerial.println("Case 3");
          tTheta = 0;
          turnToTarget();
        }
        break;
      //Turn up to look for board
      case 4:
        {
          mySerial.println("Case 4");
          tTheta = PI/4;
          turnToTarget();
        }
        break;
      // Check for board, using distance senor
      case 5:
        mySerial.println("Case 5");
        if (!wallThere()) {
          drive (0.5, 0.5);
          state += 2;
        } else {
          state++;
        }
        break;
      //Turn to other side and drive that way
      case 6:
        tTheta = -PI/4;
        turnToTarget();
      // Turn straight to continue on the path beyond the wall
      case 7:
        mySerial.println("Case 7");
        drive(0,0);
        if (marker.x > 1.0) {
          tTheta = 0;
          turnToTarget();
        }
        drive(.6,.6);
        delay(100);
        drive(0,0);
        break;
    }
}



//Helper Methods

void drive(float left, float right){
  tank.setLeftMotorPWM(left * 255.0);
  tank.setRightMotorPWM(right * 255.0);
}

void turnToTarget(){
  float err = marker.theta - tTheta;
  mySerial.print("marker");
  mySerial.println(marker.theta);
  mySerial.print("tTheta");
  mySerial.println(tTheta);
  if(abs(err) > .1){
    drive(-0.7, 0.7); //Turn left until error is negligible
  } else {
    state++;
  }
}

float findDistance() {
  float curX = marker.x;
  float curY = marker.y;
  float desiredX = 0.5;
  float desiredY = 1.0;

  float diffX = abs(marker.x - desiredX);
  float diffY = abs(marker.y - desiredY);

  float distance = sqrt((diffX)*(diffX) + (diffY)*(diffY)); // Uses pythagorean thm to calculate the distance needed to travel
  return distance;
}

float findDistance(int x, int y) {
  float curX = marker.x;
  float curY = marker.y;

  float diffX = abs(marker.x - x);
  float diffY = abs(marker.y - y);

  float distance = sqrt((diffX)*(diffX) + (diffY)*(diffY)); // Uses pythagorean thm to calculate the distance needed to travel
  return distance;
}

//checks if wall is there based on known distance
boolean wallThere() {
  if (useUltrasonic() < 112) {
    return true;
  }
  return false;
}

float useUltrasonic() {
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

  // Calculate distance in centimeters. The constants
  // are found in the datasheet, and calculated from the assumed speed 
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  return cm;

  // Wait at least 60ms before next measurement
  // delay(60);
}

