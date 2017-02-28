#include "enes100.h"
#include "math.h"

/*
 * Replace 8 and 9 with the pins you plan on using for RX and TX 
 * respectively (recall that your RX pin connects to the APC’s TX 
 * pin and vice versa).
 */
SoftwareSerial mySerial(8,9); 
Marker marker(3); // Will have to replace # with our team's marker #
RF_Comm rf(&mySerial, &marker);

//Tracks state of FSM
int state = 1; 
//Pin for distance
const int dSense=A0;

//Drives Robot to center of staging area
float tLocX = .5;
float tLocY = 1;
float tTheta = 0;

void setup() {
  rf.startMission(); // Lets the Vision System (which is timing you) know that you are starting your mission
  pinMode(dSense,OUTPUT);
  Serial.begin(9600);

  rf.updateLocation();

  //Finds vector from starting position to center of staging area
  tTheta = atan2(tLocX - marker.x, tLocY - marker.y);
}

void loop() {
    rf.updateLocation(); // Boolean. Requests an update of your OSV's location & stores it in marker object.
    
    //Naive implementation of a FSM
    switch (state) {
      //State 1 turns robot to correct angle
      case 1:
        float err = marker.theta - tTheta;
        if(abs(err) > 0.02){
          turn(1); //turn right until you're facing the right direction/
        } else {
          state++:
        }
        break;
    }
    
}

void turn(int direction){
  //TODO: Make this actually turn the robot...
}

