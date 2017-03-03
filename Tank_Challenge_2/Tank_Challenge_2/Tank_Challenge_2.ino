#include <dfr_tank.h>
#include "enes100.h"
#include "math.h"

#define PI 3.14159264

SoftwareSerial mySerial(8,9); 
Marker marker(112); // Will have to replace # with our team's marker #
RF_Comm rf(&mySerial, &marker);


DFRTank tank;


void setup() {
  rf.startMission(); // Lets the Vision System (which is timing you) know that you are starting your mission

  tank.init();

}

void loop() {
  rf.updateLocation();

  while(abs(marker.theta - PI/2) > 0.1){
    tank.setLeftMotorPWM(-175);
    tank.setRightMotorPWM(175);
    rf.updateLocation();
  }
  
  tank.turnOffMotors();
}
