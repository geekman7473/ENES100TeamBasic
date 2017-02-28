#include "enes100.h"

/*
 * Replace 8 and 9 with the pins you plan on using for RX and TX 
 * respectively (recall that your RX pin connects to the APC’s TX 
 * pin and vice versa).
 */
SoftwareSerial mySerial(8,9); 
Marker marker(3); // Will have to replace # with our team's marker #
RF_Comm rf(&mySerial, &marker);

int i; 
int val;
const int redpin=A0;

void setup() {
  rf.startMission(); // Lets the Vision System (which is timing you) know that you are starting your mission
  pinMode(redpin,OUTPUT);
  Serial.begin(9600);
}

void loop() {
    distance = analogRead(redpin);  // Read the sensor
    Serial.println(distance, DEC);   // Display value in Serial Monitor window
    delay(250); 

    // Not sure which one will work with the sensor.
   
    i=analogRead(redpin);
    Serial.println(i);
    val=(6762/(i-9))-4; // gives the distance in cm?? not sure.
    Serial.println(val);

  /* Stores the info sent in the marker object; acces by using marker.x, marker.y, and marker.theta
   *  x and y are measured in meters from the x and y axises to the CENTER of the marker. Theta is 
   *  measure in radians from -pi to pi (zero is parallel to X axis).
   */
  // rf.updateLocation(); // Boolean. Requests an update of your OSV's location & stores it in marker object.
  

  // rf.splitMission(); // Lets the Vision System know that you have finished navigation BEFORE starting Chemical objectives.
  // rf.stopMission(); //Lets the Vision System know to stop the timer when OSV has completed ALL of its objectives.
}
