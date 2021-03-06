#include <enes100.h>
#include "math.h"
#include <PID_v1.h>
#include <Servo.h>


// Pins
const int TRIG_PIN = 7;
const int ECHO_PIN = 10;
const int RIGHT_MOTOR_A = 2;
const int RIGHT_MOTOR_B = 4;
const int RIGHT_MOTOR_PWM = 5;
const int LEFT_MOTOR_A = 13;
const int LEFT_MOTOR_B = 12;
const int LEFT_MOTOR_PWM = 3;
const int PH_PIN = 0; //A0
const int SERVO_PIN = 6;
const int PUMP_NEUT_HIGH = 0;
const int PUMP_COLL_HIGH = 1;
const int PUMP_SHARED_LOW = 11;

// Anything over 400 cm (23200 us pulse) is "out of range"
const unsigned int MAX_DIST = 23200;

SoftwareSerial mySerial(8,9); 
Marker marker(10); // Will have to replace # with our team's marker #
RF_Comm rf(&mySerial, &marker);

Servo myservo;

//Define PID variables
double tTheta, motorOutput;

PID turnPID((double*)&marker.theta, &motorOutput, &tTheta, .5, 7, 0, DIRECT);

boolean isTopPath = false;
boolean isFinished = false;



void setup() {

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  pinMode(RIGHT_MOTOR_A, OUTPUT);
  pinMode(RIGHT_MOTOR_B, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_A, OUTPUT);
  pinMode(LEFT_MOTOR_B, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);

  pinMode(PUMP_NEUT_HIGH, OUTPUT);
  pinMode(PUMP_COLL_HIGH, OUTPUT);
  pinMode(PUMP_SHARED_LOW, OUTPUT);

  digitalWrite(PUMP_SHARED_LOW, LOW);
  digitalWrite(PUMP_NEUT_HIGH, LOW);
  digitalWrite(PUMP_COLL_HIGH, LOW);

  pinMode(SERVO_PIN, OUTPUT);
  myservo.attach(SERVO_PIN);
  pinMode(SERVO_PIN, INPUT);
  
  rf.transmitData(START_MISSION, NO_DATA); // Lets the Vision System (which is timing you) know that you are starting your mission
  
  turnPID.SetMode(AUTOMATIC);
  turnPID.SetSampleTime(10);
  turnPID.SetOutputLimits(-1,1);

}

void loop() {
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

    myservo.detach();
    drive(0,0);
    float ultra = getUltrasonic();
    myservo.attach(SERVO_PIN);
    
    rf.println(ultra);
    if(ultra > 30.0){
      rf.print("Move marker to 1.5, ");
      rf.println(isTopPath ? 1.7 : 0.3);
      driveToPositionX(1.5, (isTopPath ? 0.3 : 1.7));
      rf.print("Turn to angle: ");
      rf.println(atan2(1.5 - marker.y, 2.0 - marker.x));
      turnToAngle(atan2(1.5 - marker.y, 2.0 - marker.x));
      rf.print("Move marker to position 2.0, 1.5");
      driveToWithin(2.0, 1.5, .37);
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
      driveToWithin(2, 1.5, .37);
    }

    turnToAngle(atan2(1.5 - marker.y, 2.0 - marker.x));
    
    rf.println("AT CHEM SITE");
    drive(0,0);
    rf.transmitData(NAV, CHEMICAL);

    pinMode(SERVO_PIN, OUTPUT);
    myservo.write(60);
    delay(2000);
    myservo.write(90);
    pinMode(SERVO_PIN, INPUT);


    rf.println("SUBMERGED");
    
    delay(15000);
    
    rf.transmitData(BASE, getPH());

    rf.print("PH BEFORE: ");
    rf.println(getPH());

    digitalWrite(PUMP_COLL_HIGH, HIGH);
    delay(18000);
    digitalWrite(PUMP_COLL_HIGH, LOW);

    rf.println("DONE COLLECTING");

    unsigned long int phTimer = millis();
    delay(10);

    while(getPH() < 6 && millis() - phTimer < 240000){
      rf.print("NEUT PH: ");
      rf.println(getPH());
      digitalWrite(PUMP_NEUT_HIGH, HIGH);
      delay(2000);
      digitalWrite(PUMP_NEUT_HIGH, LOW);
      delay(2000);
    }

    digitalWrite(PUMP_NEUT_HIGH, LOW);

    rf.println("DONE NEUTRALIZING");

    delay(10000);

    rf.transmitData(BONUS, getPH());

    isFinished = true;

    rf.transmitData(END_MISSION, NO_DATA);
  }
}

void turnToAngle(float theta){
  tTheta = theta;
  rf.updateLocation();
  
  while(angleDiff(marker.theta, theta) > 0.04){
    turnPID.Compute();
    drive(-motorOutput, motorOutput);
    rf.updateLocation();
  }
  drive(0,0);
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
  analogWrite(LEFT_MOTOR_PWM, abs(left * 255));
  digitalWrite(LEFT_MOTOR_A, left > 0 ? LOW : HIGH);
  digitalWrite(LEFT_MOTOR_B, left > 0 ? HIGH : LOW);

  analogWrite(RIGHT_MOTOR_PWM, abs(right * 255));
  digitalWrite(RIGHT_MOTOR_A, right > 0 ? HIGH : LOW);
  digitalWrite(RIGHT_MOTOR_B, right > 0 ? LOW : HIGH);
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
    
    pulse_width = pulseIn(ECHO_PIN, HIGH, MAX_DIST);
    
    if(pulse_width == 0){
      pulse_width = MAX_DIST;
    }
  
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

float getPH(){
  unsigned long int avgValue;  //Store the average value of the sensor feedback
  float b;
  int buf[10],temp;

    for(int i=0;i<10;i++)       //Get 10 sample value from the sensor for smooth the value
  { 
    buf[i]=analogRead(PH_PIN);
    delay(10);
  }
  for(int i=0;i<9;i++)        //sort the analog from small to large
  {
    for(int j=i+1;j<10;j++)
    {
      if(buf[i]>buf[j])
      {
        temp=buf[i];
        buf[i]=buf[j];
        buf[j]=temp;
      }
    }
  }
  avgValue=0;
  for(int i=2;i<8;i++)                      //take the average value of 6 center sample
    avgValue+=buf[i];
  float phValue=(float)avgValue*5.0/1024/6; //convert the analog into millivolt
  phValue=(3.25)*phValue;                      //convert the millivolt into pH value

  return phValue;
}

void driveToWithin(float x, float y, float tolerance){
    while(sqrt(pow(2 - marker.x, 2) + pow(1.5 - marker.y, 2)) > tolerance){
      if(angleDiff(marker.theta, atan2(y - marker.y, x - marker.x)) < 0.6){
        drive(.6,.6);
      } else {
        turnToAngle(atan2(y - marker.y, x - marker.x));
      }
      rf.updateLocation();
    }
}

