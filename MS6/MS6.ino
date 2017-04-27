#include <Servo.h>
#include <enes100.h>

const int SENSOR_PIN = 0; //A0

const int SERVO_PIN = 6;

const int RIGHT_MOTOR_A = 2;
const int RIGHT_MOTOR_B = 4;
const int RIGHT_MOTOR_PWM = 5;
const int LEFT_MOTOR_A = 13;
const int LEFT_MOTOR_B = 12;
const int LEFT_MOTOR_PWM = 3;

boolean finished = false;

SoftwareSerial mySerial(8,9);
Marker marker(102);
RF_Comm rf(&mySerial, &marker);

Servo myservo;

void setup() {
  pinMode(RIGHT_MOTOR_A, OUTPUT);
  pinMode(RIGHT_MOTOR_B, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_A, OUTPUT);
  pinMode(LEFT_MOTOR_B, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);

  pinMode(SERVO_PIN, OUTPUT);
  myservo.attach(SERVO_PIN);

  rf.transmitData(NAV, CHEMICAL);
}

void loop() {
  rf.updateLocation();
  
  if(!finished){
    
    myservo.write(60);
    
    while(sqrt(pow(2 - marker.x, 2) + pow(1.5 - marker.y, 2)) > .25){
      drive(1,1);
      rf.updateLocation();
    }

    drive(0,0);

    myservo.write(150);
    delay(800);
    myservo.write(90);

    rf.transmitData(BASE, getPH());
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

float getPH(){
  unsigned long int avgValue;  //Store the average value of the sensor feedback
  float b;
  int buf[10],temp;

    for(int i=0;i<10;i++)       //Get 10 sample value from the sensor for smooth the value
  { 
    buf[i]=analogRead(SENSOR_PIN);
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
  phValue=(3.68)*phValue;                      //convert the millivolt into pH value

  return phValue;
}

