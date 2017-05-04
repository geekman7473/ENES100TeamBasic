#include "enes100.h"

const int PH_PIN = 1; //A0
const int PUMP_NEUT_HIGH = 8;
const int PUMP_COLL_HIGH = 9;
const int PUMP_SHARED_LOW = 11;

//SoftwareSerial mySerial(8,9);
//Marker marker(102);
//RF_Comm rf(&mySerial, &marker);


boolean isFinished = false;

void setup() {
  Serial.begin(9600);
  
  pinMode(PUMP_NEUT_HIGH, OUTPUT);
  pinMode(PUMP_COLL_HIGH, OUTPUT);
  pinMode(PUMP_SHARED_LOW, OUTPUT);

  digitalWrite(PUMP_SHARED_LOW, LOW);
  digitalWrite(PUMP_NEUT_HIGH, LOW);
  digitalWrite(PUMP_COLL_HIGH, LOW);

  //rf.transmitData(NAV, CHEMICAL);
}

void loop() {
  Serial.println(getPH());
  if(false){
  //if(!isFinished){
    //rf.transmitData(BASE, getPH());
    Serial.print("BEFORE ");
    Serial.println(getPH());
    delay(5000);
    digitalWrite(PUMP_COLL_HIGH, HIGH);
    delay(8000);
    digitalWrite(PUMP_COLL_HIGH, LOW);

    while(getPH() < 6.0){
      Serial.print("DURING ");
      Serial.println(getPH());
      digitalWrite(PUMP_NEUT_HIGH, HIGH);
      delay(2000);
      digitalWrite(PUMP_NEUT_HIGH, LOW);
      delay(10000);
    }

    Serial.print("AFTER ");
    Serial.println(getPH());
    
    isFinished = true;
  }
  
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
  phValue=(1.9)*phValue;                      //convert the millivolt into pH value

  return phValue;
}

