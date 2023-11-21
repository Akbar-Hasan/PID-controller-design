



#include "LedControl.h"  // instal this library

int DIN = 7;
int CS = 6;
int CLK = 5;

LedControl matrix = LedControl(DIN, CLK, CS, 1);
// U
byte one[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff };
byte two[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff };
byte three[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff };
byte four[8] = { 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff };
byte five[8] = { 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff };
byte six[8] = { 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
byte seven[8] = { 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
byte eight[8] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
byte null[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };







#include <NewPing.h>

#define TRIGGER_PIN 9
#define ECHO_PIN 10
#define MAX_DISTANCE 400  // Maximum distance we want to measure (in centimeters).


NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
#include <PID_v1.h>

#define PIN_INPUT 0
#define PIN_OUTPUT 3
#include <Servo.h>
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
Servo myservo;
//Specify the links and initial tuning parameters
double Kp = 2, Ki = 2, Kd = 0;  //default 2,5,0
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  //initialize the variables we're linked to
  Input = analogRead(PIN_INPUT);
  Setpoint = 50;
  Serial.begin(9600);
  myservo.attach(3);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  matrix.shutdown(0, LOW);
  matrix.setIntensity(0, 8);  // Set brightness maximum is 15
  matrix.clearDisplay(0);
}

void loop() {


  int distance = sonar.ping_cm();  // Send ping, get distance in cm and print result (0 = outside set distance range)
                                   //    Serial.print(distance);
                                   // Serial.print("    ");
  distance = map(distance, 2, 19, 100, 0);




  Input = distance;

  myPID.Compute();
  analogWrite(PIN_OUTPUT, Output);
  Output = map(Output, 0, 255, 0, 100);


  Serial.print("SP: ");
  Serial.print(Setpoint);
  Serial.print("    ");

  Serial.print("PV: ");
  Serial.print(distance);
  Serial.print("%");
  Serial.print("    ");
  Serial.print("LCV: ");
  Serial.println(Output);

  myservo.write(Output);
  delay(250);

  int level = distance;
  for (int i=0;i<8;i++)
  matrix.setRow(0,i,null[i]);
  //delay(1000);


  if (level >= 2 && level <= 12) {
    for (int i = 0; i < 8; i++)
      matrix.setRow(0, i, one[i]);
  }

  else if (level >= 12 && level <= 25)
  {
     for (int i = 0; i < 8; i++)
      matrix.setRow(0, i, two[i]);
  }
  
  else if (level >= 25 && level <= 37)
  {
     for (int i = 0; i < 8; i++)
      matrix.setRow(0, i, three[i]);
  }

  else if (level >= 37 && level <= 50)
   {
      for (int i = 0; i < 8; i++)
      matrix.setRow(0, i, four[i]);
   }
  else if (level >= 50 && level <= 62)
    {
       for (int i = 0; i < 8; i++)
      matrix.setRow(0, i, five[i]);
    }
  else if (level >= 62 && level <= 75)
   {
      for (int i = 0; i < 8; i++)
      matrix.setRow(0, i, six[i]);
   }
  else if (level >= 75 && level <= 87)
    {
       for (int i = 0; i < 8; i++)
      matrix.setRow(0, i, seven[i]);
    }
  else if (level >= 87 && level <= 100)
    {
       for (int i = 0; i < 8; i++)
      matrix.setRow(0, i, eight[i]);
    }
  else
  
  {
      for (int i=0;i<8;i++)
  matrix.setRow(0,i,null[i]);

  }


  // // Display Nothing
  // for (int i=0;i<8;i++)
  // matrix.setRow(0,i,null[i]);
  // delay(1000);





}
