// created on 2/13/2024
//
//Eric Knewtson

#include <SoftwareSerial.h>
#include <Sabertooth.h>
#include <Arduino.h>
#include <Wire.h>
//#include <HMC5883L_Simple.h>
#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

float PIDcmd;
float Area0 = 0;
float kp = 1;
float ki = 0.7;
float kd = 0.9;
float dt = 0.1;
float error_1;


//RC Related
#define Throttle_Pin A0
#define Aileron_Pin A1
#define Elvator_Pin A2
#define Rudder_Pin A3

long cutoff = 1200;
long Throttle = 0;
long Aileron = 0;
long Elvator = 0;
long Rudder = 0;

//Motor Related
int Left_Motors_Pin = 38;
int Left_Motors_ES  = 42; // Set to 0 for Estop

int Right_Motors_Pin = 50;
int Right_Motors_ES  = 46; // Set to 0 for Estop


SoftwareSerial LM_Serial(NOT_A_PIN, Left_Motors_Pin); //Left side Motors
SoftwareSerial RM_Serial(NOT_A_PIN, Right_Motors_Pin); //Right side Motors

Sabertooth LM_ST(128, LM_Serial);
Sabertooth RM_ST(128, RM_Serial);

//Strobo Ligth Related
int Strobolight = 3;
int Lightmode = 4;

//Battery
#define battery A14

//Buzzers
#define buzzer_pin A8

// Create a compass
//HMC5883L_Simple Compass;

static const int RXPin = 18, TXPin = 19;
static const uint32_t GPSBaud = 9600;
//Gyroscope
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// The TinyGPS++ object
TinyGPSPlus gps;


float LngStart = gps.location.lng();
float LatStart = gps.location.lat();
float StartLngrad;
float StartLatrad;
float Heading;
float Bearing;
float Bearing1;
float Distance;
float Distance1;

//Some other Variables
int maxSpeed = 127;     //max 127




void forward(int fspeed)
{
 // digitalWrite(Strobolight, HIGH);  //trun on the strobolight
  fspeed = constrain(fspeed, -1 * maxSpeed, maxSpeed);    //limit the robot speed
  LM_ST.motor(1, fspeed);
  LM_ST.motor(2, fspeed);
  RM_ST.motor(1, fspeed);
  RM_ST.motor(2, fspeed);
  digitalWrite (Left_Motors_ES, HIGH);    //to enable the motors
  digitalWrite (Right_Motors_ES, HIGH);
}


void rotate(int rspeed)
{
  //digitalWrite(Strobolight, HIGH);    //trun on the strobolight
  rspeed = constrain(rspeed, -1 * maxSpeed, maxSpeed);    //limit the robot speed
  LM_ST.motor(1, rspeed);
  LM_ST.motor(2, rspeed);
  RM_ST.motor(1, -1 * rspeed);
  RM_ST.motor(2, -1 * rspeed);
  digitalWrite (Left_Motors_ES, HIGH);  //to enable the motors
  digitalWrite (Right_Motors_ES, HIGH);
}

void turn(float tspeed)
{
  //digitalWrite(Strobolight, HIGH);
  tspeed = constrain(tspeed, -1 * maxSpeed, maxSpeed);
  if(tspeed > 1)
  {
  LM_ST.motor(1, tspeed);
  LM_ST.motor(2, tspeed);
  RM_ST.motor(1, -1 * tspeed);
  RM_ST.motor(2, -1 * tspeed);
  digitalWrite (Left_Motors_ES, HIGH);  //to enable the motors
  digitalWrite (Right_Motors_ES, HIGH);
  }
  else
  {
  LM_ST.motor(1, -1 * tspeed);
  LM_ST.motor(2, -1 * tspeed);
  RM_ST.motor(1, tspeed);
  RM_ST.motor(2, tspeed);
  digitalWrite (Left_Motors_ES, HIGH);  //to enable the motors
  digitalWrite (Right_Motors_ES, HIGH);
  }
}

void differential(int leftspeed, int rightspeed)
{
  //digitalWrite(Strobolight, HIGH);
  digitalWrite(buzzer_pin, LOW);
  leftspeed = constrain(leftspeed, -1 * maxSpeed, maxSpeed);    //limit the robot speed
  rightspeed = constrain(rightspeed, -1 * maxSpeed, maxSpeed);    //limit the robot speed
  LM_ST.motor(1, leftspeed);
  LM_ST.motor(2, leftspeed);
  RM_ST.motor(1, rightspeed);
  RM_ST.motor(2, rightspeed);
  digitalWrite (Left_Motors_ES, HIGH);  //to enable the motors
  digitalWrite (Right_Motors_ES, HIGH);
}


void estop()
{
  digitalWrite(Strobolight, LOW); //to disable the lights
  digitalWrite(buzzer_pin, LOW);
  digitalWrite (Left_Motors_ES, LOW); //to enable the motors
  digitalWrite (Right_Motors_ES, LOW);
}

void batterycheck() {
  Serial.print("Battery Analog Read = ");
  Serial.println(analogRead(battery));
}

float getBearing() {//gets the bearing between the current positon and the starting position
  float Bearing = atan2(cos(gps.location.lat()) * sin((gps.location.lng()) - LngStart) , cos(LatStart) * sin(gps.location.lat()) - sin(LatStart) * cos((gps.location.lng())) * cos((gps.location.lng()) - LngStart));
  Serial.print("Bearing");
  Serial.println(Bearing);
  return Bearing;
}
float getDistance() {  //returns distance in meters
  float crtLng = gps.location.lng();
  float crtLat = gps.location.lat();
  float crtLatrad = crtLat * 180 / M_PI;
  float crtLngrad = crtLng * 180 / M_PI;
  float StartLatrad = LatStart * 180 / M_PI;
  float StartLngrad = LngStart * 180 / M_PI;
  float Distance = 6371009 * sqrt((crtLatrad - StartLatrad) * (crtLatrad - StartLatrad) + (cos((StartLatrad + crtLatrad) / 2) * (crtLngrad - StartLngrad)) * (cos((StartLatrad + crtLatrad) / 2) * (crtLngrad - StartLngrad)));
  return Distance;
}
float getHeading() {
  sensors_event_t event;
  bno.getEvent(&event);
  return event.orientation.x;
}

void turnto() {// turns the robot to face the starting position
   //float Distance1 = getDistance();
   float Bearing1 = getBearing();
   float Heading = getHeading();
  
   float error = Bearing1 - Heading;
  while(error > 5 or error  < -5)
  {
    if (error < 0)
    {
      differential(60, -60);
    }
    else
    {
      differential(-60, 60);
    }
    //Distance1 = getDistance();
    Bearing1 = getBearing();
    Heading = getHeading();
    error = Heading - Bearing1;
    Serial.print("Bearing: ");
    Serial.println(Bearing1);
    Serial.print("Heading: ");
    Serial.println(Heading);
    Serial.print("error: ");
    Serial.println(error);
  }


  // turn(error);
  // error_1 = error;
  // Bearing1 = getBearing();
  // while(error > 20) {
  //   error = Heading - Bearing1;
  //   PIDcmd = kp * error + ki * (Area0 + dt *(error + error_1) / 2) + kd * (error - error_1) / dt;
  //   Area0 = Area0 + dt * (error + error_1) / 2;
  //   turn(PIDcmd);
  //   error_1 = error;
  //   Serial.print("Heading");
  //   Serial.println(Heading);
  //   Heading = getHeading();
  //   Bearing1 = getBearing();
  //   error = Heading - Bearing1;
  //   Serial.print("PIDerror");
  //   Serial.println(error);
  // }
  // return;
}


void setup() {
  // put your setup code here, to run once:

  // RC Related
  pinMode(Throttle_Pin, INPUT);
  pinMode(Aileron_Pin, INPUT);
  pinMode(Elvator_Pin, INPUT);
  pinMode(Rudder_Pin, INPUT);

  //Motor Related
  digitalWrite (Left_Motors_ES, LOW);
  digitalWrite (Right_Motors_ES, LOW);

  pinMode (Left_Motors_ES, OUTPUT);
  pinMode (Right_Motors_ES, OUTPUT);

  LM_Serial.begin(9600);
  RM_Serial.begin(9600);

  //Strobo Light
  //pinMode(Strobolight, OUTPUT);
  //pinMode(Lightmode, OUTPUT);

  //digitalWrite(Strobolight, LOW);
  //digitalWrite(Lightmode, LOW);

  //Buzzer
  pinMode(buzzer_pin, OUTPUT);

  Serial.begin(9600);
  Wire.begin();


  Serial1.begin(GPSBaud);

  // Compass.SetDeclination(-9, 16, 'W');

  // Compass.SetSamplingMode(COMPASS_CONTINUOUS);

  // Compass.SetScale(COMPASS_SCALE_130);

  // Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);
    if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);
  
  // differential(-30, 30);
}

void loop() {
   //put your main code here, to run repeatedly:
   estop();    //stop the robot if the remote is not active 
   //digitalWrite(Strobolight, LOW);
   Throttle = pulseIn(Throttle_Pin, HIGH, 1000); // Maximum timeout of 1000us = 1 ms so that we don't stall the rest of the program

// Heading = getHeading();
// Serial.print("Heading: ");
// Serial.println(Heading);
  while (Throttle > cutoff) {

    
    /*
      digitalWrite(Lightmode, HIGH);
      delay(1000);
      digitalWrite(Lightmode, LOW);
      digitalWrite(Strobolight, HIGH);
      delay(10000);
    */


    Throttle = pulseIn(Throttle_Pin, HIGH);
    Aileron = pulseIn(Aileron_Pin, HIGH);
    Elvator = pulseIn(Elvator_Pin, HIGH);
    Rudder = pulseIn(Rudder_Pin, HIGH);

    int forward = (Elvator - 1500) / 3;
    int side = (Aileron - 1500) / 3;

    int rightspeed = forward + side;
    int leftspeed = forward - side;

    Serial.println("RSpeed = " + String(rightspeed) + "/" + "LSpeed = " + String(leftspeed));

    differential(leftspeed, rightspeed);
}
// */
  // float heading = getHeading();
  // Serial.print("Heading: ");
  // Serial.println(String(heading));

//   if(heading > 357){
//     differential(20, -20);
//   }
//   else if(heading > 3){
//     differential(-20, 20);
//   }
//   else{
//     // Stop
//     differential(0, 0);
//   }

 // if (Serial.available())
  //{
    //char ch = Serial.read();
   // Serial.println(ch);
    //if(ch == 'R')
   // {
     
    //  int x = 0;
    //  while(x < 50){
    //    forward(22500);
    //    x++;
    //   }
    //  turnto();
    //  delay(25000);
    //}
  //}
    // float dist = getdist;
    // while(dist > 500)
    // {
    //   forward(200);
    //   dist = getdist();
    // }
  //while(){
    if(Serial.available()){
      char ch = Serial.read();
      int cond = Serial.parseInt();

      // Clear the serial buffer after reading the desired data
      while (Serial.available()) {
        Serial.read();  // Read and discard one byte of data from the buffer
      }
      Serial.print(ch);
      Serial.println(cond);
      if(ch =='D'){
        if(cond == 1/*condition 1*/){
          differential(-64,64);
        }
        else if(cond == 2/*condition 2*/){
          differential(64,127);
        }
        else if(cond == 3  /*condition 3*/){
          differential(127,127);
        }
        else if(cond == 4/*condition 4*/){
          differential(127,64);
        }
        else{/*condition 5*/
        differential(64,-64);
        }
      }
    }
  //}
}
