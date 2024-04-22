#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SoftwareSerial.h>
#include <Sabertooth.h>


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

//Some other Variables
int maxSpeed = 127;     //max 127

//Ultrasonic sensor
const int pingPin = 7;


//gyroscope
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

//forward P
int kp = 50;
bool isStopped;

void differential(int leftspeed, int rightspeed)//moves the left and right motors at differnt speeds
{
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
  //digitalWrite(Strobolight, LOW); //to disable the lights
  digitalWrite (Left_Motors_ES, LOW); //to enable the motors
  digitalWrite (Right_Motors_ES, LOW);
}
void gyroturn(int turn_angle){// turns the robot to the 
  sensors_event_t event;
  bno.getEvent(&event);
  int start_angle = event.orientation.x;

  int final_angle = (turn_angle + start_angle + 360) % 360; // 
  
  Serial.print("Change in Heading: ");
  Serial.print(turn_angle);  
  Serial.print("\tFinal Heading: ");
  Serial.println(final_angle);
  int current_angle = start_angle;
  if(current_angle < final_angle){
    while(current_angle < final_angle){
      sensors_event_t event;
      bno.getEvent(&event);
      differential(32, -32);
      current_angle = event.orientation.x;
      
      Serial.print("R Current Heading: ");
      Serial.println(current_angle);
    }
  }
  else if(current_angle > final_angle){
    while(current_angle > final_angle){  
      sensors_event_t event;
      bno.getEvent(&event);
      differential(-32, 32);
      current_angle = event.orientation.x;
      Serial.print("L Current Heading: ");
      Serial.println(current_angle);
    }
  }
  
  differential(0, 0);
}
void drive_f(){//follows the sidewalk detected by the camra
    // Ping ultrasonic sensor and get distance
  sensors_event_t event;
  bno.getEvent(&event); 
  long duration, cm;
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(3);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);
  cm = microsecondsToCentimeters(duration);
  // Check if distance is far enough instead of true
  static int leftspeed = 0;
  static int rightspeed = 0;
  
  
  if(cm > 100){
    if(Serial.available()){
        char ch = Serial.read();
        float cond = Serial.parseFloat();
        int p;
        Serial.print(ch);
        Serial.println(cond);
        if(ch =='D'){
          p = (int)(kp * cond);
          rightspeed = 65 - p;
          leftspeed = 65 + p;
          Serial.print("Leftspeed: ");
          Serial.println(leftspeed);
          Serial.print("Rightspeed: ");
          Serial.print(rightspeed);

        }
    }
    // set diff using vars
    differential(leftspeed, rightspeed);
    isStopped = false;
  }
  else{
    estop();
    if(isStopped == false){
      Serial.println("STOP");
    }
    isStopped = true;
  }
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

  Serial.begin(9600);
  Serial.println("Starting...");
  /* Initialise the gyroscope */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

    bno.setExtCrystalUse(true);
}

long microsecondsToCentimeters(long microseconds){
  return microseconds / 29/ 2;
}
long last_timestamp = 0, current_timestamp;
// r & l pwr variables
void loop(){
// enter route here
  gyroturn(90);
  int t = millis();
  while(millis() < t + 500){
    differential(64, 64);
  }
  t = millis();
  while(millis() < t + 40000){
    drive_f();
  }
  gyroturn(300);
  t = millis();
  while(millis() < t + 300){
    drive_f();
  }
  gyroturn(330);
  while(millis() < t + 1000){
    differential(64, 64);
  }
  t = millis();
  while(millis() < t + 400){
    drive_f();
  }
  gyroturn(180);
  gyroturn(180);
  while(true){
    differential(0, 0);
  }
}
