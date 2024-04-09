
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

void differential(int leftspeed, int rightspeed)
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
}

long microsecondsToCentimeters(long microseconds){
  return microseconds / 29/ 2;
}
// r & l pwr variables
void loop(){
  // Ping ultrasonic sensor and get distance
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
  //Serial.print("distance: ");
  //Serial.println(cm);
  // Check if distance is far enough instead of true
  static int leftspeed = 0;
  static int rightspeed = 0;
  if(cm > 50){
    if(Serial.available()){
        char ch = Serial.read();
        int cond = Serial.parseInt();
      
        Serial.print(ch);
        Serial.println(cond);
        if(ch =='D'){
          // set vars
          if(cond == 1/*condition 1*/){
            leftspeed = 64;
            rightspeed = 127;
          }
          else if(cond == 2/*condition 2*/){
            leftspeed = 100;
            rightspeed = 127;
          }
          else if(cond == 3  /*condition 3*/){
            leftspeed = 127;
            rightspeed = 127;
          }
          else if(cond == 4/*condition 4*/){
            leftspeed = 127;
            rightspeed = 100;
          }
          else if(cond == 5){/*condition 5*/
            leftspeed = 127;
            rightspeed = 64;
          }
        }
    }
    // set diff using vars
    differential(leftspeed, rightspeed);
  }
  else{
    estop();
    Serial.println("STOP");
  }
  
}
