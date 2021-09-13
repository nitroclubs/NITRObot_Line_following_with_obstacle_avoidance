//Line_following_5_sensors
#include <Arduino.h>

// The rangefinders work well to show the distance to objects from around
// 1 inch (2 cm) to around 9 feet away (3 meters), but they have trouble when
// they aren't approximately at a right angle to the object they are detecting.
// If the angle is too great (over about 15 degrees) not enough of the sound
// bounces back for it to get a reliable range.

//#include <Servo.h>

#define LEFT_FOR 9    // PWMB
#define LEFT_BACK 5   // DIRB  ---  Left
#define RIGHT_FOR 6   // PWMA
#define RIGHT_BACK 10 // DIRA  ---  Right

#define LN_SENS_PIN_RIGHTEDGE 22 // right edge sensor - Connected to D1 pin of the sensor
// #define LN_SENS_PIN_RIGHT 23       // right sensor - Connected to D2 pin of the sensor
#define LN_SENS_PIN_RIGHT 25  // right sensor - Connected to D2 pin of the sensor
#define LN_SENS_PIN_MIDDLE 24 // middle sensor - Connected to D3 pin of the sensor
// #define LN_SENS_PIN_LEFT 25       // left sensor Connected to D4 pin of the sensor
#define LN_SENS_PIN_LEFT 23     // left sensor Connected to D4 pin of the sensor
#define LN_SENS_PIN_LEFTEDGE 26 // left edge sensor - Connected to D5 pin of the sensor
#define LN_SENS_CALIB_PIN 27    // Connected to CAL pin of the sensor
#define LN_SENS_ANALOG_PIN A15  // Connected to AN pin of the sensor

const int LeftIrAvoidancePin = 12;
const int RightIrAvoidancePin = A5;
const int UltrasonicPin = 3;
const int RgbPin = 2;
const int ServoPin = 13;
const int LedPin = 33;

// Robot parameters:
// Robot length measured on the robot is 25.0 cm.
// Robot width measured on the robot is  16.7 cm.

// Maze parameters:
// In order for the robot to be able to safely make an U turn,
// we will choose the maze width to be 3 times the robot width,
// which is equal to 50.1, we will approximate this value to 50 cm.
const int MazeCorridorWidth = 50;

// Tresholds:
const float FrontDistanceTreshold = MazeCorridorWidth / 2;
const float WallToCorridorMiddle = MazeCorridorWidth / 2;
const float SideCorridorTreshold = MazeCorridorWidth;

const float CenterLineTolerance = 2.5; // plus/minus how many cm are acceptable to consider the movement to be on the center line...
                                       // +- 1 cm from centerline is considered straight movement!!!
const float SharpTurnTreshold = 15.0;  // Measured by experiments with the robot
const int WallFollowingSide = -90;     //Set: -90 for right wall following or +90 for left wall following
                                       //we will add this value to the servo position i.e. myservo.write(90 + WallFollowingSide);
                                       // in order to set to which side the servo should move (0 or 180 degrees)
//Servo parameters
const int FrontServoAngle = 90;
const int SideServoAngle = FrontServoAngle + WallFollowingSide; //(0 or 180 degrees)
const int FrontServoDelay = 90;
const int SideServoDelay = 90;

const int LeftSpeed =100; //да се подбере оптималната скорост на левия двигател
const int RightSpeed =100; //да се подбере оптималната скорост на десния двигател

float maxDistance = 130.0;
int speedLeft = LeftSpeed;
int speedRight = RightSpeed;
int currentState = 0;

//Servo myservo;

void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void stopMoving();
float getDistance(int servoAngle, int delayAfterServoMovement); //read the Ultasonic Sensor pointing at the given servo angle


int rightEdge();

//-----------------------------------------------

void setup()
{
  pinMode(LN_SENS_PIN_RIGHTEDGE, INPUT);
  pinMode(LN_SENS_PIN_RIGHT, INPUT);
  pinMode(LN_SENS_PIN_MIDDLE, INPUT);
  pinMode(LN_SENS_PIN_LEFT, INPUT);
  pinMode(LN_SENS_PIN_LEFTEDGE, INPUT);
  // pinMode(LN_SENS_CALIB_PIN, OUTPUT);
  pinMode(LN_SENS_ANALOG_PIN, INPUT);

  pinMode(ServoPin, OUTPUT);
  pinMode(LEFT_FOR, OUTPUT);
  pinMode(LEFT_BACK, OUTPUT);
  pinMode(RIGHT_FOR, OUTPUT);
  pinMode(RIGHT_BACK, OUTPUT);
  pinMode(UltrasonicPin, OUTPUT);
  pinMode(LedPin, OUTPUT);
  Serial.begin(9600);

  // moveForward();
  delay(500);
}

//---------------------------------------------------------

void loop()
{
 //
 int leftEdge = digitalRead(LN_SENS_PIN_RIGHTEDGE);
 int left = digitalRead(LN_SENS_PIN_RIGHT);
 int mid = digitalRead(LN_SENS_PIN_MIDDLE);
 int right = digitalRead(LN_SENS_PIN_LEFT);
 int rightEdge = digitalRead(LN_SENS_PIN_LEFTEDGE);
 
  if ((leftEdge == 0) && (left == 0) && (mid == 1) && (right == 0) && (rightEdge == 0)) //                0 0 1 0 0
  {
    currentState = 1;
    Serial.println("case1");
  }
  else if ((leftEdge == 0) && (left == 1) && (mid == 1) && (right == 0) && (rightEdge == 0)) //           0 1 1 0 0
  {
    currentState = 2;
     Serial.println("case2");
  }
  else if ((leftEdge == 0) && (left == 1) && (mid == 0) && (right == 0) && (rightEdge == 0)) //         0 1 0 0 0
  {
    currentState = 3;
     Serial.println("case3");
  }
  else if ((leftEdge == 1) && (left == 1) && (mid == 0) && (right == 0) && (rightEdge == 0)) //       1 1 0 0 0
  {
    currentState = 4;
     Serial.println("case4");
  }
  else if ((leftEdge == 1) && (left == 0) && (mid == 0) && (right == 0) && (rightEdge == 0)) //     1 0 0 0 0
  {
    currentState = 5;
     Serial.println("case5");    
  }
  else if ((leftEdge == 0) && (left == 0) && (mid == 1) && (right == 1) && (rightEdge == 0)) //           0 0 1 1 0
  {
    currentState = 6;
     Serial.println("case6");
  }
  else if ((leftEdge == 0) && (left == 0) && (mid == 0) && (right == 1) && (rightEdge == 0)) //         0 0 0 1 0
  {
    currentState = 7;
    Serial.println("case7");
  }
  else if ((leftEdge == 0) && (left == 0) && (mid == 0) && (right == 1) && (rightEdge == 1)) //       0 0 0 1 1
  {
    currentState = 8;
     Serial.println("case8");
  }
  else if ((leftEdge == 0) && (left == 0) && (mid == 0) && (right == 0) && (rightEdge == 1)) //     0 0 0 0 1
  {
    currentState = 9;    
    Serial.println("case9");
  }
  else if (((leftEdge == 1) && (left == 1) && (mid == 0) && (right == 1) && (rightEdge == 1))|| // 1 1 0 1 1
          ((leftEdge == 1) && (left == 0) && (mid == 0) && (right == 1) && (rightEdge == 1))|| // 1 0 0 1 1
          ((leftEdge == 1) && (left == 1) && (mid == 0) && (right == 0) && (rightEdge == 1))|| // 1 1 0 0 1
          ((leftEdge == 1) && (left == 1) && (mid == 1) && (right == 0) && (rightEdge == 1))|| // 1 1 1 0 1
          ((leftEdge == 1) && (left == 0) && (mid == 1) && (right == 1) && (rightEdge == 1)))  // 1 0 1 1 1
 {
   currentState = 10;
 }
 else
 {
   moveForward();
 }

 if (frontThreshold()<10){
   
   stopMoving();   //стоп
    delay(250);
    speedLeft = 255;      
    speedRight = 255;
    turnRight();
    delay(350);   //завой на дясно на 90 градуза (задава се продължителността на завоя)
    speedLeft = LeftSpeed;      
    speedRight = RightSpeed;
    moveForward();  // придвижване 20 см напред (задава се продължителността на придвижването)
    delay(750);
    speedLeft = 255;    
    speedRight = 255;
    turnLeft();
    delay(380);  //завой на ляво на 90 градуза (задава се продължителността на завоя)
    speedLeft = LeftSpeed;      
    speedRight = RightSpeed;
    moveForward();
    delay(1200);  // придвижване 30 см напред (задава се продължителността на придвижването)
    speedLeft = 255;    
    speedRight = 255;
    turnLeft();  //завой на ляво на 80 градуза (задава се продължителността на завоя)
    delay(300); 
     stopMoving();
     delay(100);
     leftEdge = digitalRead(LN_SENS_PIN_RIGHTEDGE);
     left = digitalRead(LN_SENS_PIN_RIGHT);
     mid = digitalRead(LN_SENS_PIN_MIDDLE);
    right = digitalRead(LN_SENS_PIN_LEFT);
    rightEdge = digitalRead(LN_SENS_PIN_LEFTEDGE);
   while ((leftEdge == 0) && (left == 0) && (mid ==0) && (right == 0) && (rightEdge == 0))
   {
    speedLeft = LeftSpeed;      
    speedRight = RightSpeed;
    moveForward(); 
     leftEdge = digitalRead(LN_SENS_PIN_RIGHTEDGE);
     left = digitalRead(LN_SENS_PIN_RIGHT);
     mid = digitalRead(LN_SENS_PIN_MIDDLE);
    right = digitalRead(LN_SENS_PIN_LEFT);
    rightEdge = digitalRead(LN_SENS_PIN_LEFTEDGE); 
    }
     stopMoving();
     delay(100);
   speedLeft = 255;      //въртене на дясно
    speedRight = 255;
    turnRight();
    delay(150);
   }

  switch (currentState)
  {
  case 1:
    speedLeft = LeftSpeed;
    speedRight = RightSpeed;
    moveForward();
    Serial.println("1");  
    break;
  case 2:
    speedLeft = LeftSpeed * .6;
    speedRight = 180;
    moveForward();
    Serial.println("2");
    break;
  case 3:
    speedLeft = LeftSpeed * .4;
    speedRight = 200;
    moveForward();
    Serial.println("3");
    break;
  case 4:
    speedLeft = LeftSpeed * .3;
    speedRight = 220;
    Serial.println("4");
    moveForward();
    break;
  case 5:
    speedLeft = LeftSpeed * .2;
    speedRight = 255;
    moveForward();
    Serial.println("5");    
    break;
  case 6:
    speedLeft = 180;
    speedRight = RightSpeed * .6;
    moveForward();
    Serial.println("6");
    break;
  case 7:
    speedLeft = 200;
    speedRight = RightSpeed * .4;    
    moveForward();
    Serial.println("7");
    break;
  case 8:
    speedLeft = 220;
    speedRight = RightSpeed * .3;
    moveForward();
    Serial.println("8");
    break;
  case 9:
    speedLeft = 255;
    speedRight = RightSpeed * .2;
    moveForward();
     Serial.println("9");
    break;
  case 10:
    stopMoving();  
     delay(1000);
    Serial.println("10");
    break;
  default:
    break;
  }
}

//==================================== FUNCTIONS =====================================================

void moveForward() // Move forward
{
  analogWrite(LEFT_FOR, abs(speedLeft));
  analogWrite(LEFT_BACK, LOW);
  analogWrite(RIGHT_FOR, abs(speedRight));
  analogWrite(RIGHT_BACK, LOW);
}

void moveBackward() // Move backward
{
  analogWrite(LEFT_FOR, LOW);
  analogWrite(LEFT_BACK, abs(speedLeft));
  analogWrite(RIGHT_FOR, LOW);
  analogWrite(RIGHT_BACK, abs(speedRight));
}

void turnLeft() // Turn Left
{
  analogWrite(LEFT_FOR, LOW);
  analogWrite(LEFT_BACK, speedLeft);
  analogWrite(RIGHT_FOR, speedLeft);
  analogWrite(RIGHT_BACK, LOW);
}

void turnRight() // Turn Right
{
  analogWrite(LEFT_FOR, speedRight);
  analogWrite(LEFT_BACK, LOW);
  analogWrite(RIGHT_FOR, LOW);
  analogWrite(RIGHT_BACK, speedRight);
}

void stopMoving() // Stop movement
{
  analogWrite(LEFT_FOR, HIGH);
  analogWrite(LEFT_BACK, HIGH);
  analogWrite(RIGHT_FOR, HIGH);
  analogWrite(RIGHT_BACK, HIGH);
}

float frontThreshold()
{
  float distance;  
  pinMode(UltrasonicPin, OUTPUT);
  digitalWrite(UltrasonicPin, LOW);
  delayMicroseconds(2);
  digitalWrite(UltrasonicPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(UltrasonicPin, LOW);
  pinMode(UltrasonicPin, INPUT);
  distance = pulseIn(UltrasonicPin, HIGH) / 58.00;
  return distance;
}
