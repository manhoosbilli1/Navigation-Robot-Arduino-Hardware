#include <DRV8835.h>
#include <HBridgeMotor.h>
#include <InInMotor.h>
#include <InInPWMMotor.h>
#include <L298.h>
#include <Motor.h>
#include <MotorDriver.h>
#include <PhaseEnableMotor.h>
#include <SN754410.h>
#include <TB67H420FTG.h>
#include <Servo.h>
L298 driver(24, 22, 6, 25, 23, 5);
Servo servo;
#define Echo 29
#define Trig 28
#define Echo1 33
#define Trig1 32

const double KP = 10;
const double KD = 0.1;
double lastError = 0;
const int GOAL = 10;
const unsigned char MAX_SPEED = 40;
bool obstacleDetected = false;
bool pathBlocked = false;
int distanceRight, distanceLeft, distanceFront;
int turnDelay = 2000;
int distanceMain;
int distanceWall;
bool detected;
bool trulyObstacle;
unsigned int counter;
const bool left = 0;
const bool right = 1;
void setup() {
  // put your setup code here, to run once:
  servo.attach(8);
  servo.write(85);
  delay(500);
  pinMode(Echo, INPUT);
  pinMode(Trig, OUTPUT);
  pinMode(Echo1, INPUT);
  pinMode(Trig1, OUTPUT);
  Serial.begin(9600);
  driver.init();
}

void loop() {
  // put your main code here, to run repeatedly:
  checkWallDistance();
  checkFrontDistance();
  if (distanceFront <= 15) { obstacleDetected = true; }
  if(obstacleDetected) {
  avoidObstacle();
  } else follow();
}

void avoidObstacle() {
  if (obstacleDetected) {
    stop(500);                                //stop
    turn(right, 350);                         //turn right for 350 ms this will turn car 90 degree
    stop(1000);                               //stop for a second
    forward(40, 1000);                        //go forward for a second
    stop(3000);                               //stop
    checkWallDistance();                      //checks wall distance again to see if it was a simply a wall or an obstacle
    if (distanceWall > 25) {
      trulyObstacle = true;                  //if the ditance is more than 25 that means there isn't any wall present so truly obstacle
    } else {
      Serial.println("it was a wall");
      trulyObstacle = false;
      obstacleDetected = false;
    }
    if (trulyObstacle) {
      Serial.println("Truly obstacle");
      turn(left, 350);
      stop(500);
      forward(40, 1000);
      while (distanceWall > 25) {
        checkWallDistance();
        Serial.println("going left until we can find the wall");
        driver.setMotorAPower(60);        //this is the speed thats going to make the robot curve towards the wall
        driver.setMotorBPower(40);
      }
      obstacleDetected = false;
      trulyObstacle = false;
    }
  }
}


void follow() {
  int error = distanceWall - GOAL;
  // Compute motor adjustment
  int adjustment = KP * error + KD * (error - lastError);

  // Store error for next increment
  lastError = error;
  // Adjust motors
  driver.setMotorAPower(constrain(MAX_SPEED + adjustment, 0, MAX_SPEED));
  driver.setMotorBPower(constrain(MAX_SPEED - adjustment, 0, MAX_SPEED));
}

int distance_test() {      //used to find distance with ultra sonic
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(20);
  digitalWrite(Trig, LOW);
  float Fdistance = pulseIn(Echo , HIGH);
  Fdistance = Fdistance / 58;
  return (int)Fdistance;
}

int distance_test1() {
  digitalWrite(Trig1, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig1, HIGH);
  delayMicroseconds(20);
  digitalWrite(Trig1, LOW);
  float Fdistance = pulseIn(Echo1, HIGH);
  Fdistance = Fdistance / 58;
  return (int)Fdistance;
}

void turn(bool dir, int turnDelay) {
  if (dir == right) {
    driver.setMotorAPower(-100);
    driver.setMotorBPower(100);
    delay(turnDelay);
  } else if (dir == left) {
    driver.setMotorAPower(100);
    driver.setMotorBPower(-100);
    delay(turnDelay);
  }
}

void stop(int Delay) {
  driver.setMotorAPower(0);
  driver.setMotorBPower(0);
  delay(Delay);
}

void forward(int power, int Delay) {
  driver.setMotorAPower(power);
  driver.setMotorBPower(power);
  delay(Delay);
}

void checkFrontDistance() {
  if (servo.read() != 85) {
    servo.write(85);      // move the servo to front
    delay(1000);
  }
  distanceFront = distance_test1();
}

void checkRightDistance() {
  servo.write(10);     // move the servo to right position
  delay(1000);
  distanceRight = distance_test1();
}

void checkWallDistance() {
  distanceWall = distance_test();
}
