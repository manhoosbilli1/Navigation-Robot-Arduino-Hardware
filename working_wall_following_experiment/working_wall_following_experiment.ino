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


const double KP = 8;
const double KD = 0.0;
double lastError = 0;
const int GOAL = 10;
const unsigned char MAX_SPEED = 40;

void setup() {
  // put your setup code here, to run once:
  servo.attach(8);
  servo.write(90);
  delay(500);
  pinMode(Echo, INPUT);
  pinMode(Trig, OUTPUT);
  Serial.begin(9600);
  driver.init();
}

void loop() {
  // put your main code here, to run repeatedly:
  int distance = distance_test();
  if (distance > 15) {
    distance = 15;
  }
  if (distance < 5) {
    distance = 5;
  }
  Serial.println(distance);
  int error = distance - GOAL;
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
