/*this is the bluetooth communication part of the project
  using two serial ports to accomodate the hc-05
  Progress report
  1: 1/13/2020 -Serial communication working okay unable to convert incoming string to human readable but is able tocompare and do an action based on that
  2: 1/14/2020 -Fixed serial communication. readstring solved the problem and now receiving full string with this
  3: 1/14/2020 -Created ir sensor handling system which outputs the ir sensor values as a string which can be later compared
  4: 1/15/2020 -Added alot of new things like speed sensor, motor driver, servo, ultrasonic but they are unchecked and uncofirm need to rewire the whole car
  5: 1/20/2020 -Rewired the robot, raised the motor a bit to make way for ir sensors.
                motors, servo,bluetooth module and sd card now checked and working. robot ready for experimenation 
  6:1
*/
#include "TimerOne.h"
#include <Servo.h>
Servo servo;
#define Echo 26  //analog pin 
#define Trig 27  //analog pin 
#define ENA 6   //unchecked motor pins. remember to check it 
#define ENB 5
#define IN1 22
#define IN2 24
#define IN3 23
#define IN4 25
#define maxCarSpeed 100
#define speedSensorPin 2
#define servoPin 8
#define servoRightPos 10
#define servoLeftPos 150
#define servoCenterPos 85
int rightDistance, leftDistance, middleDistance;
volatile unsigned int counter = 0;
float diskslots = 20.00;
const float wheeldiameter = 66.10;
int irPins[] = {30, 33, 31, 28, 29, 32};    //TESTED WITH MY IR SENSORS ALL SHOWING OK
int irStatus[] = {0, 0, 0, 0, 0, 0};
String ir = "";       //ACTUAL STORED VALUES OF SENSORS
String s = "";

void setup ()
{
servo.attach(servoPin);
/*  Timer1.initialize(1000000);
  attachInterrupt(digitalPinToInterrupt(speedSensorPin), ISR_count1, RISING);
 Timer1.attachInterrupt(ISR_timerone);
 */
  Serial.begin (9600);   //com port serial
  Serial1.begin(9600);   //bluetooth module serial
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(Echo, INPUT);
  pinMode(Trig, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  Serial.println("program start");
  delay(100);

  
  
}
void loop ()
{
//  middleDistance = distance_test();   //get value when ultrasonic is in middle position
//  ir = "";    //dump already stored values in the variables
//  s = "";
/*  irSensorHandler();
  if(Serial.available()) {
     s = Serial.readString();
   }
   if(s == "f") {
    digitalWrite(ENA, HIGH);
    digitalWrite(ENB, HIGH);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
   }
    if(s == "b") {
    digitalWrite(ENA, HIGH);
    digitalWrite(ENB, HIGH);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
   }
   */

}
String blutoothHandler() {
  while (Serial1.available()) {
    s = Serial1.readString();
  }
  if (s != "") {
    Serial.println(s);
  }

  if (s == "*all on") {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("working");
  }
}
void irSensorHandler() {
  for (int i = 0; i < 6; i++) {
    irStatus[i] = digitalRead(irPins[i]);
    String c = String(irStatus[i]);
    ir += c;
    c = "";
  }

  Serial.println(ir);
}
void ISR_count1() {
  counter++;
}
void ISR_timerone() {
  Timer1.detachInterrupt();
  float rotation1 = (counter / diskslots) * 60.00;
  counter = 0;
  Timer1.attachInterrupt(ISR_timerone);
}
int CMtoSteps(float cm) {
  int result;
  float circumference = (wheeldiameter * 3.14) / 10;
  float cm_step = circumference / diskslots;
  float f_result = cm / cm_step;
  result = (int) f_result;
  return result;
}
void staticForward() {
  analogWrite(ENA, maxCarSpeed);
  analogWrite(ENB, maxCarSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
void staticBack() {
  analogWrite(ENA, maxCarSpeed);
  analogWrite(ENB, maxCarSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void staticRight() {
  analogWrite(ENA, maxCarSpeed);
  analogWrite(ENB, maxCarSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
void staticLeft() {
  analogWrite(ENA, maxCarSpeed);
  analogWrite(ENB, maxCarSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void stop() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
}
int distance_test() {      //used to find distance with ultra sonic
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(20);
  digitalWrite(Trig, LOW);
  float Fdistance = pulseIn(Echo, HIGH);
  Fdistance = Fdistance / 58;
  return (int)Fdistance;
}
void obstacleAvoidance() {  //a very basic obstacle avoidance 
  if (middleDistance <= 20) {
    stop();
    delay(500);
    servo.write(servoRightPos);
    delay(1000);
    rightDistance = distance_test();

    delay(500);
    servo.write(servoLeftPos);
    delay(1000);
    leftDistance = distance_test();
    delay(500);
    servo.write(90);
    delay(1000);
    if (rightDistance < leftDistance) {
      staticLeft();
      delay(360);
    }
    else if (rightDistance > leftDistance) {
      staticRight();
      delay(360);
    }
    else if (rightDistance <= 20 || leftDistance<= 20){
      staticBack();
      delay(360);    
    }else {
      staticForward();
    }

  } else {
    staticForward();
 }
}
