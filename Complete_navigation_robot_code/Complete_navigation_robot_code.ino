//libraries
#include <Servo.h>
#include <QTRSensors.h>
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


//diy line following library
const int motorRPin1 = 22; // signal pin 1 for the right motor, connect to IN1
const int motorRPin2 = 24;  // signal pin 2 for the right motor, connect to IN2
const int motorREnable = 6; // enable pin for the right motor (PWM enabled)

const int motorLPin1 = 23; // signal pin 1 for the left motor, connect to IN3
const int motorLPin2 = 25; // signal pin 2 for the left motor, connect to IN4
const int motorLEnable = 5; // enable pin for the left motor (PWM enabled)

/* Define the pins for the IR receivers */
const int irPins[6] = {A14, A13, A12, A11, A10, A15};

/* Define values for the IR Sensor readings */

// an array to hold values from analogRead on the ir sensor (0-1023)
int irSensorAnalog[6] = {0, 0, 0, 0, 0, 0};

// an array to hold boolean values (1/0) for the ir sensors
int irSensorDigital[6] = {0, 0, 0, 0, 0, 0};

int treashold = 700; // IR sensor treashold value for line detection

// binary representation of the sensor reading
//from left to right when facing the same direction as the robot
int irSensors = B000000;

int count = 0; // number of sensors detecting the line

// A score to determine deviation from the line [-180 ; +180].
// Negative means the robot is left of the line.
int error = 0;

int errorLast = 0;  //  store the last value of error

// A correction value, based on the error from target.
// It is used to change the relative motor speed with PWM.
int correction = 0;

int lap = 0; // keep track of the laps

/* Set up maximum speed and speed for turning (to be used with PWM) */
int maxSpeed = 100; // used for PWM to control motor speed [0 - 255]

/* variables to keep track of current speed of motors */
int motorLSpeed = 0;
int motorRSpeed = 0;
//--ends


//ultrasonic properties
#define Echo 29
#define Trig 28
#define Echo1 33
#define Trig1 32

//wall following pid contants
const double KPW = 10;
const double KDW = 0.1;
double lastErrorW = 0;
const int GOALW = 10;
const unsigned char MAX_SPEEDW = 40;
int distanceWall, distanceFront, distanceRight;
bool obstacleDetected;
int turnDelay = 2000;
bool detected;
bool trulyObstacle;
unsigned int counter;
const bool left = 0;
const bool right = 1;

//servo

Servo servo;
#define servoPin 8
#define servoRightPos 10
#define servoLeftPos 15
#define servoCenterPos 85

//line sensor properties

#define NUM_SENSORS             6  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  20  // average 4 analog samples per sensor reading
#define EMITTER_PIN             QTR_NO_EMITTER_PIN  // emitter is controlled by digital pin 2
QTRSensorsAnalog qtra((unsigned char[]) {
  A14, A13, A12, A11, A10, A15
}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
unsigned int position;

//bluetooth variables

String s;           //stores incoming bluetooth data

//motor driver properties

L298 driver(24, 22, 6, 25, 23, 5);
const double KP = 0.04;
const double KD = 0.0;
double lastError = 0;
const int GOAL = 2500;
const unsigned char MAX_SPEED = 50;


// option which lets you choose between two functions
bool typeOfLineFollowing = 0;    //1 for qtr line following
bool followWall= 1;              //variable for choosing if you want to follow wall or not
void setup() {
  // put your setup code here, to run once:
  driver.init();
  calibrateSensor();            //used to calibrate line sensors
  servo.attach(servoPin);
  servo.write(85);
  Serial.begin (9600);   //com port serial
  Serial1.begin(9600);   //bluetooth module serial
  pinMode(Echo, INPUT);
  pinMode(Trig, OUTPUT);
  pinMode(Echo1, INPUT);
  pinMode(Trig1, OUTPUT);
  if (typeOfLineFollowing == noQtr) {   //0 means without qtr library
    pinMode(motorLPin1, OUTPUT);
    pinMode(motorLPin2, OUTPUT);
    pinMode(motorLEnable, OUTPUT);
    pinMode(motorRPin1, OUTPUT);
    pinMode(motorRPin2, OUTPUT);
    pinMode(motorREnable, OUTPUT);
    /* Set-up IR sensor pins as input */
    for (int i = 0; i < 6; i++) {
      pinMode(irPins[i], INPUT);
    }
    TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM01) | _BV(WGM00);
    TCCR0B = _BV(CS00);
  } else {}
}

void loop() {
  s = bluetoothHandler();     // catches incoming bluetooth data
  position = qtra.readLine(sensorValues);   //gets line sensors readin
  checkWallDistance();
  checkFrontDistance();
  if (distanceFront <= 15) {
    obstacleDetected = true; //check if obstacle is detected
  }
  if (!follow_wall) {         //if follow line is not true
    if (typeOfLineFollowing = 0) {
      followLineNoQtr();
    } else {
      followLineWithQtr();
    }
  }
  if (followWall) {
    if (obstacleDetected) {
      avoidObstacle();
    } else follow_wall();
  }
}

void followLineWithQtr() {               //pid line following function use this if you are not using the other line following function
  int error = GOAL - position;
  // Compute motor adjustment
  int adjustment = KP * error + KD * (error - lastError);

  // Store error for next increment
  lastError = error;

  // Adjust motors
  driver.setMotorAPower(constrain(MAX_SPEED + adjustment, 0, MAX_SPEED));
  driver.setMotorBPower(constrain(MAX_SPEED - adjustment, 0, MAX_SPEED));
}

void calibrateSensor() {
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 100; i++)  // make the calibration take about 10 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  digitalWrite(LED_BUILTIN, LOW);     // turn off Arduino's LED to indicate we are through with calibration
}

String bluetoothHandler() {
  if (Serial1.available()) {
    s = Serial1.readString();
  }
  return s;
}

void Scan() {
  // Initialize the sensor counter and binary value
  count = 0;
  irSensors = B000000;

  for (int i = 0; i < 6; i++) {
    irSensorAnalog[i] = analogRead(irPins[i]);

    if (irSensorAnalog[i] >= treashold) {
      irSensorDigital[i] = 1;
    }
    else {
      irSensorDigital[i] = 0;
    }
    //Serial.print(irSensorAnalog[i]);
    //Serial.print("|");
    count = count + irSensorDigital[i];
    int b = 5 - i;
    irSensors = irSensors + (irSensorDigital[i] << b);
  }
}

void UpdateError() {

  errorLast = error;
  Serial.println(irSensors);
  switch (irSensors) {

    case B000000:
      if (errorLast < 0) {
        error = -180;
      }
      else if (errorLast > 0) {
        error = 180;
      }
      break;

    case B100000: // leftmost sensor on the line
      error = -150;
      break;

    case B010000:
      error = -90;
      break;

    case B001000:
      error = -30;
      break;

    case B000100:
      error = 30;
      break;

    case B000010:
      error = 90;
      break;

    case B000001: // rightmost sensor on the line
      error = 150;
      break;

    /* 2 Sensors on the line */

    case B110000:
      error = -120;
      break;

    case B011000:
      error = -60;
      break;

    case B001100:
      error = 0;
      break;

    case B000110:
      error = 60;
      break;

    case B000011:
      error = 120;
      break;

    /* 3 Sensors on the line */

    case B111000:
    case B011100:
      error = -150;
      break;

    case B000111:
    case B001110:
      error = 150;
      break;

    /* 4 Sensors on the line */
    case B111100:
      error = -150;
      break;

    case B111010:
      error = -150;
      break;

    case B001111:
      error = 150;
      break;

    case B010111:
      error = 150;
      break;

    /* 5 Sensors on the line */
    case B111110:
      error = -150;
      break;

    case B011111:
      error = +150;
      break;

    case B111111:
      lap = lap + 1;
      error = 0;
      break;

    default:
      error = errorLast;
  }
}

void UpdateCorrection() {

  if (error >= 0 && error < 30) {
    correction = 0;
  }

  else if (error >= 30 && error < 60) {
    correction = 15;
  }

  else if (error >= 60 && error < 90) {
    correction = 40;
  }

  else if (error >= 90 && error < 120) {
    correction = 55;
  }

  else if (error >= 120 && error < 150) {
    correction = 75;
  }

  else if (error >= 150 && error < 180) {
    correction = 255;
  }

  else if (error >= 180) {
    correction = 305;
  }

  if (error <= 0 && error > -30) {
    correction = 0;
  }

  else if (error <= -30 && error > -60) {
    correction = -15;
  }

  else if (error <= -60 && error > -90) {
    correction = -40;
  }

  else if (error <= -90 && error > -120) {
    correction = -55;
  }

  else if (error <= -120 && error > -150) {
    correction = -75;
  }

  else if (error <= -150 && error > -180) {
    correction = -255;
  }

  else if (error <= -180) {
    correction = -305;
  }

  /* Adjust the correction value if maxSpeed is less than 255 */
  correction = (int) (correction * maxSpeed / 255 + 0.5);

  if (correction >= 0) {
    motorRSpeed = maxSpeed - correction;
    motorLSpeed = maxSpeed;
  }

  else if (correction < 0) {
    motorRSpeed = maxSpeed;
    motorLSpeed = maxSpeed + correction;
  }
}

void Drive() {
  if (motorRSpeed > 255) {
    motorRSpeed = 255;
  }
  else if (motorRSpeed < -255) {
    motorRSpeed = -255;
  }

  if (motorLSpeed > 255) {
    motorLSpeed = 255;
  }
  else if (motorLSpeed < -255) {
    motorLSpeed = -255;
  }

  if (motorRSpeed > 0) { // right motor forward (using PWM)
    analogWrite(motorREnable, motorRSpeed);
    digitalWrite(motorRPin1, HIGH);
    digitalWrite(motorRPin2, LOW);
  }

  else if (motorRSpeed < 0) {// right motor reverse (using PWM)
    analogWrite(motorREnable, abs(motorRSpeed));
    digitalWrite(motorRPin1, LOW);
    digitalWrite(motorRPin2, HIGH);
  }

  else if (motorRSpeed == 0) { // right motor fast stop
    digitalWrite(motorREnable, HIGH);
    digitalWrite(motorRPin1, LOW);
    digitalWrite(motorRPin2, LOW);
  }

  if (motorLSpeed > 0) { // left motor forward (using PWM)
    analogWrite(motorLEnable, motorLSpeed);
    digitalWrite(motorLPin1, HIGH);
    digitalWrite(motorLPin2, LOW);
  }

  else if (motorLSpeed < 0) { // right motor reverse (using PWM)
    analogWrite(motorLEnable, abs(motorLSpeed));
    digitalWrite(motorLPin1, LOW);
    digitalWrite(motorLPin2, HIGH);
  }

  else if (motorLSpeed == 0) { // left motor fast stop
    digitalWrite(motorLEnable, HIGH);
    digitalWrite(motorLPin1, LOW);
    digitalWrite(motorLPin2, LOW);
  }
}

void followLineNoQtr() {
  Scan();
  UpdateError();
  UpdateCorrection();
  Drive();
}

void follow_wall() {     //pid controller for following wall
  int errorW = distanceWall - GOALW;
  // Compute motor adjustment
  int adjustmentW = KPW * errorW + KDW * (errorW - lastErrorW);

  // Store error for next increment
  lastErrorW = errorW;
  // Adjust motors
  driver.setMotorAPower(constrain(MAX_SPEED + adjustmentW, 0, MAX_SPEED));
  driver.setMotorBPower(constrain(MAX_SPEED - adjustmentW, 0, MAX_SPEED));
}

int distance_test() {
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
  if(servo.read() != 10){
  servo.write(10);     // move the servo to right position
  delay(1000);
  }
  distanceRight = distance_test1();
}

void checkWallDistance() {       
  distanceWall = distance_test();
}   

void avoidObstacle() {           //main obstacle avoidance function
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
