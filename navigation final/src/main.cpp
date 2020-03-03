#include <DRV8835.h>
#include <L298.h>
#include <Servo.h>
#define EchoWall 31
#define TrigWall 30
#define EchoFront 32
#define TrigFront 35

Servo servo;
#define servoPin 8
#define servoRightPos 10
#define servoLeftPos 15
#define servoCenterPos 85

String s;
//ir
int ir[] = {A9, A10, A11};
int irADC[] = {0, 0, 0};
char irVal[] = {'0', '0', '0'};
const int irThreshold = 700;
String prevVal, linePos;
L298 driver(38, 37, 6, 36, 41, 5);
const double KP = 6.0; //you should edit this with your experiments....start with same values then increase or decrease as you like
const double KD = 20.0;
double lastError = 0;
const int GOAL = 20;
const unsigned char MAX_SPEED = 40;
int distanceWall, distanceFront, distanceRight;
bool obstacleDetected = false;
int turnDelay = 2000;
bool detected;
bool trulyObstacle = false;
;
unsigned int counter;
int i = 0;
int state = 0;
const bool left = 0;
const bool right = 1;
const int wallThreshold = 20;
const int obstacleThreshold = 20;
int targetCheckPoint = 0;
bool cmdStop; //commands from bluetooth
//turns
unsigned int prevMillis, prevMillis1, prevMillis2, prevMillis3; //timers
int degree180 = 400;
int degree90 = 185;
//progress pid wall following working.                            done
//add a checkpoint counting function inside wall following        upnext
//make a line following program
//take in bluetooth commands and control the robot
//try drawing checkpoints and telling the robot which checkpoint to go to via app and see if it follows

void setup()
{
  // put your setup code here, to run once:
  driver.init();
  servo.attach(servoPin);
  servo.write(70);
  Serial.begin(9600);
  Serial1.begin(9600); //bluetooth
  pinMode(EchoWall, INPUT);
  pinMode(TrigWall, OUTPUT);
  pinMode(EchoFront, INPUT);
  pinMode(TrigFront, OUTPUT);
  delay(10000);   //wait for the bluetooth to connect
  Serial1.println("Robot Connected");
  delay(100);
  
}

//-------------------functions-------------------

void wall_distance()
{ //returns ultrasonic distance
  digitalWrite(TrigWall, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigWall, HIGH);
  delayMicroseconds(20);
  digitalWrite(TrigWall, LOW);
  float Fdistance = pulseIn(EchoWall, HIGH);
  Fdistance = Fdistance / 58;
  distanceWall = (int)Fdistance;
}

void front_distance()
{ //returns ultrasonic distance
  digitalWrite(TrigFront, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigFront, HIGH);
  delayMicroseconds(20);
  digitalWrite(TrigFront, LOW);
  float Mdistance = pulseIn(EchoFront, HIGH);
  Mdistance = Mdistance / 58;
  distanceFront = (int)Mdistance;
}

void follow_wall()
{ //pid controller for following wall
  if (distanceWall >= 30)
  {
    distanceWall = 30;
  }
  if (distanceWall <= 10)
  {
    distanceWall = 10;
  }

  int error = distanceWall - GOAL;
  // Compute motor adjustment
  int adjustment = KP * error + KD * (error - lastError);

  // Store error for next increment
  lastError = error;
  // Adjust motors
  driver.setMotorAPower(constrain(MAX_SPEED + adjustment, 0, MAX_SPEED));
  driver.setMotorBPower(constrain(MAX_SPEED - adjustment, 0, MAX_SPEED));
}

void turn(bool dir, int turnDelay)
{ //type direction and for how long...only takes 'right' or 'left'
  if (dir == right)
  {
    driver.setMotorAPower(-100);
    driver.setMotorBPower(100);
    delay(turnDelay);
  }
  else if (dir == left)
  {
    driver.setMotorAPower(100);
    driver.setMotorBPower(-100);
    delay(turnDelay);
  }
}

void stop(int Delay)
{ //type in a value like 500 and the motor will stop for that amount of time
  driver.setMotorAPower(0);
  driver.setMotorBPower(0);
  delay(Delay);
}

void forward(int power, int Delay)
{ //type in with how much speed do you want to travel forward
  driver.setMotorAPower(power);
  driver.setMotorBPower(power);
  delay(Delay);
}

void avoidObstacle()
{ //main obstacle avoidance function
  stop(1000);
  turn(right, degree90);
  stop(1000);
  forward(40, 500); //move forward for 500ms with speed of 40
  stop(1000);       //stop for 500 second
  //check if the wall is still present
  wall_distance();
  if (distanceWall >= wallThreshold)
  {
    trulyObstacle = true;
    Serial1.println("false");
    delay(100);
  }
  else
  {
    obstacleDetected = false;
    trulyObstacle = false;
    Serial.println("wall present, exiting...");
    Serial1.println("following");
    delay(20);
  }
  while (trulyObstacle)
  {
    Serial.println("else was true and now in truly obstacle");
    Serial1.println("avoiding");
    delay(100);
    turn(left, degree90);
    stop(500);
    forward(40, 500); //go forward for one second
    stop(500);
    turn(left, degree90 - 50);
    stop(1000);
    forward(40, 300); //go forwards towards the wall
    stop(500);
    turn(right, degree90 + 50); //go a bit wide
    trulyObstacle = false;
    obstacleDetected = false;
  }
}

void follow_line()
{
  for (int i = 0; i <= 2; i++)
  {
    irADC[i] = analogRead(ir[i]);
    if (irADC[i] >= 700)
    {
      irVal[i] = '1';
    }
    else
    {
      irVal[i] = '0';
    }
  }
  linePos = irVal + 0x00;
  Serial.println(linePos);
  switch (state)
  {
  case 0:
    if (linePos == "101")
    {
      state = 1;
    }
    else if (linePos == "110")
    {
      state = 2;
    }
    else if (linePos == "011")
    {
      state = 3;
    }
    else if (linePos == "111" && prevVal != "110" && prevVal != "011")
    {
      state = 4; //stop the motor if not on black line ever
    }
    else if (linePos == "111" && prevVal == "110")
    { //if it derails after being on black line
      state = 5;
    }
    else if (linePos == "111" && prevVal == "011")
    { //if derails the other way
      state = 6;
    }
    else if (linePos == "000" && prevVal != "000")
    {
      state = 7; //if on checkpoint
    }
    break;

  case 1: //if on center
    driver.setMotorAPower(MAX_SPEED);
    driver.setMotorBPower(MAX_SPEED);
    state = 0;
    break;

  case 2: //if more towards the left
    driver.setMotorAPower(0);
    driver.setMotorBPower(MAX_SPEED);
    state = 0;
    break;

  case 3: //if more towards the right
    driver.setMotorAPower(MAX_SPEED);
    driver.setMotorBPower(0);
    state = 0;
    break;

  case 4: //stop the motors
    driver.setMotorAPower(0);
    driver.setMotorBPower(0);
    state = 0;
    break;

  case 5: //if derails after having been on black line
    driver.setMotorAPower(MAX_SPEED);
    driver.setMotorBPower(0);
    state = 0;
    break;

  case 6: //if derails the other way after having been on black line
    driver.setMotorAPower(0);
    driver.setMotorBPower(MAX_SPEED);
    state = 0;
    break;

  case 7: //checkpoint counter
    i += 1;
    state = 0;

    break;

  default:
    driver.setMotorBPower(MAX_SPEED);
    driver.setMotorAPower(MAX_SPEED);
  }
  prevVal = linePos; //save the last position
}

void checkPointCounter()
{
  if (millis() - prevMillis > 100)
  {
    for (int i = 0; i <= 2; i++)
    {
      irADC[i] = analogRead(ir[i]);
      if (irADC[i] >= 700)
      {
        irVal[i] = '1';
      }
      else
      {
        irVal[i] = '0';
      }
    }
    linePos = irVal + 0x00;
    if (linePos == "000" && prevVal != "000")
    {
      counter++;
      Serial.println("No of checkpoints");
      Serial.println(counter);
    }
    if (counter >= 20)
    {
      counter = 0;
    }
    prevVal = linePos;
    prevMillis = millis();
  }
}

void bluetooth_handler()
{
  if (Serial1.available())
  {
    s = Serial1.readString();
    Serial.print("Message received: ");
    Serial.println(s);
    if (s == "class 1" || s == "class one")
    {
      targetCheckPoint = 1;
    }
    else if (s == "class 2" || s == "class two")
    {
      targetCheckPoint = 2;
    }
    else if (s == "class 3" || s == "class three")
    {
      targetCheckPoint = 3;
    }
    else if (s == "class 4" || s == "class four")
    {
      targetCheckPoint = 4;
    }
    else if (s == "class 5" || s == "class five")
    {
      targetCheckPoint = 5;
    }
    else if (s == "class 6" || s == "class six")
    {
      targetCheckPoint = 6;
    }
    else if (s == "class 7" || s == "class seven")
    {
      targetCheckPoint = 7;
    }
    else if (s == "stop" || s == "STOP")
    {
      cmdStop = true;
      while (stop)
      {
        if (Serial1.available())
        {
          s = Serial1.readString();
          if (s == "Resume" || s == "resume")
          {
            cmdStop = false;
            s = "";
            break;
          }
        }
        driver.setMotorAPower(0);
        driver.setMotorBPower(0); // turn motors off and wait for command
      }
    }
  }
}

void guide(){

}
//-------------------functions end---------------

void loop()
{
  // put your main code here, to run repeatedly:
  wall_distance();     //checks wall distance
  front_distance();    //checks front distance
  checkPointCounter(); //counts checkpoints constantly
  bluetooth_handler(); //handles incoming bluetooth strings
  if (distanceFront <= 20)
  {
    obstacleDetected = true; //trigger if distance is below threshold
    Serial.println("Obstacle detected");
    Serial1.println("detected");
  }
  while (obstacleDetected)
  { //keep executing until obstacle is not detected
    avoidObstacle();
  }
  follow_wall(); //else follow wall
  delay(100);
}

//algorithm for line following
