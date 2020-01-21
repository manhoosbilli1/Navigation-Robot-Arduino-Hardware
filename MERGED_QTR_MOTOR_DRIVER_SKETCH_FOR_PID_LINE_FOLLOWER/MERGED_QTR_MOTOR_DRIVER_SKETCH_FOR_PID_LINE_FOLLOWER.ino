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
 
// Line Sensor Properties
#define NUM_SENSORS             6  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  10  // average 4 analog samples per sensor reading
#define EMITTER_PIN             QTR_NO_EMITTER_PIN  // emitter is controlled by digital pin 2
 
QTRSensorsAnalog qtra((unsigned char[]) {A14, A13, A12, A11, A10, A15}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
 
// Motor Driver Properties
L298 driver(24, 22, 6, 25, 23, 5);
 
// PID Properties
const double KP = 0.020;
const double KD = 0.0;
double lastError = 0;
const int GOAL = 2500;
const unsigned char MAX_SPEED = 50;
 
 
void setup() {
  driver.init();
 
  // Initialize line sensor array
  calibrateLineSensor();
}
 
void loop() {
 
  // Get line position
  unsigned int position = qtra.readLine(sensorValues);
 
  // Compute error from line
  int error = GOAL -  position;
 
  // Compute motor adjustment
  int adjustment = KP*error + KD*(error - lastError);
 
  // Store error for next increment
  lastError = error;
 
  // Adjust motors 
  driver.setMotorAPower(constrain(MAX_SPEED + adjustment, 0, MAX_SPEED));
  driver.setMotorBPower(constrain(MAX_SPEED - adjustment, 0, MAX_SPEED));
 
}
 
void calibrateLineSensor() {
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 200; i++)  // make the calibration take about 10 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  digitalWrite(LED_BUILTIN, LOW);     // turn off Arduino's LED to indicate we are through with calibration
}
