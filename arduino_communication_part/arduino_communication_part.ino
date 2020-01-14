/*this is the bluetooth communication part of the project
  using two serial ports to accomodate the hc-05
  Progress report
  1: 12/13/2020 -serial communication working okay unable to convert incoming string to human readable but is able tocompare and do an action based on that
  2: 12/14/2020 -fixed serial communication. readstring solved the problem and now receiving full string with this
  3: 12/14/2020 -created ir sensor handling system which outputs the ir sensor values as a string which can be later compared
*/

int irPins[] = {30, 33, 31, 28, 29, 32};    //TESTED WITH MY IR SENSORS ALL SHOWING OK
int irStatus[] = {0, 0, 0, 0, 0, 0};
String ir="";         //ACTUAL STORED VALUES OF SENSORS
String s = "";

void setup ()
{
  Serial.begin (9600);   //com port serial
  Serial1.begin(9600);   //bluetooth module serial
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("program start");
}
void loop ()
{
  ir = "";
  s = "";
  irSensorHandler();
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
