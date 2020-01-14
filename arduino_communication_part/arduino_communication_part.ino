/*this is the bluetooth communication part of the project 
using two serial ports to accomodate the hc-05
Progress report 
1: 12/13/2020 -serial communication working okay unable to convert incoming string to human readable but is able to 
   compare and do an action based on that

2: 12/14/2020 -fixed serial communication. readstring solved the problem and now receiving full string with this 
*/

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

  s = "";
  while(Serial1.available()) {
    s = Serial1.readString(); 
  }
  if(s!= ""){
    Serial.println(s);
  }

  if(s == "*all on"){
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("working");
  }
  

}
