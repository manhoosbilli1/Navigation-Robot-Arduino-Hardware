/*this is the bluetooth communication part of the project 
using two serial ports to accomodate the hc-05
Progress report 
1: 12/1/2020 -serial communication working okay unable to convert incoming string to human readable but is able to 
   compare and do an action based on that
*/
String c = "";

void setup ()
{
  Serial.begin (9600);   //com port serial
  Serial1.begin(9600);   //bluetooth module serial 
  pinMode(13, OUTPUT);
  Serial.println("program start");
}
void loop ()
{

  if (Serial1.available()) {
    c = Serial1.read();
    delay(10);
  }
  
  Serial.print(c);
  

  if(c.length() > 0) {
    if(c == "all on"){
      digitalWrite(13, HIGH);
    }
  }

}
