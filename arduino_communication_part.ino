
String c = "";
void setup ()
{
  Serial.begin (9600);
  Serial1.begin(9600);
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
