#define PIN_LED 7

void setup()
{
  pinMode(PIN_LED, OUTPUT);
}

void loop()
{
  digitalWrite(PIN_LED, 0);
  delay(1000);
  digitalWrite(PIN_LED, 1); //처음 1초
  
  for(int i = 0; i < 5; i++){
    digitalWrite(PIN_LED, 0);
    digitalWrite(PIN_LED, 1);
  }
  while(1)
  {
    digitalWrite(PIN_LED, 1);
  }
  
}
