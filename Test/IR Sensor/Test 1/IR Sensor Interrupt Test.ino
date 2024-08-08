#define irSensorPin       2
#define ledPin            13

// ISR
void detectWhiteLine();

void setup()
{
  pinMode(irSensorPin, INPUT);
  pinMode(ledPin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(irSensorPin), detectWhiteLine, RISING);

  digitalWrite(ledPin, LOW);
}

void loop()
{
  delay(50);
}

void detectWhiteLine()
{
  int sensorRead = analogRead(irSensorPin);
  while (digitalRead(irSensorPin)==HIGH)
  {
    digitalWrite(ledPin, HIGH);
  }
  digitalWrite(ledPin, LOW);
}
