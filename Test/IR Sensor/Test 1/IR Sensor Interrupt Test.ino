#define irSensorPin       2
#define ledPin            13

// ISR
void detectBlackLine();

void setup()
{
  pinMode(irSensorPin, INPUT);
  pinMode(ledPin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(irSensorPin), detectBlackLine, FALLING);

  digitalWrite(ledPin, LOW);
}

void loop()
{
  delay(50);
}

void detectBlackLine()
{
  while (digitalRead(irSensorPin) == LOW)
  {
    digitalWrite(ledPin, HIGH);
  }
  digitalWrite(ledPin, LOW);
}
