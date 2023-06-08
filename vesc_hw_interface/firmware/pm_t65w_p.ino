#include <Arduino.h>

const int output1Pin = 2;  // black cable -> D2
bool detected = false;

void setup()
{
  Serial.begin(9600);
  pinMode(output1Pin, INPUT);
}

void loop()
{
  detected = digitalRead(output1Pin);
  Serial.write(detected);
  delay(10);
}
