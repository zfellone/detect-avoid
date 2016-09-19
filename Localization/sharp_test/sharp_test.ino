#include <SharpIR.h>

int distance;

SharpIR sharp(A0, 25, 93, 20150);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  distance = sharp.distance();
  Serial.println(distance);
  delay(20);
}
