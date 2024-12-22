#include <Servo.h>

Servo left;
Servo right;

void setup() {
  // put your setup code here, to run once:
  left.attach(A0);
  right.attach(A1);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  // servo.write(180);
  // delay(1000);
  setFlightMode();
  Serial.println("Fly");
  delay(5000);
  setDriveMode();
  Serial.println("Drive");
  delay(5000);
}

void setFlightMode(void) {
  left.write(90);
  right.write(90);
}

void setDriveMode(void) {
  left.write(0);
  right.write(180);
}
