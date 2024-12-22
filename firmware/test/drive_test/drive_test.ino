void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(5, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(5, 255);
}
