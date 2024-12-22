const uint32_t MOTORS[] = { 10 }; //{ 0, 1, 3, 2 };
#define NUM_MOTORS 1

uint32_t freq = 60;
uint32_t duty = 100;
uint32_t width = 0;

void setup() {
  pinMode(1, OUTPUT);
  // pinMode(6, OUTPUT);
  // pinMode(5, OUTPUT);
  // pinMode(11, OUTPUT);
  // pinMode(3, OUTPUT);
  // pinMode(9, OUTPUT);
  analogWriteFrequency(1, 490);
  // analogWrite(10, 175);
  // delay(1000);
  // Serial.println("DONE WITH INTI");
  
  // myPwm(225, 60);

  analogWrite(1, 130);
  delay(2000);
}

void loop() {
  // analogWrite(10, 200);
  // analogWrite(11, 50);
  // pwm10(duty, 60);
  // duty += 10;
  // delay(1000);
  // pwm11(50, 60);
  // put your main code here, to run repeatedly:
  // pwm_manual(width);
  // width += 1;
  // width %= 1000;
  // if (width % 20 == 0) {
  //   Serial.print("width: ");
  //   Serial.println(width);
  // }
  analogWrite(3, 190);
  // analogWrite(12, d);
  // pwm10(duty + 175, 490);
  // duty += 5;
  // duty %= 255;
  delay(1000);
  
  // myPwm(255, freq);
  // // delay(7000);
  // freq += 10;
  // // duty %= 255;
  Serial.print("duty: ");
  Serial.println(duty);
  // Serial.print("freq: ");
  // Serial.println(freq);
  // // myPwm(250, 250);
  // // analogWrite(10, 200);
  // digitalWrite(8, HIGH);
  // digitalWrite(9, LOW);
}

// void pwm10(unsigned char duty, float freq) {
//   TCCR1A = 0x21;
//   TCCR1B = 0x14;
//   OCR1A = 0x7A12 / freq;
//   OCR1B = OCR1A * (duty / 255.0);
// }

void pwm_manual(uint32_t width) {
  digitalWrite(10, HIGH);
  delayMicroseconds(width + 1000); // Approximately 10% duty cycle @ 1KHz
  digitalWrite(10, LOW);
  delayMicroseconds(1000 - width);
}

// void pwm11(unsigned char duty, float freq) {
//   TCCR3A = 0x21;
//   TCCR3B = 0x14;
//   OCR0A = 0x7A12 / freq;
//   OCR0B = OCR0A * (duty / 255.0);
// }
