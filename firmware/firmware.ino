#include <PID_v1.h>
#include <Wire.h>

#define NUM_MOTORS 4
#define PWM_FREQ_HZ 25000  /* Increase from default 4.482 kHz */

/* PINOUT (for Teensy 4.1)  */

/* PWM (motor control)*/
const unsigned int PWM_FLY[] = { 2, 3, 4, 5 };
const unsigned int PWM_DRIVE[] = { 6, 7, 8, 9 };

/* DIGITAL (ultrasonic)*/
const unsigned int US_TRIG[] = { 33, 34, 35, 36 };
const unsigned int US_ECHO[] = { 37, 38, 39, 40 };

void setup() {
  Serial.begin(115200);
  Wire.begin();  /* Automatically uses default I2C; SDA=18 and SCL=19 */

  for (unsigned int ch = 0; ch < NUM_MOTORS; ++ch) {
    pinMode(PWM_FLY[ch], OUTPUT);
    analogWriteFrequency(PWM_FLY[ch], PWM_FREQ_HZ);
    pinMode(PWM_DRIVE[ch], OUTPUT);
    analogWriteFrequency(PWM_DRIVE[ch], PWM_FREQ_HZ);
    pinMode(US_TRIG[ch], OUTPUT);
    pinMode(US_ECHO[ch], INPUT);
  }
}

void loop() {

}

unsigned int readUltrasonicInches(unsigned int ch) {
  digitalWrite(US_TRIG[i], LOW);
  delayMicroseconds(2);  /* Ensure pin is reset */
  digitalWrite(US_TRIG[i], HIGH);
  delayMicroseconds(10);  /* 10us delay to start module */
  digitalWrite(US_TRIG[i], LOW);
  return pulseIn(US_ECHO[i], HIGH) / 58.0;
}

void readIMU() {

}