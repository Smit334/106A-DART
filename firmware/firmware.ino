#include <Wire.h>
#include "util.h"

void setup() {
  Serial.begin(115200);

  /* Initialize I2C. Automatically uses default I2C; SDA=18 and SCL=19 */
  Wire.begin();

  /* Initialize IMU (over I2C) */
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(IMU_INIT_ADDR);
  Wire.write(IMU_INIT_VALUE);
  Wire.endTransmission(false);

  /* Initialize pin modes for pin types with channels. */
  for (uint32_t ch = 0; ch < NUM_MOTORS; ++ch) {
    pinMode(PWM_FLY[ch], OUTPUT);
    analogWriteFrequency(PWM_FLY[ch], PWM_FREQ_HZ);
    pinMode(PWM_DRIVE[ch], OUTPUT);
    analogWriteFrequency(PWM_DRIVE[ch], PWM_FREQ_HZ);
    pinMode(US_TRIG[ch], OUTPUT);
    pinMode(US_ECHO[ch], INPUT);
  }
}

void loop() {
  // TODO populate
}

/*
 * Read ultrasonic sensor CH and return the measured distance in inches.
 *
 * @param ch the channel of the sensor to collect a measurement from
 * @return the reading of the ultrasonic sensor, in inches
 */
uint32_t readUltrasonicInches(uint32_t ch) {
  digitalWrite(US_TRIG[i], LOW);
  delayMicroseconds(2);  /* Ensure pin is reset */
  digitalWrite(US_TRIG[i], HIGH);
  delayMicroseconds(10);  /* 10us delay to start module */
  digitalWrite(US_TRIG[i], LOW);
  return pulseIn(US_ECHO[i], HIGH) / 58.0;
}

/*
 * Read a single IMU register.
 *
 * Assumes the IMU read operation has already been initiated.
 * Ordering and register synchroization is not maintained.
 * NOT for use outside readIMU(...)
 * 
 * @return the value of the IMU register as a uint16
 * 
 * @see readIMU(struct IMUData*)
 */
inline uint16_t readNextIMUReg(void) {
  return (Wire.read() << 8) | Wire.read();
}

/*
 * Read accelerometer and gyroscope on IMU over I2C.
 *
 * @param data the IMUData struct to store data in
 */
void readIMU(IMUData *data) {
  /* Start a request to read from the beginning of the IMU data registers */
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(IMU_DATA_REG_START_ADDR);
  Wire.endTransmission(false);

  /* Request 6 uint16's of IMU data from I2C */
  Wire.requestFrom(IMU_I2C_ADDR, 6 * sizeof(uint16_t));

  /* Read I2C data sequentially (in order of registers on IMU) */
  data->ACC_X = readNextIMUReg();
  data->ACC_Y = readNextIMUReg();
  data->ACC_Z = readNextIMUReg();
  data->GYR_X = readNextIMUReg();
  data->GYR_Y = readNextIMUReg();
  data->GYR_Z = readNextIMUReg();
}