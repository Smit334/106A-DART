#include <Wire.h>

typedef unsigned short uint16_t;
typedef unsigned int uint32_t;

struct IMUData {
  uint16_t ACC_X,
  uint16_t ACC_Y,
  uint16_t ACC_Z,
  uint16_t GYR_X,
  uint16_t GYR_Y,
  uint16_t GYR_Z
};

#define IMU_I2C_ADDR 0x68
#define IMU_DATA_REG_START_ADDR 0x03
#define IMU_INIT_ADDR 0x20
#define IMU_INIT_VALUE 0x4000 | 0x0000 | 0x0000 | 0x0020 | 0x0007

#define NUM_MOTORS 4
#define PWM_FREQ_HZ 25000  /* Increase from default 4.482 kHz */

/* PINOUT (for Teensy 4.1)  */

/* PWM (motor control)*/
const uint32_t PWM_FLY[] = { 2, 3, 4, 5 };
const uint32_t PWM_DRIVE[] = { 6, 7, 8, 9 };

/* DIGITAL (ultrasonic)*/
const uint32_t US_TRIG[] = { 33, 34, 35, 36 };
const uint32_t US_ECHO[] = { 37, 38, 39, 40 };

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
void readIMU(struct IMUData *data) {
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