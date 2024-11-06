#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include "util.h"
#include "flightController.h"

IMUData imu;
float ultrasonicInches[NUM_MOTORS];
RPYAngles position;
RPYAngles pid;
MotorCommands cmds;
float arcadePower[2];
RadioPacket packet;

RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);

void setup() {
  Serial.begin(115200);

  initConstants();

  /* Initialize I2C. Automatically uses default I2C; SDA=18 and SCL=19 */
  Wire.begin();

  /* Initialize IMU (over I2C) */
  // Wire.beginTransmission(IMU_I2C_ADDR);
  // Wire.write(IMU_INIT_ADDR);
  // Wire.write(IMU_INIT_VALUE);
  // Wire.endTransmission(false);

  /* Initialize radio (over SPI) */
  // radio.begin();
  // radio.setPALevel(RF24_PA_HIGH);
  // radio.setChannel(110);
  // radio.openReadingPipe(1, PIPE);
  // radio.startListening();

  /* Initialize pin modes for pin types with channels. */
  for (uint32_t ch = 0; ch < NUM_MOTORS; ++ch) {
    pinMode(MOTORS[ch], OUTPUT);
    analogWriteFrequency(MOTORS[ch], PWM_FREQ_HZ);
    // pinMode(US_TRIG[ch], OUTPUT);
    // pinMode(US_ECHO[ch], INPUT);
  }

  packet.isFlightMode = false;
}

void loop() {
  trackLoopTime();

  // readIMU(&imu);
  // Madgwick6DOF(&imu, dt, &position);
  // for (uint32_t ch = 0; ch < NUM_MOTORS; ++ch) {
  //   ultrasonicInches[ch] = readUltrasonicInches(ch);
  // }
  // radio.read(&packet, sizeof(packet));

  if (packet.isFlightMode) {
    controlANGLE(&imu, &position, &packet.des, packet.throttle, &pid);
    controlMixer(&pid, packet.throttle, &cmds);
  } else {
    // arcadeDrive(packet.des->roll, packet.throttle, &arcadePower);
    arcadePower[0] = 0.05;
    arcadePower[1] = 0.05;
    driveControlMixer(arcadePower, &cmds);
  }
  sendMotorCommands(&cmds);
  clearMotorCommands(&cmds);

  limitLoopRate(2000);
}

/**
 * Read ultrasonic sensor CH and return the measured distance in inches.
 *
 * @param ch the channel of the sensor to collect a measurement from
 * @return the reading of the ultrasonic sensor, in inches
 */
uint32_t readUltrasonicInches(uint32_t ch) {
  digitalWrite(US_TRIG[ch], LOW);
  delayMicroseconds(2);  /* Ensure pin is reset */
  digitalWrite(US_TRIG[ch], HIGH);
  delayMicroseconds(10);  /* 10us delay to start module */
  digitalWrite(US_TRIG[ch], LOW);
  return pulseIn(US_ECHO[ch], HIGH) / 58.0;
}

/**
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

/**
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

/**
 * Calculates bounded motor outputs for an arcade drive system.
 *
 * Turns using the x input and fwd/back using the y.
 * Generally used to emulate the feel of driving in an "arcade"
 * instead of standard tank drive (west coast motor configuration).
 * May pass any input, most often used with two joysticks:
 * left driver Y axis and right driver X axis.
 *
 * @param x joystick input [-1, 1] used for turning.
 * @param y joystick input [-1, 1] used for forward/backward movement.
 * @param out power set [-1, 1] for left (index 0)
 *         and right (index 1) drive sides
 */
void arcadeDrive(float x, float y, float *out) {
  // TODO have this go in reverse too
    float *left = &out[0];
    float *right = &out[1];

    float max_v = max(abs(x), abs(y));
    float diff = y - x;
    float sum = y + x;
    if (y > 0) {
        if (x > 0) {
            *left = max_v;
            *right = diff;
        } else {
            *left = sum;
            *right = max_v;
        }
    } else {
        if (x > 0) {
            *left = sum;
            *right = -max_v;
        } else {
            *left = -max_v;
            *right = diff;
        }
    }
}

void driveControlMixer(float *sidedPowers, MotorCommands *outCommands) {
  outCommands->frontLeft = sidedPowers[0];
  outCommands->backLeft = sidedPowers[0];
  outCommands->frontRight = sidedPowers[1];
  outCommands->backRight = sidedPowers[1];
}

void sendMotorCommands(MotorCommands *cmds) {
  float cmd;
  for (uint32_t ch = 0; ch < NUM_MOTORS; ++ch) {
    cmd = *(&cmds->frontLeft + ch) * 255;
    analogWrite(MOTORS[ch], floor(cmd));
  }
}

void clearMotorCommands(MotorCommands *cmds) {
  for (uint32_t ch = 0; ch < NUM_MOTORS; ++ch) {
    *(&cmds->frontLeft + ch) = 0;
  }
}