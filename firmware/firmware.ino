#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

extern "C" {
  #include "util.h"
  #include "sensors.h"
  #include "flightController.h"
  #include "driveController.h"
}

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
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(IMU_INIT_ADDR);
  Wire.write(IMU_INIT_VALUE);
  Wire.endTransmission(false);

  /* Initialize radio (over SPI) */
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  radio.setChannel(110);
  radio.openReadingPipe(1, PIPE);
  radio.startListening();

  /* Initialize pin modes for motors and servos. */
  for (uint32_t ch = 0; ch < NUM_MOTORS; ++ch) {
    pinMode(FLY_MOTORS[ch], OUTPUT);
    pinMode(DRIVE_SERVOS[ch], OUTPUT);
    analogWriteFrequency(DRIVE_SERVOS[ch], PWM_FREQ_HZ);
  }

  /* Initialize pin modes for ultrasonic sensors. */
  for (uint32_t ch = 0; ch < NUM_US; ++ch) {
    pinMode(US_TRIG[ch], OUTPUT);
    pinMode(US_ECHO[ch], INPUT);
  }

  packet.isFlightMode = false;
}

void loop() {
  trackLoopTime();

  readIMU(&imu);
  Madgwick6DOF(&imu, &position);
  for (uint32_t ch = 0; ch < NUM_MOTORS; ++ch) {
    ultrasonicInches[ch] = usToInches(readUltrasonicAveraged(ch));
  }
  radio.read(&packet, sizeof(packet));

  if (packet.isFlightMode) {
    controlANGLE(&imu, &position, &packet.des, packet.throttle, &pid);
    controlMixer(&pid, packet.throttle, &cmds);
  } else {
    arcadeDrive(packet.des.roll, packet.throttle, arcadePower);
    driveControlMixer(arcadePower, &cmds);
  }
  
  sendMotorCommands(&cmds, packet.isFlightMode);
  clearMotorCommands(&cmds);

  limitLoopRate();
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


void sendMotorCommands(MotorCommands *cmds, bool isFlightMode) {
  float cmd;
  if (isFlightMode) {
    for (uint32_t ch = 0; ch < NUM_MOTORS; ++ch) {
      cmd = *(&cmds->frontLeft + ch);
      analogWrite(FLY_MOTORS[ch], FLY_PWM_MIN + (floor(cmd) * FLY_PWM_RANGE));
    }
  } else {
    for (uint32_t ch = 0; ch < NUM_MOTORS; ++ch) {
      cmd = *(&cmds->frontLeft + ch);
      analogWrite(DRIVE_SERVOS[ch], floor(cmd));
    }
  }
}

void clearMotorCommands(MotorCommands *cmds) {
  for (uint32_t ch = 0; ch < NUM_MOTORS; ++ch) {
    *(&cmds->frontLeft + ch) = 0;
  }
}