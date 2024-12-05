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
DriveCommands driveCmds;
FlyCommands flyCmds;

RadioPacket packet;
float throttle;
RPYAngles des;

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
  radio.openReadingPipe(1, RADIO_ADDR);
  radio.startListening();

  /* Initialize pin modes for fly motors. */
  for (uint32_t ch = 0; ch < NUM_FLY_MOTORS; ++ch) {
    pinMode(FLY_MOTORS[ch], OUTPUT);
    analogWriteFrequency(FLY_MOTORS[ch], PWM_FREQ_HZ);
  }

  /* Initialize pin modes for drive motors. */
  for (uint32_t ch = 0; ch < NUM_DRIVE_MOTORS; ++ch) {
    pinMode(DRIVE_MOTORS[ch], OUTPUT);
    analogWriteFrequency(DRIVE_MOTORS[ch], PWM_FREQ_HZ);
  }

  /* Initialize pin modes for ultrasonic sensors. */
  for (uint32_t ch = 0; ch < NUM_US; ++ch) {
    pinMode(US_TRIG[ch], OUTPUT);
    pinMode(US_ECHO[ch], INPUT);
  }

  packet.buttons.isFlightMode = false;
}

void loop() {
  trackLoopTime();

  readIMU(&imu);
  Madgwick6DOF(&imu, &position);
  for (uint32_t ch = 0; ch < NUM_US; ++ch) {
    ultrasonicInches[ch] = usToInches(readUltrasonicAveraged(ch));
  }
  radio.read(&packet, sizeof(packet));
  decodeRadioPacket(&packet, &throttle, &des);

  if (packet.buttons.bits.isFlightMode) {
    controlANGLE(&imu, &position, &des, throttle, &pid);
    controlMixer(&pid, throttle, &flyCmds);
  } else {
    tankDrive(packet.left.y, packet.right.y, &driveCmds);
  }
  limitLoopRate();
}

void decodeRadioPacket(RadioPacket *packet, float *throttle, RPYAngles *des) {
    *throttle = packet->left.y;
    des->roll = packet->left.x;
    des->pitch = packet->right.y;
    des->yaw = packet->right.x;
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

void sendFlightCommands(FlyCommands *cmds) {
  float cmd;
  for (uint32_t ch = 0; ch < NUM_FLY_MOTORS; ++ch) {
    cmd = *(&cmds->frontLeft + ch);
    analogWrite(FLY_MOTORS[ch], FLY_PWM_MIN + (floor(cmd) * FLY_PWM_RANGE));
  }
}

void sendDriveCommands(DriveCommands *cmds) {
  float cmd;
  for (uint32_t ch = 0; ch < NUM_DRIVE_MOTORS; ++ch) {
    cmd = *(&cmds->left + ch);
    analogWrite(DRIVE_MOTORS[ch], floor(cmd));
  }
}