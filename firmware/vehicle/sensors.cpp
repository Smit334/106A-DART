#include "sensors.h"

uint32_t avgAccs[NUM_US] = {};
uint32_t avgIdxs[NUM_US] = {};
uint32_t avgBufs[NUM_US][WINDOW_SIZE] = {};

Adafruit_MPU6050 imu;
Adafruit_Sensor *imuAccel;
Adafruit_Sensor *imuGyro;
sensors_event_t sensorEvent;

uint32_t movingAverage(uint32_t value, uint32_t *buffer, uint32_t *accumulator, uint32_t *index) {
  *accumulator -= buffer[*index];         // Remove the oldest entry from the sum
  buffer[*index] = value;                 // Add the newest reading to the window
  *accumulator += value;                  // Add the newest reading to the sum
  *index = (*index + 1) % WINDOW_SIZE;    // Increment the index, and wrap to 0 if it exceeds the window size
  return *accumulator / WINDOW_SIZE; 
}

uint32_t readUltrasonicAveraged(uint32_t ch) {
  uint32_t us = readUltrasonicRaw(ch);
  return movingAverage(us, avgBufs[ch], &avgAccs[ch], &avgIdxs[ch]);
}

uint32_t readUltrasonicRaw(uint32_t ch) {
  digitalWrite(US_TRIG[ch], LOW);
  delayMicroseconds(2);  /* Ensure pin is reset */
  digitalWrite(US_TRIG[ch], HIGH);
  delayMicroseconds(10);  /* 10us delay to start module */
  digitalWrite(US_TRIG[ch], LOW);
  return pulseIn(US_ECHO[ch], HIGH, 1);
}

void initIMU(void) {
  imu = Adafruit_MPU6050();
  imuAccel = imu.getAccelerometerSensor();
  imuGyro = imu.getGyroSensor();

  imu.begin();
  imu.setGyroRange(GYRO_SCALE);
  imu.setAccelerometerRange(ACCEL_SCALE);
}

void readIMU(IMUData *data) {
  imuAccel->getEvent(&sensorEvent);
  data->ACC_X = sensorEvent.acceleration.x;
  data->ACC_Y = sensorEvent.acceleration.y;
  data->ACC_Z = sensorEvent.acceleration.z;

  imuGyro->getEvent(&sensorEvent);
  data->GYR_X = sensorEvent.gyro.x;
  data->GYR_Y = sensorEvent.gyro.y;
  data->GYR_Z = sensorEvent.gyro.z;
}


