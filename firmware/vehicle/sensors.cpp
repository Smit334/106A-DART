#include "sensors.h"

uint32_t avgAccs[NUM_US] = {};
uint32_t avgIdxs[NUM_US] = {};
uint32_t avgBufs[NUM_US][WINDOW_SIZE] = {};

Adafruit_MPU6050 imu;
Adafruit_Sensor *imuAccel;
Adafruit_Sensor *imuGyro;
sensors_event_t sensorEvent;
IMUData *prevImuData;

uint32_t movingAverage(uint32_t value, uint32_t *buffer, uint32_t *accumulator, uint32_t *index) {
  *accumulator -= buffer[*index];         // Remove the oldest entry from the sum
  buffer[*index] = value;                 // Add the newest reading to the window
  *accumulator += value;                  // Add the newest reading to the sum
  *index = (*index + 1) % WINDOW_SIZE;    // Increment the index, and wrap to 0 if it exceeds the window size
  return *accumulator / WINDOW_SIZE; 
}

// float movingAverageFloat(float value, float *buffer, float *accumulator, uint32_t *index) {
//   *accumulator -= buffer[*index];         // Remove the oldest entry from the sum
//   buffer[*index] = value;                 // Add the newest reading to the window
//   *accumulator += value;                  // Add the newest reading to the sum
//   *index = (*index + 1) % WINDOW_SIZE_FLOAT;    // Increment the index, and wrap to 0 if it exceeds the window size
//   return *accumulator / WINDOW_SIZE_FLOAT; 
// }

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
  return pulseIn(US_ECHO[ch], HIGH);
}

void initIMU(TwoWire *wire) {
  imu = Adafruit_MPU6050();

  imu.begin(MPU6050_I2CADDR_DEFAULT, wire);
  imu.setFilterBandwidth(LPF_BW);
  imu.setGyroRange(GYRO_SCALE);
  imu.setAccelerometerRange(ACCEL_SCALE);

  imuAccel = imu.getAccelerometerSensor();
  imuGyro = imu.getGyroSensor();
}

void calibrateIMU(IMUData *error) {
  IMUData nextData;
  uint32_t i;
  for(i = 0; i < NUM_IMU_CALIBRATION_ITER; i++) {
    readIMU(&nextData);
    addIMUData(error, &nextData);
  }
  divIMUData(error, NUM_IMU_CALIBRATION_ITER);
}

// #define IMU_DATA_IDXS 6
// float avgImuAccs[IMU_DATA_IDXS] = {};
// uint32_t avgImuIdxs[IMU_DATA_IDXS] = {};
// float avgImuBufs[IMU_DATA_IDXS][WINDOW_SIZE] = {};

void readIMU(IMUData *data) {
  imuAccel->getEvent(&sensorEvent);
  data->accX = -sensorEvent.acceleration.x;
  data->accY = -sensorEvent.acceleration.y;
  data->accZ = -sensorEvent.acceleration.z;
  // data->accX = movingAverageFloat(-sensorEvent.acceleration.x, avgImuBufs[0], &avgImuAccs[0], &avgImuIdxs[0]);
  // data->accY = movingAverageFloat(-sensorEvent.acceleration.y, avgImuBufs[1], &avgImuAccs[1], &avgImuIdxs[1]);
  // data->accZ = movingAverageFloat(-sensorEvent.acceleration.z, avgImuBufs[2], &avgImuAccs[2], &avgImuIdxs[2]);

  imuGyro->getEvent(&sensorEvent);
  data->gyrX = sensorEvent.gyro.x;
  data->gyrY = sensorEvent.gyro.y;
  data->gyrZ = sensorEvent.gyro.z;
  // data->gyrX = movingAverageFloat(sensorEvent.gyro.x, avgImuBufs[3], &avgImuAccs[3], &avgImuIdxs[3]);
  // data->gyrY = movingAverageFloat(sensorEvent.gyro.y, avgImuBufs[4], &avgImuAccs[4], &avgImuIdxs[4]);
  // data->gyrZ = movingAverageFloat(sensorEvent.gyro.z, avgImuBufs[5], &avgImuAccs[5], &avgImuIdxs[5]);
}

void addIMUData(IMUData *a, IMUData *b) {
  a->accX += b->accX;
  a->accY += b->accY;
  a->accZ += b->accZ;

  a->gyrX += b->gyrX;
  a->gyrY += b->gyrY;
  a->gyrZ += b->gyrZ;
}

void subIMUData(IMUData *a, IMUData *b) {
  a->accX -= b->accX;
  a->accY -= b->accY;
  a->accZ -= b->accZ;

  a->gyrX -= b->gyrX;
  a->gyrY -= b->gyrY;
  a->gyrZ -= b->gyrZ;
}

void divIMUData(IMUData *data, uint32_t k) {
  data->accX /= k;
  data->accY /= k;
  data->accZ /= k;

  data->gyrX /= k;
  data->gyrY /= k;
  data->gyrZ /= k;
}

void printIMU(IMUData *data) {
  Serial.println("IMU DATA");
  
  Serial.print("Acceleration X: ");
  Serial.println(data->accX);
  Serial.print("Acceleration Y: ");
  Serial.println(data->accY);
  Serial.print("Acceleration Z: ");
  Serial.println(data->accZ);

  Serial.print("Gyroscope X: ");
  Serial.println(data->gyrX);
  Serial.print("Gyroscope Y: ");
  Serial.println(data->gyrY);
  Serial.print("Gyroscope Z: ");
  Serial.println(data->gyrZ);
}

