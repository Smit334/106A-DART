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

void initIMU(TwoWire *wire) {
  imu = Adafruit_MPU6050();

  imu.begin(MPU6050_I2CADDR_DEFAULT, wire);
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

void readIMU(IMUData *data) {
  imuAccel->getEvent(&sensorEvent);
  data->accX = -sensorEvent.acceleration.x;
  data->accY = -sensorEvent.acceleration.y;
  data->accZ = sensorEvent.acceleration.z;

  imuGyro->getEvent(&sensorEvent);
  data->gyrX = -sensorEvent.gyro.x;
  data->gyrY = -sensorEvent.gyro.y;
  data->gyrZ = sensorEvent.gyro.z;
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

