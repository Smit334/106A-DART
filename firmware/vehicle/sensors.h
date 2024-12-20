#ifndef SENSORS_H
#define SENSORS_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

extern "C" {
  #include "util.h"
}

#define WINDOW_SIZE 50
#define WINDOW_SIZE_FLOAT 30

#define US_FRONT_IDX 0
#define US_LEFT_IDX 1
#define US_RIGHT_IDX 2
#define US_BACK_IDX 3
#define US_BOTTOM_IDX 4

#define FRONT_OBSTACLE_THRESH_CM 25

uint32_t movingAverage(uint32_t value, uint32_t *buffer, uint32_t *accumulator, uint32_t *index);
uint32_t readUltrasonicAveraged(uint32_t ch);
uint32_t readUltrasonicRaw(uint32_t ch);

void initIMU(TwoWire *wire);
void calibrateIMU(IMUData *error);
void readIMU(IMUData *data);
void addIMUData(IMUData *a, IMUData *b);
void subIMUData(IMUData *a, IMUData *b);
void divIMUData(IMUData *data, uint32_t k);
void printIMU(IMUData *data);

inline float usToCentimeters(uint32_t us) {
  return us / 58.0f;
}

#endif /* SENSORS_H */