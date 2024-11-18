#ifndef SENSORS_H
#define SENSORS_H

#include "util.h"

#define WINDOW_SIZE 5

uint32_t movingAverage(uint32_t value, uint32_t *buffer, uint32_t *accumulator, uint32_t *index);
uint32_t readUltrasonicAveraged(uint32_t ch);
uint32_t readUltrasonicRaw(uint32_t ch);

inline float usToInches(uint32_t us) {
  return us / 58.0f;
}

#endif /* SENSORS_H */