#include "util.h"

float dt;
uint32_t currTime;
uint32_t prevTime;

const uint32_t FLY_MOTORS[NUM_FLY_MOTORS] = { 0, 1, 3, 2 };
const uint32_t DRIVE_MOTORS[NUM_DRIVE_MOTORS] = { 14, 15 };
const uint32_t TRANSITION_SERVOS[NUM_TSERVO] = { 22, 25 };

const uint32_t US_TRIG[NUM_US] = { 4, 5, 6, 7, 8 };
const uint32_t US_ECHO[NUM_US] = { 28, 29, 30, 31, 32 };

void trackLoopTime(void) {
  prevTime = currTime;
  currTime = micros();
  dt = (currTime - prevTime) / 1000000.0;
}

void limitLoopRate(void) {
  delayMicroseconds(LOOP_RATE_HZ - (micros() - currTime));
}