#include "util.h"

float dt;
uint32_t currTime;
uint32_t prevTime;

const uint32_t FLY_MOTOR_PINS[NUM_FLY_MOTORS] = { 2, 1, 3, 0 }; //{ 0, 3, 1, 2 };
const uint32_t DRIVE_MOTOR_PINS[NUM_DRIVE_MOTORS] = { 15, 14 };
const uint32_t TRANSITION_SERVO_PINS[NUM_TSERVO] = { 25, 23 };

const uint32_t US_TRIG[NUM_US] = { 4, 5, 30, 7, 8 };
const uint32_t US_ECHO[NUM_US] = { 28, 29, 6, 31, 32 };

void trackLoopTime(void) {
  prevTime = currTime;
  currTime = micros();
  dt = (currTime - prevTime) / 1000000.0;
}

void limitLoopRate(void) {
  uint32_t elapsedLoopTime = micros() - currTime;
  if (elapsedLoopTime < LOOP_RATE_US) {
    delayMicroseconds(LOOP_RATE_US - elapsedLoopTime);
  }
}