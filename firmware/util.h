#ifndef UTIL_H
#define UTIL_H

#include <Arduino.h>

struct {
  uint16_t ACC_X;
  uint16_t ACC_Y;
  uint16_t ACC_Z;
  uint16_t GYR_X;
  uint16_t GYR_Y;
  uint16_t GYR_Z;
} typedef IMUData;

struct {
  float yaw;
  float pitch;
  float roll;
} typedef RPYAngles;

struct {
  float frontLeft;
  float frontRight;
  float backRight;
  float backLeft;
} typedef MotorCommands;

struct {
  RPYAngles des;
  float throttle;
  bool isFlightMode;
} typedef RadioPacket;

#define IMU_I2C_ADDR 0x68
#define IMU_DATA_REG_START_ADDR 0x03
#define IMU_INIT_ADDR 0x20
#define IMU_INIT_VALUE 0x4000 | 0x0000 | 0x0000 | 0x0020 | 0x0007

// TODO make sure these are correct (I do not think they are)
#define RADIO_CE_PIN 9
#define RADIO_CSN_PIN 10

#define NUM_MOTORS 4
#define PWM_FREQ_HZ 60  /* Increase from default 4.482 kHz */

/* PINOUT (for Teensy 4.1)  */

/* PWM (motor control)*/
const uint32_t MOTORS[] = { 6, 9, 10, 11 }; //{ 0, 1, 3, 2 };

/* DIGITAL (ultrasonic)*/
const uint32_t US_TRIG[] = { 4, 5, 6, 7, 8 };
const uint32_t US_ECHO[] = { 28, 29, 30, 31, 32 };

// NOTE: the "LL" at the end of the constant is "LongLong" type
const uint64_t PIPE = 0xE8E8F0F0E1LL; // Define the transmit pipe

inline float deg2rad(float deg) {
    return deg * 0.0174533f;
}

#endif /* UTIL_H */