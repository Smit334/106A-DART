#ifndef UTIL_H
#define UTIL_H

#include <Arduino.h>

#define IMU_I2C_ADDR 0x68
#define IMU_DATA_REG_START_ADDR 0x03
#define IMU_INIT_ADDR 0x20
#define IMU_INIT_VALUE 0x4000 | 0x0000 | 0x0000 | 0x0020 | 0x0007

/* Number of motors (drive or fly) */
#define NUM_MOTORS 4

/* Number of ultrasonic sensors */
#define NUM_US 5

/* Number of servos (for state transition) */
#define NUM_TSERVO 2

/* PWM frequency for fly motors */
#define PWM_FREQ_HZ 490

/* Control loop frequency */
#define LOOP_RATE_HZ 2000

// TODO make sure these are correct (I do not think they are)
/* Radio pins */
#define RADIO_CE_PIN 9
#define RADIO_CSN_PIN 10

/* Stores data returned from IMU */
typedef struct {
  uint16_t ACC_X;
  uint16_t ACC_Y;
  uint16_t ACC_Z;
  uint16_t GYR_X;
  uint16_t GYR_Y;
  uint16_t GYR_Z;
} IMUData;

/* General RPY/YPR float values */
typedef struct {
  float yaw;
  float pitch;
  float roll;
} RPYAngles;

/* Motor control values, float range [0, 1] */
typedef struct {
  float frontLeft;
  float frontRight;
  float backRight;
  float backLeft;
} MotorCommands;

/* Packet received from radio */
typedef struct {
  RPYAngles des;
  float throttle;
  bool isFlightMode;
} RadioPacket;

/* Loop timing */
extern float dt;
extern uint32_t currTime;
extern uint32_t prevTime;

/* PWM pins (motor/servo control)*/
extern const uint32_t FLY_MOTORS[NUM_MOTORS];
extern const uint32_t DRIVE_MOTORS[NUM_MOTORS];
extern const uint32_t TRANSITION_SERVOS[NUM_TSERVO];

/* Digital pins (ultrasonic)*/
extern const uint32_t US_TRIG[NUM_US];
extern const uint32_t US_ECHO[NUM_US];

/* Radio constants (nRF24L01) */
extern const uint64_t PIPE;

/** Track loop time by measuring current time per loop. */
void trackLoopTime(void);

/** Enforce loop frequency by waiting until the loop time has elapsed. */
void limitLoopRate(void);

/**
 * Degree to radian utility function
 * 
 * Approximated to mul(0.0174533) instead of raw pi for performance optimization.
 * 
 * @param deg the angle value to convert, in degrees
 * @return the angle value converted to radians
 */
inline float deg2rad(float deg) {
  return deg * 0.0174533f;
}

#endif /* UTIL_H */