#ifndef UTIL_H
#define UTIL_H

#include <Arduino.h>

#define IMU_I2C_ADDR 0x68
#define IMU_DATA_REG_START_ADDR 0x03
#define IMU_INIT_ADDR 0x20
#define IMU_INIT_VALUE 0x4000 | 0x0000 | 0x0000 | 0x0020 | 0x0007

// TODO make sure these are correct (I do not think they are)
#define RADIO_CE_PIN 9
#define RADIO_CSN_PIN 10

#define NUM_MOTORS 4
#define NUM_US 5
#define NUM_TSERVO 2
#define PWM_FREQ_HZ 490

#define LOOP_RATE_HZ 2000

typedef struct {
  uint16_t ACC_X;
  uint16_t ACC_Y;
  uint16_t ACC_Z;
  uint16_t GYR_X;
  uint16_t GYR_Y;
  uint16_t GYR_Z;
} IMUData;

typedef struct {
  float yaw;
  float pitch;
  float roll;
} RPYAngles;

typedef struct {
  float frontLeft;
  float frontRight;
  float backRight;
  float backLeft;
} MotorCommands;

typedef struct {
  RPYAngles des;
  float throttle;
  bool isFlightMode;
} RadioPacket;

/* Loop timing */
extern float dt;
extern uint32_t currTime;
extern uint32_t prevTime;

/* PWM (motor/servo control)*/
extern const uint32_t FLY_MOTORS[NUM_MOTORS];
extern const uint32_t DRIVE_SERVOS[NUM_MOTORS];
extern const uint32_t TRANSITION_SERVOS[NUM_TSERVO];

/* DIGITAL (ultrasonic)*/
extern const uint32_t US_TRIG[NUM_US];
extern const uint32_t US_ECHO[NUM_US];

/* RADIO (nRF24L01) */
extern const uint64_t PIPE;

void trackLoopTime(void);
void limitLoopRate(void);

inline float deg2rad(float deg) {
  return deg * 0.0174533f;
}

#endif /* UTIL_H */