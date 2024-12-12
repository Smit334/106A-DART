#ifndef UTIL_H
#define UTIL_H

#include <Arduino.h>
#include "common.h"

/* Number of motors (drive or fly) */
#define NUM_FLY_MOTORS 4
#define NUM_DRIVE_MOTORS 2

/* Number of ultrasonic sensors */
#define NUM_US 5

/* Number of servos (for state transition) */
#define NUM_TSERVO 2

/* PWM frequency for fly motors */
#define PWM_FREQ_HZ 490

#define MIN_MOTOR_US 1000
#define MAX_MOTOR_US 2000

/* Control loop frequency */
#define LOOP_RATE_US 500U

/* IMU parameters */
#define GYRO_SCALE MPU6050_RANGE_250_DEG
#define ACCEL_SCALE MPU6050_RANGE_2_G

/* Servo angles per vehicle mode */
#define LEFT_SERVO_FLIGHT_DEG 90
#define RIGHT_SERVO_FLIGHT_DEG 35
#define LEFT_SERVO_DRIVE_DEG 0
#define RIGHT_SERVO_DRIVE_DEG 130

/* Radio pins */
#define RADIO_CE_PIN 37
#define RADIO_CSN_PIN 10
#define RADIO_IRQ_PIN 38
#define RADIO_RX_TIMEOUT_MS 1000

#define I2C_SCL 16
#define I2C_SDA 17

/* LED pins (for debug / state display) */
#define LED_RED_A_PIN 16  /* USED FOR I2C */
#define LED_RED_B_PIN 17  /* USED FOR I2C */
#define LED_BLUE_A_PIN 20
#define LED_BLUE_B_PIN 21

/* Battery voltage reading (from ADC) */
#define V_BAT_PIN 41
#define V_BAT_DIV_RATIO (680.0f / (680.0f + 2000.0f))
#define V_BAT_OFFSET 0.15f
#define V_BAT_WARN_THRESH 9.6
#define VCC 3.3f
#define ADC_MAX 1023.0f

#define LEFT_JS_X_CTR 512.0f
#define LEFT_JS_Y_CTR 512.0f
#define RIGHT_JS_X_CTR 512.0f
#define RIGHT_JS_Y_CTR 522.0f

#define NUM_IMU_CALIBRATION_ITER 500

/* Stores data returned from IMU */
typedef struct {
  float accX;
  float accY;
  float accZ;
  float gyrX;
  float gyrY;
  float gyrZ;
} IMUData;

/* General RPY/YPR float values */
typedef struct {
  float yaw;
  float pitch;
  float roll;
} RPYAngles;

/* Flight motor control values, float range [0, 1] */
typedef struct {
  float frontLeft;
  float frontRight;
  float backRight;
  float backLeft;
} FlyCommands;

/* Drive motor control values, integer range [0, 255] */
typedef struct {
  uint8_t left;
  uint8_t right;
} DriveCommands;

/* Loop timing */
extern float dt;
extern uint32_t currTime;
extern uint32_t prevTime;

/* PWM pins (motor/servo control)*/
extern const uint32_t FLY_MOTOR_PINS[NUM_FLY_MOTORS];
extern const uint32_t DRIVE_MOTOR_PINS[NUM_DRIVE_MOTORS];
extern const uint32_t TRANSITION_SERVO_PINS[NUM_TSERVO];

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