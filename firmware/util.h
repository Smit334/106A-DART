typedef unsigned short uint16_t;
typedef unsigned int uint32_t;

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

#define IMU_I2C_ADDR 0x68
#define IMU_DATA_REG_START_ADDR 0x03
#define IMU_INIT_ADDR 0x20
#define IMU_INIT_VALUE 0x4000 | 0x0000 | 0x0000 | 0x0020 | 0x0007

#define NUM_MOTORS 4
#define PWM_FREQ_HZ 25000  /* Increase from default 4.482 kHz */

/* PINOUT (for Teensy 4.1)  */

/* PWM (motor control)*/
const uint32_t MOTORS[] = { 2, 3, 4, 5 };

/* DIGITAL (ultrasonic)*/
const uint32_t US_TRIG[] = { 33, 34, 35, 36 };
const uint32_t US_ECHO[] = { 37, 38, 39, 40 };

float dt;
uint32_t currTime;
uint32_t prevTime;

inline float deg2rad(float deg) {
    return deg * 0.0174533f;
}