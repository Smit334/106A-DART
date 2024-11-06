// #ifndef FLIGHTCONTROLLER_H
// #define FLIGHTCONTROLLER_H

#include "util.h"

#define MIN_THROTTLE 1060

const uint32_t LOOP_FREQ_HZ = 2000;

//Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
const float B_madgwick = 0.04;  //Madgwick filter parameter

//Controller parameters (take note of defaults before modifying!): 
const float i_limit = 25.0;     //Integrator saturation level, mostly for safety (default 25.0)
const float maxRoll = 30.0;     //Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode 
const float maxPitch = 30.0;    //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
const float maxYaw = 160.0;     //Max yaw rate in deg/sec

const float Kp_roll = 0.2;    //Roll P-gain - angle mode 
const float Ki_roll = 0.3;    //Roll I-gain - angle mode
const float Kd_roll = 0.05;   //Roll D-gain - angle mode (has no effect on controlANGLE2)
const float Kp_pitch = 0.2;   //Pitch P-gain - angle mode
const float Ki_pitch = 0.3;   //Pitch I-gain - angle mode
const float Kd_pitch = 0.05;  //Pitch D-gain - angle mode (has no effect on controlANGLE2)

const float Kp_yaw = 0.3;           //Yaw P-gain
const float Ki_yaw = 0.05;          //Yaw I-gain
const float Kd_yaw = 0.00015;       //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)

float q0; //Initialize quaternion for madgwick filter
float q1;
float q2;
float q3;

//Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des;
float roll_passthru, pitch_passthru, yaw_passthru;

//Controller:
float integral_yaw_prev, integral_pitch_prev, error_yaw_prev, integral_roll_prev;

float dt;
uint32_t currTime;
uint32_t prevTime;

void initConstants();

void Madgwick6DOF(IMUData *imu, float dt, RPYAngles *angles);
void controlANGLE(IMUData *imu, RPYAngles *actual, RPYAngles *des, float throttle, RPYAngles *pid);
void controlMixer(RPYAngles *pid, float throttle, MotorCommands *cmds);
float invSqrt(float value);

void trackLoopTime();
void limitLoopRate()

// #endif /* FLIGHTCONTROLLER_H */