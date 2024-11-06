#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include "util.h"

const uint32_t MIN_THROTTLE = 1060;

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

void initConstants(void);

void Madgwick6DOF(IMUData *imu, RPYAngles *angles);
void controlANGLE(IMUData *imu, RPYAngles *actual, RPYAngles *des, float throttle, RPYAngles *pid);
void controlMixer(RPYAngles *pid, float throttle, MotorCommands *cmds);
float invSqrt(float value);

#endif /* FLIGHTCONTROLLER_H */