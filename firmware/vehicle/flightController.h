#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include "util.h"

const uint32_t MIN_THROTTLE = 1060; //Minimum throttle to effect output, in raw packet units
const uint8_t FLY_PWM_MIN = 175;    //Minimum PWM to move flight motors, in [0, 255]
const uint8_t FLY_PWM_RANGE = 80;   //PWM range for flight motors, in [0, 255]

//Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
const float B_madgwick = 0.04;      //Madgwick filter parameter

//Controller parameters (take note of defaults before modifying!): 
const float i_limit = 25.0;         //Integrator saturation level, mostly for safety (default 25.0)
const float maxRoll = 30.0;         //Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode 
const float maxPitch = 30.0;        //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
const float maxYaw = 160.0;         //Max yaw rate in deg/sec

const float Kp_roll = 0.2;          //Roll P-gain - angle mode 
const float Ki_roll = 0.3;          //Roll I-gain - angle mode
const float Kd_roll = 0.05;         //Roll D-gain - angle mode (has no effect on controlANGLE2)
const float Kp_pitch = 0.2;         //Pitch P-gain - angle mode
const float Ki_pitch = 0.3;         //Pitch I-gain - angle mode
const float Kd_pitch = 0.05;        //Pitch D-gain - angle mode (has no effect on controlANGLE2)

const float Kp_yaw = 0.3;           //Yaw P-gain
const float Ki_yaw = 0.05;          //Yaw I-gain
const float Kd_yaw = 0.00015;       //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)

/** Initialize flight controller constants. */
void initConstants(void);

/**
 * DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
 * 
 * See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
 * available (for example when using the recommended MPU6050 IMU for the default setup).
 */
void Madgwick6DOF(IMUData *imu, RPYAngles *angles);

/**
 * DESCRIPTION: Computes control commands based on state error (angle)
 * 
 * Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des computed in 
 * getDesState(). Error is simply the desired state minus the actual state (ex. roll_des - roll_IMU). Two safety features
 * are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent 
 * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
 * they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
 * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I 
 * terms will always start from 0 on takeoff. This function updates the variables pid->roll, pid->pitch, and pid->yaw which
 * can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in controlMixer().
 */
void controlANGLE(IMUData *imu, RPYAngles *actual, RPYAngles *des, float throttle, RPYAngles *pid);

/**
 * Mix PID'd RPY/YPR and throttle into motor commands
 * 
 * @param pid PID'd RPY/YPR control values
 * @param throttle controlled throttle value
 * @param cmds pointer to MotorCommands to set (output)
 */
void controlMixer(RPYAngles *pid, float throttle, MotorCommands *cmds);

/**
 * Inverse square root utility function
 * 
 * sqrt(...) function is used since Teensy has enough overhead.
 * Can be changed to be more approximate if performance is limited.
 * 
 * @param value float to take the square root of
 * @return 1 / sqrt(value) 
 */
inline float invSqrt(float value) {
  return 1.0 / sqrt(value);
}

#endif /* FLIGHTCONTROLLER_H */