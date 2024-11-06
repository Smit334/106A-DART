#include "flightController.h"

void initConstants() {
  q0 = 1.0f;
  q1 = 0.0f;
  q2 = 0.0f;
  q3 = 0.0f;
}

void Madgwick6DOF(IMUData *imu, float dt, RPYAngles *angles) {
  //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
  /*
   * See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
   * available (for example when using the recommended MPU6050 IMU for the default setup).
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  //Convert gyroscope degrees/sec to radians/sec
  float gx = deg2rad(imu->GYR_X);
  float gy = deg2rad(imu->GYR_Y);
  float gz = deg2rad(imu->GYR_Z);

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((imu->ACC_X == 0.0f) && (imu->ACC_Y == 0.0f) && (imu->ACC_Z == 0.0f))) {
    //Normalise accelerometer measurement
    recipNorm = invSqrt(imu->ACC_X * imu->ACC_X + imu->ACC_Y * imu->ACC_Y + imu->ACC_Z * imu->ACC_Z);
    float ax = imu->ACC_X * recipNorm;
    float ay = imu->ACC_Y * recipNorm;
    float az = imu->ACC_Z * recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    //Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  //Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //Compute angles
  angles->roll = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
  angles->pitch = -asin(constrain(-2.0f * (q1*q3 - q0*q2),-0.999999,0.999999))*57.29577951; //degrees
  angles->yaw = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
}

void controlANGLE(IMUData *imu, RPYAngles *actual, RPYAngles *des, float throttle, RPYAngles *pid) {
  //DESCRIPTION: Computes control commands based on state error (angle)
  /*
   * Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des computed in 
   * getDesState(). Error is simply the desired state minus the actual state (ex. roll_des - roll_IMU). Two safety features
   * are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent 
   * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
   * they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
   * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I 
   * terms will always start from 0 on takeoff. This function updates the variables pid->roll, pid->pitch, and pid->yaw which
   * can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in controlMixer().
   */
  
  //Roll
  float error_roll = des->roll - actual->roll;
  float integral_roll = integral_roll_prev + error_roll*dt;
  if (throttle < MIN_THROTTLE) {   //Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_roll = imu->GYR_X;
  pid->roll = 0.01*(Kp_roll*error_roll + Ki_roll*integral_roll - Kd_roll*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  float error_pitch = des->pitch - actual->pitch;
  float integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (throttle < MIN_THROTTLE) {   //Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_pitch = imu->GYR_Y;
  pid->pitch = .01*(Kp_pitch*error_pitch + Ki_pitch*integral_pitch - Kd_pitch*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  float error_yaw = des->yaw - imu->GYR_Z;
  float integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (throttle < MIN_THROTTLE) {   //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  pid->yaw = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  integral_roll_prev = integral_roll;
  //Update pitch variables
  integral_pitch_prev = integral_pitch;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void controlMixer(RPYAngles *pid, float throttle, MotorCommands *cmds) {
    cmds->frontLeft = throttle - pid->pitch + pid->roll + pid->yaw; //Front Left
    cmds->frontRight = throttle - pid->pitch - pid->roll - pid->yaw; //Front Right
    cmds->backRight = throttle + pid->pitch - pid->roll + pid->yaw; //Back Right
    cmds->backLeft = throttle + pid->pitch + pid->roll - pid->yaw; //Back Left
}

float invSqrt(float value) {
  return 1.0 / sqrt(value);
}

void trackLoopTime() {
  prevTime = currTime;
  currTime = micros();
  dt = (currTime - prevTime) / 1000000.0;
}

void limitLoopRate() {
  delayMicroseconds(freqHz - (micros() - currTime));
}