#include "driveController.h"

void arcadeDrive(float x, float y, float *out) {
  // TODO have this go in reverse too
  float *left = &out[0];
  float *right = &out[1];

  float max_v = max(abs(x), abs(y));
  float diff = y - x;
  float sum = y + x;
  if (y > 0) {
    if (x > 0) {
      *left = max_v;
      *right = diff;
    } else {
      *left = sum;
      *right = max_v;
    }
  } else {
    if (x > 0) {
      *left = sum;
      *right = -max_v;
    } else {
      *left = -max_v;
      *right = diff;
    }
  }
}

void driveControlMixer(float *sidedPowers, MotorCommands *outCommands) {
  outCommands->frontLeft = sidedPowers[0];
  outCommands->backLeft = sidedPowers[0];
  outCommands->frontRight = sidedPowers[1];
  outCommands->backRight = sidedPowers[1];
}
