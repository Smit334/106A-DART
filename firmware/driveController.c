#include "driveController.h"

/**
 * Calculates bounded motor outputs for an arcade drive system.
 *
 * Turns using the x input and fwd/back using the y.
 * Generally used to emulate the feel of driving in an "arcade"
 * instead of standard tank drive (west coast motor configuration).
 * May pass any input, most often used with two joysticks:
 * left driver Y axis and right driver X axis.
 *
 * @param x joystick input [-1, 1] used for turning.
 * @param y joystick input [-1, 1] used for forward/backward movement.
 * @param out power set [-1, 1] for left (index 0)
 *         and right (index 1) drive sides
 */
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
