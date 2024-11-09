#ifndef DRIVECONTROLLER_H
#define DRIVECONTROLLER_H

#include "util.h"

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
void arcadeDrive(float x, float y, float *out);

/**
 * Mix sided powers [0, 1] into motor commands. Assumes tank drive.
 * 
 * @param sidedPowers a float[2] array of [0, 1] powers; 
 *                    left = array[0], right = array[1]
 * @param outCommands pointer to MotorCommands to set (output)
 */
void driveControlMixer(float *sidedPowers, MotorCommands *outCommands);

#endif /* DRIVECONTROLLER_H */