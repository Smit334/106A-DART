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
 * @param out DriveCommands power set [-1, 1] for left and right.
 */
void arcadeDrive(float x, float y, DriveCommands *cmds);

#endif /* DRIVECONTROLLER_H */