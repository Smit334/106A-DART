#ifndef DRIVECONTROLLER_H
#define DRIVECONTROLLER_H

#include "util.h"

void arcadeDrive(float x, float y, float *out);
void driveControlMixer(float *sidedPowers, MotorCommands *outCommands);

#endif /* DRIVECONTROLLER_H */