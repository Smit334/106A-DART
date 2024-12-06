#include "driveController.h"

void arcadeDrive(float x, float y, DriveCommands *cmds) {
  float max_v = max(abs(x), abs(y));
  float diff = y - x;
  float sum = y + x;
  if (y > 0) {
    if (x > 0) {
      cmds->left = max_v;
      cmds->right = diff;
    } else {
      cmds->left = sum;
      cmds->right = max_v;
    }
  } else {
    if (x > 0) {
      cmds->left = sum;
      cmds->right = -max_v;
    } else {
      cmds->left = -max_v;
      cmds->right = diff;
    }
  }
}

void tankDrive(uint16_t left, uint16_t right, DriveCommands *cmds) {
    cmds->left = left < LEFT_JOYSTICK_CTR ? 0 : ((left - LEFT_JOYSTICK_CTR) / 2);
    cmds->right = right < RIGHT_JOYSTICK_CTR ? 0 : ((right - RIGHT_JOYSTICK_CTR) / 2);
}