#ifndef COMMON_H
#define COMMON_H

#pragma pack(1)

#include <stdint.h>

#define DRIVE_MODE 0
#define FLIGHT_MODE 1

#define RADIO_ADDR 0xE8E8F0F0E1LL

#define LEFT_JOYSTICK_CTR 512
#define RIGHT_JOYSTICK_CTR 522

typedef uint8_t vehicle_mode_t;

typedef struct {
    uint16_t leftJoystickX:10;
    uint16_t leftJoystickY:10;
    uint8_t leftJoystickSelect:1;

    uint16_t rightJoystickX:10;
    uint16_t rightJoystickY:10;
    uint8_t rightJoystickSelect:1;

    vehicle_mode_t vehicleMode:1;
    uint8_t isAutoMode:1;
    uint8_t buttonOne:1;
    uint8_t buttonTwo:1;
    uint8_t buttonThree:1;
} RadioPacket;

#endif