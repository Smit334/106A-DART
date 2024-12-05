#ifndef COMMON_H
#define COMMON_H

#pragma pack(1)

#include <stdint.h>

const uint64_t RADIO_ADDR = 0xE8E8F0F0E1LL;

typedef struct {
    uint16_t leftJoystickX:10;
    uint16_t leftJoystickY:10;
    uint8_t leftJoystickSelect:1;

    uint16_t rightJoystickX:10;
    uint16_t rightJoystickY:10;
    uint8_t rightJoystickSelect:1;

    uint8_t isFlightMode:1;
    uint8_t isAutoMode:1;
    uint8_t buttonOne:1;
    uint8_t buttonTwo:1;
    uint8_t buttonThree:1;
} RadioPacket;

#endif