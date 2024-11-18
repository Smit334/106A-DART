#ifndef COMMON_H
#define COMMON_H

#pragma pack(1)

#include <stdint.h>

typedef struct {
    uint16_t x:10;
    uint16_t y:10;
    uint16_t sel:1;
} Joystick;

typedef struct {
    Joystick left;
    Joystick right;
    union {
        struct {
            uint8_t isFlightMode:1;
            uint8_t isAutoMode:1;
            uint8_t switchOne:1;
            uint8_t switchTwo:1;
            uint8_t switchThree:1;
        } bits;
        uint8_t all:5;
    } buttons;
} RadioPacket;

#endif