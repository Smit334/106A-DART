#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "util.h"

RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);  // CE, CSN pins for the nRF24 module

const byte address[6] = "00001";  // Communication address for nRF24 module

// Define the analog pins for joystick inputs
#define THROTTLE_PIN A0        // Left joystick vertical (Throttle)
#define YAW_PIN A1             // Left joystick horizontal (Yaw)
#define PITCH_PIN A2           // Right joystick vertical (Pitch)
#define ROLL_PIN A3            // Right joystick horizontal (Roll)
#define MODE_BUTTON_PIN 7      // Digital pin for mode (flight/drive) for DEbugging unsure if we need this

// Define maximum and minimum joystick values
#define JOYSTICK_MAX 1023
#define JOYSTICK_MIN 0

// Struct definitions
typedef struct {
    uint32_t yaw;
    uint32_t pitch;
    uint32_t roll;
} RPYAngles;

typedef struct {
    RPYAngles des;
    uint32_t throttle;
    uint8_t isFlightMode;
    uint8_t isAutoMode;
    uint8_t buttons[NUM_BUTTONS];
} RadioPacket;

// Variables to hold joystick data and mode
RadioPacket data;
uint8_t flightMode = 1;  // 1 for Flight mode, 0 for Drive mode
uint8_t previousButtonState = LOW;  // Used for detecting button press changes

// Function to map values (analogous to Arduino's map() function)
int32_t mapValue(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
    pinMode(THROTTLE_PIN, INPUT);
    pinMode(YAW_PIN, INPUT);
    pinMode(PITCH_PIN, INPUT);
    pinMode(ROLL_PIN, INPUT);
    pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);  // Button for toggling mode

    radio.begin();
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_LOW);
    radio.stopListening();
}

void loop() {
    // Read button state to toggle mode
    uint8_t buttonState = digitalRead(MODE_BUTTON_PIN);
    if (buttonState == LOW && previousButtonState == HIGH) {
        flightMode = !flightMode;  // Toggle between Flight and Drive mode
    }
    previousButtonState = buttonState;

    // Read joystick values
    int throttleRaw = analogRead(THROTTLE_PIN);
    int yawRaw = analogRead(YAW_PIN);
    int pitchRaw = analogRead(PITCH_PIN);
    int rollRaw = analogRead(ROLL_PIN);

    // Map joystick values to control ranges
    data.throttle = mapValue(throttleRaw, JOYSTICK_MIN, JOYSTICK_MAX, 0, 255);

    if (flightMode) {
        // Flight Mode mappings
        data.des.yaw = mapValue(yawRaw, JOYSTICK_MIN, JOYSTICK_MAX, -127, 127);
        data.des.pitch = mapValue(pitchRaw, JOYSTICK_MIN, JOYSTICK_MAX, -127, 127);
        data.des.roll = mapValue(rollRaw, JOYSTICK_MIN, JOYSTICK_MAX, -127, 127);
    } else {
        // Drive Mode mappings
        data.des.yaw = mapValue(yawRaw, JOYSTICK_MIN, JOYSTICK_MAX, -127, 127);  // Steering
        data.des.pitch = 0;  // No pitch in Drive mode
        data.des.roll = mapValue(rollRaw, JOYSTICK_MIN, JOYSTICK_MAX, -127, 127);  // Forward/Backward speed
    }

    data.isFlightMode = flightMode;
    data.isAutoMode = !flightMode;  

    // Send data via nRF24
    radio.write(&data, sizeof(data));

    delay(50);  // Small delay to limit update frequency
}
