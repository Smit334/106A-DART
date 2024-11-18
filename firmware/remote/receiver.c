#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "driveController.h"
#include "flightController.h"
#include "util.h"

#define RADIO_CE_PIN 7
#define RADIO_CSN_PIN 8

RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);
const byte address[6] = "00001";

typedef struct {
  uint32_t yaw;
  uint32_t pitch;
  uint32_t roll;
} RPYAngles;

typedef struct {
  RPYAngles des;
  uint32_t throttle;
  bool isFlightMode;
  bool isAutoMode;
  bool buttons[NUM_BUTTONS];
} RadioPacket;

RadioPacket data;

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    radio.read(&data, sizeof(RadioPacket));
    
    if (data.isFlightMode) {
      // Apply flight mode controls
      controlANGLE(&imu, &position, &data.des, data.throttle, &pid);
      controlMixer(&pid, data.throttle, &flyCmds);
    } else {
      // Apply drive mode controls
      arcadeDrive(data.des.roll, data.throttle, &driveCmds);
    }
    
    // Debug output
    Serial.print("Mode: ");
    Serial.print(data.isFlightMode ? "Flight" : "Drive");
    Serial.print(" | Yaw: ");
    Serial.print(data.des.yaw);
    Serial.print(" | Pitch: ");
    Serial.print(data.des.pitch);
    Serial.print(" | Roll: ");
    Serial.print(data.des.roll);
    Serial.print(" | Throttle: ");
    Serial.print(data.throttle);
    Serial.println();
  }
}
