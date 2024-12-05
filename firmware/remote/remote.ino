#include <nRF24L01.h>
#include <RF24.h>

extern "C" {
    #include "util.h"
}

RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);

RadioPacket packet;

void setup() {
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  radio.setChannel(110);
  radio.openWritingPipe(RADIO_ADDR);
}

void loop() {
  packet.leftJoystickX = analogRead(LEFT_JS_X_PIN);
  packet.leftJoystickY = analogRead(LEFT_JS_Y_PIN);
  packet.leftJoystickSelect = digitalRead(LEFT_JS_SEL_PIN);

  packet.rightJoystickX = analogRead(RIGHT_JS_X_PIN);
  packet.rightJoystickY = analogRead(RIGHT_JS_Y_PIN);
  packet.rightJoystickSelect = digitalRead(RIGHT_JS_SEL_PIN);

  packet.isFlightMode = digitalRead(FLIGHT_MODE_SW_PIN);
  packet.isAutoMode = digitalRead(AUTO_MODE_SW_PIN);
  packet.buttonOne = digitalRead(BTN_ONE_PIN);
  packet.buttonTwo = digitalRead(BTN_TWO_PIN);
  packet.buttonThree = digitalRead(BTN_THREE_PIN);

  radio.write(packet, sizeof(packet));
}