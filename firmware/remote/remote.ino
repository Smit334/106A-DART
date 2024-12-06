#include <nRF24L01.h>
#include <RF24.h>

extern "C" {
  #include "util.h"
}

RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);

RadioPacket packet;

void setup() {
  Serial.begin(115200);

  /* Initialize radio to transmit/write */
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  radio.setChannel(110);
  radio.openWritingPipe(RADIO_ADDR);

  /* Set all non-radio pins to INPUT */
  pinMode(LEFT_JS_X_PIN, INPUT);
  pinMode(LEFT_JS_Y_PIN, INPUT);
  pinMode(LEFT_JS_SEL_PIN, INPUT);

  pinMode(RIGHT_JS_X_PIN, INPUT);
  pinMode(RIGHT_JS_Y_PIN, INPUT);
  pinMode(RIGHT_JS_SEL_PIN, INPUT);

  pinMode(FLIGHT_MODE_SW_PIN, INPUT);
  pinMode(AUTO_MODE_SW_PIN, INPUT);
  pinMode(BTN_ONE_PIN, INPUT);
  pinMode(BTN_TWO_PIN, INPUT);
  pinMode(BTN_THREE_PIN, INPUT);
}

void loop() {
  /* Read inputs and construct radio packet */
  packet.leftJoystickX = 1023 - analogRead(LEFT_JS_X_PIN);
  packet.leftJoystickY = 1023 - analogRead(LEFT_JS_Y_PIN);
  packet.leftJoystickSelect = digitalRead(LEFT_JS_SEL_PIN);

  packet.rightJoystickX = 1023 - analogRead(RIGHT_JS_X_PIN);
  packet.rightJoystickY = 1023 - analogRead(RIGHT_JS_Y_PIN);
  packet.rightJoystickSelect = digitalRead(RIGHT_JS_SEL_PIN);

  packet.vehicleMode = digitalRead(FLIGHT_MODE_SW_PIN);
  packet.isAutoMode = digitalRead(AUTO_MODE_SW_PIN);
  packet.buttonOne = digitalRead(BTN_ONE_PIN);
  packet.buttonTwo = digitalRead(BTN_TWO_PIN);
  packet.buttonThree = digitalRead(BTN_THREE_PIN);

  /* Write radio packet */
  radio.write(&packet, sizeof(packet));
  /* Do not delay after sending as vehicle loop time is very fast */
  delay(100);
  printPacket(&packet);
  Serial.println(radio.isChipConnected());
}

void printPacket(RadioPacket *packet) {
  Serial.println("PACKET");

  Serial.print("Left Joystick X: ");
  Serial.println(packet->leftJoystickX);
  Serial.print("Left Joystick Y: ");
  Serial.println(packet->leftJoystickY);
  Serial.print("Left Joystick Select: ");
  Serial.println(packet->leftJoystickSelect);

  Serial.print("Right Joystick X: ");
  Serial.println(packet->rightJoystickX);
  Serial.print("Right Joystick Y: ");
  Serial.println(packet->rightJoystickY);
  Serial.print("Right Joystick Select: ");
  Serial.println(packet->rightJoystickSelect);

  Serial.print("Vehicle Mode: ");
  Serial.println(packet->vehicleMode);
  Serial.print("Auto Mode: ");
  Serial.println(packet->isAutoMode);
  Serial.print("Button One: ");
  Serial.println(packet->buttonOne);
  Serial.print("Button Two: ");
  Serial.println(packet->buttonTwo);
  Serial.print("Button Three: ");
  Serial.println(packet->buttonThree);
}
