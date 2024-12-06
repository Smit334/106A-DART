#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

#include "sensors.h"

extern "C" {
  #include "util.h"
  #include "flightController.h"
  #include "driveController.h"
}

IMUData imuData;
float ultrasonicInches[NUM_US];
RPYAngles position;
RPYAngles pid;
DriveCommands driveCmds;
DriveCommands nullDriveCmds;
FlyCommands flyCmds;
FlyCommands nullFlyCmds;

RadioPacket packet;
float throttle;
RPYAngles des;
vehicle_mode_t lastVehicleMode;

RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);
Servo transitionServos[NUM_TSERVO];

void setup() {
  Serial.begin(115200);

  initConstants();
  
  /* Initialize I2C */
  Wire.begin();
  Wire.setClock(1000000);

  /* Initialize IMU (over I2C) */
  initIMU();

  SPI.begin();

  /* Initialize radio (over SPI) */
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  radio.setChannel(110);
  radio.openReadingPipe(1, RADIO_ADDR);
  radio.startListening();

  /* Initialize pin modes for fly motors. */
  for (uint32_t ch = 0; ch < NUM_FLY_MOTORS; ++ch) {
    pinMode(FLY_MOTOR_PINS[ch], OUTPUT);
    analogWriteFrequency(FLY_MOTOR_PINS[ch], PWM_FREQ_HZ);
  }

  /* Initialize pin modes for drive motors. */
  for (uint32_t ch = 0; ch < NUM_DRIVE_MOTORS; ++ch) {
    pinMode(DRIVE_MOTOR_PINS[ch], OUTPUT);
    analogWriteFrequency(DRIVE_MOTOR_PINS[ch], PWM_FREQ_HZ);
  }

  /* Initialize pin modes for ultrasonic sensors. */
  for (uint32_t ch = 0; ch < NUM_US; ++ch) {
    pinMode(US_TRIG[ch], OUTPUT);
    pinMode(US_ECHO[ch], INPUT);
  }

  /* Create and attach Servo objects to servo pins. */
  for (uint32_t ch = 0; ch < NUM_TSERVO; ++ch) {
    transitionServos[ch].attach(TRANSITION_SERVO_PINS[ch]);
    analogWriteFrequency(TRANSITION_SERVO_PINS[ch], PWM_FREQ_HZ);
  }

  setServos(FLIGHT_MODE);
  lastVehicleMode = FLIGHT_MODE;
}

void loop() {
  // trackLoopTime();

  // readIMU(&imuData);
  // Madgwick6DOF(&imuData, &position);
  // for (uint32_t ch = 0; ch < NUM_US; ++ch) {
  //   ultrasonicInches[ch] = usToInches(readUltrasonicAveraged(ch));
  // }
  
  Serial.println(radio.isChipConnected());
  while (!radio.available()) {
  }
  radio.read(&packet, sizeof(packet));
  printPacket(&packet);
  // decodeRadioPacket(&packet, &throttle, &des);
  Serial.println(packet.vehicleMode);
  transitionModeServos(packet.vehicleMode);

  if (packet.vehicleMode == FLIGHT_MODE) {
    // controlANGLE(&imuData, &position, &des, throttle, &pid);
    // controlMixer(&pid, throttle, &flyCmds);
    float control = packet.leftJoystickY < 512 ? 0 : ((packet.leftJoystickY - 512) / 512);
    flyCmds.frontLeft = control;
    flyCmds.frontRight = control;
    flyCmds.backLeft = control;
    flyCmds.backRight = control;
    sendFlightCommands(&flyCmds);
    sendDriveCommands(&nullDriveCmds);
  } else {
    tankDrive(packet.leftJoystickY, packet.rightJoystickY, &driveCmds);
    sendFlightCommands(&nullFlyCmds);
    sendDriveCommands(&driveCmds);
  }
  // limitLoopRate();
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


void decodeRadioPacket(RadioPacket *packet, float *throttle, RPYAngles *des) {
    *throttle = packet->leftJoystickY;
    des->roll = packet->leftJoystickX;
    des->pitch = packet->rightJoystickY;
    des->yaw = packet->rightJoystickX;
}

void transitionModeServos(vehicle_mode_t vehicleMode) {
  if (lastVehicleMode != vehicleMode) {
    setServos(vehicleMode);
    lastVehicleMode = vehicleMode;
  }
}

void setServos(vehicle_mode_t vehicleMode) {
  if (vehicleMode == FLIGHT_MODE) {
    transitionServos[0].write(95);
    transitionServos[1].write(85);
  } else {
    transitionServos[0].write(5);
    transitionServos[1].write(180);
  }
}

void sendFlightCommands(FlyCommands *cmds) {
  float cmd;
  for (uint32_t ch = 0; ch < NUM_FLY_MOTORS; ++ch) {
    cmd = *(&cmds->frontLeft + ch);
    analogWrite(FLY_MOTOR_PINS[ch], FLY_PWM_MIN + (cmd * FLY_PWM_RANGE));
  }
}

void sendDriveCommands(DriveCommands *cmds) {
  uint8_t cmd;
  for (uint32_t ch = 0; ch < NUM_DRIVE_MOTORS; ++ch) {
    cmd = *(&cmds->left + ch);
    analogWrite(DRIVE_MOTOR_PINS[ch], cmd);
  }
}