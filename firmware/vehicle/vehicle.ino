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
FlyCommands flyCmds;

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
  }

  packet.vehicleMode = DRIVE_MODE;
  lastVehicleMode = DRIVE_MODE;
}

void loop() {
  trackLoopTime();

  readIMU(&imuData);
  Madgwick6DOF(&imuData, &position);
  for (uint32_t ch = 0; ch < NUM_US; ++ch) {
    ultrasonicInches[ch] = usToInches(readUltrasonicAveraged(ch));
  }
  radio.read(&packet, sizeof(packet));
  decodeRadioPacket(&packet, &throttle, &des);
  transitionModeServos(packet.vehicleMode);

  if (packet.vehicleMode == FLIGHT_MODE) {
    controlANGLE(&imuData, &position, &des, throttle, &pid);
    controlMixer(&pid, throttle, &flyCmds);
  } else {
    tankDrive(packet.leftJoystickY, packet.rightJoystickY, &driveCmds);
  }
  limitLoopRate();
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
  const uint8_t setAngle = (vehicleMode == FLIGHT_MODE) ? 0 : 90;
  for (uint32_t ch = 0; ch < NUM_TSERVO; ++ch) {
    transitionServos[ch].write(setAngle);
  }
}

void sendFlightCommands(FlyCommands *cmds) {
  float cmd;
  for (uint32_t ch = 0; ch < NUM_FLY_MOTORS; ++ch) {
    cmd = *(&cmds->frontLeft + ch);
    analogWrite(FLY_MOTOR_PINS[ch], FLY_PWM_MIN + (floor(cmd) * FLY_PWM_RANGE));
  }
}

void sendDriveCommands(DriveCommands *cmds) {
  float cmd;
  for (uint32_t ch = 0; ch < NUM_DRIVE_MOTORS; ++ch) {
    cmd = *(&cmds->left + ch);
    analogWrite(DRIVE_MOTOR_PINS[ch], floor(cmd));
  }
}