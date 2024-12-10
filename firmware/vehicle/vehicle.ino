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
  Wire1.begin();
  Wire1.setSCL(I2C_SCL);
  Wire1.setSDA(I2C_SDA);

  /* Initialize IMU (over I2C) */
  initIMU(&Wire1);

  /* Initialize SPI */
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

  /* Debug / LED pins */
  pinMode(LED_BLUE_A_PIN, OUTPUT);
  pinMode(LED_BLUE_B_PIN, OUTPUT);
  pinMode(V_BAT_PIN, INPUT);

  setServos(FLIGHT_MODE);
  lastVehicleMode = FLIGHT_MODE;
}

void loop() {
  // trackLoopTime();

  readIMU(&imuData);
  printIMU(&imuData);
  // Madgwick6DOF(&imuData, &position);
  // for (uint32_t ch = 0; ch < NUM_US; ++ch) {
  //   ultrasonicInches[ch] = usToInches(readUltrasonicAveraged(ch));
  // }

  // Serial.println(radio.isChipConnected());
  // while (!radio.available()) {
  // }
  // radio.read(&packet, sizeof(packet));
  // printPacket(&packet);
  // decodeRadioPacket(&packet, &throttle, &des);

  transitionModeServos(packet.vehicleMode);
  if (packet.vehicleMode == FLIGHT_MODE) {
    // controlANGLE(&imuData, &position, &des, throttle, &pid);
    // controlMixer(&pid, throttle, &flyCmds);
    float control = packet.leftJoystickY < 512 ? 0 : ((packet.leftJoystickY - 512.0f) / 512.0f);
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
  displayDebugIndicators();
  // limitLoopRate();
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
    transitionServos[0].write(LEFT_SERVO_FLIGHT_DEG);
    transitionServos[1].write(RIGHT_SERVO_FLIGHT_DEG);
  } else {
    transitionServos[0].write(LEFT_SERVO_DRIVE_DEG);
    transitionServos[1].write(RIGHT_SERVO_DRIVE_DEG);
  }
}

void sendFlightCommands(FlyCommands *cmds) {
  float cmd;
  uint8_t range;
  for (uint32_t ch = 0; ch < NUM_FLY_MOTORS; ++ch) {
    cmd = *(&cmds->frontLeft + ch);
    range = FLY_PWM_MAXIMUMS[ch] - FLY_PWM_MINIMUMS[ch];
    analogWrite(FLY_MOTOR_PINS[ch], FLY_PWM_MINIMUMS[ch] + (cmd * range));
  }
}

void sendDriveCommands(DriveCommands *cmds) {
  uint8_t cmd;
  for (uint32_t ch = 0; ch < NUM_DRIVE_MOTORS; ++ch) {
    cmd = *(&cmds->left + ch);
    analogWrite(DRIVE_MOTOR_PINS[ch], cmd);
  }
}

void displayDebugIndicators(void) {
  bool lowVoltage = readBatteryVoltage() < V_BAT_WARN_THRESH;
  digitalWrite(LED_BLUE_A_PIN, lowVoltage);
  digitalWrite(LED_BLUE_B_PIN, lowVoltage);
}

float readBatteryVoltage(void) {
  uint32_t batReading = analogRead(V_BAT_PIN);
  return (batReading * VCC) / (ADC_MAX * V_BAT_DIV_RATIO) - V_BAT_OFFSET;
}