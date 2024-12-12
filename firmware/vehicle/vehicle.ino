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
IMUData imuError;
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
bool isEmergencyStopped;
unsigned long radioRxStartTime;

RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);
Servo transitionServos[NUM_TSERVO];
Servo flyMotorServos[NUM_FLY_MOTORS];

void setup() {
  Serial.begin(115200);

  initConstants();

  /* Initialize I2C */
  Wire1.begin();
  Wire1.setSCL(I2C_SCL);
  Wire1.setSDA(I2C_SDA);
  Wire1.setClock(1000000);

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
    flyMotorServos[ch].attach(FLY_MOTOR_PINS[ch]);
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
  calibrateAttitude();
  calibrateIMU(&imuError);
}

void calibrateAttitude() {
  uint32_t i;
  for (i = 0; i < NUM_IMU_CALIBRATION_ITER; ++i) {
    trackLoopTime();
    readIMU(&imuData);
    Madgwick6DOF(&imuData, &position);
    limitLoopRate();
  }
}

void loop() {
  trackLoopTime();

  readIMU(&imuData);
  subIMUData(&imuData, &imuError);
  // printIMU(&imuData);
  Madgwick6DOF(&imuData, &position);
  // Serial.print("PYAW ");
  // Serial.println(position.yaw);
  // Serial.print("PPITCH ");
  // Serial.println(position.pitch);
  // Serial.print("PROLL ");
  // Serial.println(position.roll);
  
  // readAllUltrasonic();

  // Serial.println(radio.isChipConnected());
  radioRxStartTime = millis();
  while (!radio.available()) {
    if ((millis() - radioRxStartTime) > RADIO_RX_TIMEOUT_MS) {
      isEmergencyStopped = true;
      break;
    }
  }
  radio.read(&packet, sizeof(packet));
  if (packet.buttonThree || isEmergencyStopped) {
    isEmergencyStopped = true;
    digitalWrite(LED_BLUE_B_PIN, HIGH);
    sendFlightCommands(&nullFlyCmds);
    sendDriveCommands(&nullDriveCmds);
    clearServos();
    return;
  } else {
    digitalWrite(LED_BLUE_B_PIN, LOW);
  }
  // printPacket(&packet);
  decodeRadioPacket(&packet, &throttle, &des);
  // Serial.print("YAW ");
  // Serial.println(des.yaw);
  // Serial.print("PITCH ");
  // Serial.println(des.pitch);
  // Serial.print("ROLL ");
  // Serial.println(des.roll);

  transitionModeServos(packet.vehicleMode);
  if (packet.vehicleMode == FLIGHT_MODE) {
    controlANGLE(&imuData, &position, &des, throttle, &pid);
    controlMixer(&pid, throttle, &flyCmds);
    // float control = packet.leftJoystickY < 512 ? 0 : ((packet.leftJoystickY - 512.0f) / 512.0f);
    // flyCmds.frontLeft = control;
    // flyCmds.frontRight = control;
    // flyCmds.backLeft = control;
    // flyCmds.backRight = control;
    // sendFlightCommands(&flyCmds);
    Serial.println("COMMANDS");
    Serial.print("FL: ");
    Serial.println(flyCmds.frontLeft);
    Serial.print("FR: ");
    Serial.println(flyCmds.frontRight);
    Serial.print("BL: ");
    Serial.println(flyCmds.backLeft);
    Serial.print("BR: ");
    Serial.println(flyCmds.backRight);
    sendDriveCommands(&nullDriveCmds);
  } else {
    tankDrive(packet.leftJoystickY, packet.rightJoystickY, &driveCmds);
    sendFlightCommands(&nullFlyCmds);
    sendDriveCommands(&driveCmds);
  }
  displayDebugIndicators();
  limitLoopRate();
}

void decodeRadioPacket(RadioPacket *packet, float *throttle, RPYAngles *des) {
  *throttle = zeroCenteredJoystick(packet->leftJoystickY, LEFT_JS_Y_CTR, true);
  des->roll = zeroCenteredJoystick(packet->leftJoystickX, LEFT_JS_X_CTR, false) * maxRoll;
  des->pitch = zeroCenteredJoystick(packet->rightJoystickY, RIGHT_JS_Y_CTR, false) * maxPitch;
  des->yaw = zeroCenteredJoystick(packet->rightJoystickX, RIGHT_JS_X_CTR, false) * maxYaw;
}

inline float zeroCenteredJoystick(uint16_t value, float center, bool strictlyPositive) {
  return (value < center && strictlyPositive) ? 0 : ((value - center) / center);
}

void readAllUltrasonic(void) {
  uint32_t ch;
  for (ch = 0; ch < NUM_US; ++ch) {
    ultrasonicInches[ch] = usToInches(readUltrasonicAveraged(ch));
  }
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

void clearServos(void) {
  uint32_t ch;
  for (ch = 0; ch < NUM_TSERVO; ++ch) {
    transitionServos[ch].detach();
  }
}

void sendFlightCommands(FlyCommands *cmds) {
  float cmd;
  const uint32_t range = MAX_MOTOR_US - MIN_MOTOR_US;
  for (uint32_t ch = 0; ch < NUM_FLY_MOTORS; ++ch) {
    cmd = *(&cmds->frontLeft + ch);
    flyMotorServos[ch].writeMicroseconds(MIN_MOTOR_US + (range * cmd));
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
}

float readBatteryVoltage(void) {
  uint32_t batReading = analogRead(V_BAT_PIN);
  return (batReading * VCC) / (ADC_MAX * V_BAT_DIV_RATIO) - V_BAT_OFFSET;
}