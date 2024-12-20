#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

#include "sensors.h"
#include "attitude_estimator.h"

extern "C" {
  #include "util.h"
  #include "flightController.h"
  #include "driveController.h"
}

IMUData imuData;
IMUData imuError;
float ultrasonicCentimeters[NUM_US];
// AttitudeEstimator position;
RPYAngles position;
RPYAngles pid;
DriveCommands driveCmds;
DriveCommands nullDriveCmds = {};
FlyCommands flyCmds;
FlyCommands nullFlyCmds = {};

RadioPacket packet;
float throttle;
RPYAngles des;
vehicle_mode_t lastVehicleMode;
bool isEmergencyStopped;
unsigned long radioRxStartTime;
uint32_t autoFlightStartTimeMs;

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
  radio.setChannel(RADIO_CH);
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

  currTime = micros();
  trackLoopTime();

  Serial.println("START CALIBRATION");
  // calibrateAttitude();
  // position.setMagCalib(0, 0, 0);
  calibrateIMU(&imuError);
  Serial.println("END CALIBRATION");
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

uint32_t loopCtr = 0;

void loop() {
  trackLoopTime();

  readIMU(&imuData);
  subIMUData(&imuData, &imuError);
  // printIMU(&imuData);
  // Madgwick6DOF(&imuData, &position);
  // position.update(dt, imuData.accX, imuData.accY, imuData.accZ, deg2rad(imuData.gyrX), deg2rad(imuData.gyrY), deg2rad(imuData.gyrZ), 0, 0, 0);
  // if (loopCtr % 100 == 0) {
    // Serial.print("PYAW ");
    // Serial.println(position.yaw);
    // Serial.print("PPITCH ");
    // Serial.println(position.pitch);
    // Serial.print("PROLL ");
    // Serial.println(position.roll);
  //   Serial.print("LOOP TIME ");
  //   Serial.println(dt*1e6);
  // }
  // loopCtr++;

  // Serial.print("PYAW ");
  // Serial.println(position.eulerYaw());
  // Serial.print("PPITCH ");
  // Serial.println(position.eulerPitch());
  // Serial.print("PROLL ");
  // Serial.println(position.eulerRoll());

  // Serial.println(radio.isChipConnected());
  // ultrasonicCentimeters[US_FRONT_IDX] = usToCentimeters(readUltrasonicAveraged(US_FRONT_IDX));
  // Serial.println(ultrasonicCentimeters[US_FRONT_IDX]);
  
  displayDebugIndicators();
  radioRxStartTime = millis();
  while ((!radio.available() || !radio.isChipConnected()) && !isEmergencyStopped) {
    if ((millis() - radioRxStartTime) > RADIO_RX_TIMEOUT_MS) {
      isEmergencyStopped = true;
      break;
    }
  }
  radio.read(&packet, sizeof(packet));
  // printPacket(&packet);
  // Serial.println(usToCentimeters(readUltrasonicAveraged(US_FRONT_IDX)));
  if (packet.buttonThree || isEmergencyStopped) {
    isEmergencyStopped = true;
    digitalWrite(LED_BLUE_B_PIN, HIGH);
    clearFlightMotors();
    sendDriveCommands(&nullDriveCmds);
    clearServos();
    return;
  } else {
    digitalWrite(LED_BLUE_B_PIN, LOW);
  }
  
  decodeRadioPacket(&packet, &throttle, &des);
  // Serial.print("YAW ");
  // Serial.println(des.yaw);
  // Serial.print("PITCH ");
  // Serial.println(des.pitch);
  // Serial.print("ROLL ");
  // Serial.println(des.roll);

  if (packet.isAutoMode) {
    autoMode();
  } else {
    teleopMode();
  }
  limitLoopRate();
}

void autoMode(void) {
  // readAllUltrasonic();
  ultrasonicCentimeters[US_FRONT_IDX] = usToCentimeters(readUltrasonicAveraged(US_FRONT_IDX));
  if (lastVehicleMode == FLIGHT_MODE) {
    uint32_t autoTimeElapsed = millis() - autoFlightStartTimeMs;
    if (autoTimeElapsed < AUTO_FLIGHT_TIMEOUT_MS) {
      transitionModeServos(FLIGHT_MODE, true);
      flyCmds.backLeft = 0.2f;
      flyCmds.backRight = 0.2f;
      flyCmds.frontLeft = 0.3f;
      flyCmds.frontRight = 0.3f;
      sendFlightCommands(&flyCmds);
      sendDriveCommands(&nullDriveCmds);
    } else {
      clearFlightMotors();
      delay(AUTO_TRANSITION_DELAY_MS);
      transitionModeServos(DRIVE_MODE, true);
    }
    sendDriveCommands(&nullDriveCmds);
  } else if (lastVehicleMode == DRIVE_MODE) {
    transitionModeServos(DRIVE_MODE, true);
    if (ultrasonicCentimeters[US_FRONT_IDX] >= FRONT_OBSTACLE_THRESH_CM) {
      driveCmds.left = 255;
      driveCmds.right = 255;
      sendDriveCommands(&driveCmds);
      clearFlightMotors();
    } else {
      sendDriveCommands(&nullDriveCmds);
      transitionModeServos(FLIGHT_MODE, true);
      delay(AUTO_TRANSITION_DELAY_MS);
      autoFlightStartTimeMs = millis();
    }
  }
}

void teleopMode(void) {
  transitionModeServos(packet.vehicleMode, false);
  if (packet.vehicleMode == FLIGHT_MODE) {
    controlRATE(&imuData, &des, throttle, &pid);
    controlMixer(&pid, throttle, &flyCmds);
    constrainFlightCommands(&flyCmds);
    // float control = packet.leftJoystickY < 512 ? 0 : ((packet.leftJoystickY - 512.0f) / 512.0f);
    // flyCmds.frontLeft = control;
    // flyCmds.frontRight = control;
    // flyCmds.backLeft = control;
    // flyCmds.backRight = control;
    sendFlightCommands(&flyCmds);
    // delayMicroseconds(100000);
      // Serial.println("COMMANDS");
      // Serial.print("FL: ");
      // Serial.println(flyCmds.frontLeft);
      // Serial.print("FR: ");
      // Serial.println(flyCmds.frontRight);
      // Serial.print("BL: ");
      // Serial.println(flyCmds.backLeft);
      // Serial.print("BR: ");
      // Serial.println(flyCmds.backRight);
      // sendDriveCommands(&nullDriveCmds);
    // delayMicroseconds(100000);
  } else {
    tankDrive(packet.leftJoystickY, packet.rightJoystickY, &driveCmds);
    clearFlightMotors();
    sendDriveCommands(&driveCmds);
  }
}

void decodeRadioPacket(RadioPacket *packet, float *throttle, RPYAngles *des) {
  *throttle = zeroCenteredJoystick(packet->leftJoystickY, LEFT_JS_Y_CTR, true);
  des->yaw = zeroCenteredJoystick(packet->leftJoystickX, LEFT_JS_X_CTR, false) * maxYaw;
  des->pitch = zeroCenteredJoystick(packet->rightJoystickY, RIGHT_JS_Y_CTR, false) * maxPitch;
  des->roll = zeroCenteredJoystick(packet->rightJoystickX, RIGHT_JS_X_CTR, false) * maxRoll;
}

inline float zeroCenteredJoystick(uint16_t value, float center, bool strictlyPositive) {
  return (value < center && strictlyPositive) ? 0 : ((value - center) / center);
}

void readAllUltrasonic(void) {
  uint32_t ch;
  for (ch = 0; ch < NUM_US; ++ch) {
    ultrasonicCentimeters[ch] = usToCentimeters(readUltrasonicAveraged(ch));
  }
}

void transitionModeServos(vehicle_mode_t vehicleMode, bool override) {
  if (override || lastVehicleMode != vehicleMode) {
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

void constrainFlightCommands(FlyCommands *cmds) {
 float maxCmd = max(max(max(cmds->frontLeft, cmds->frontRight), cmds->backLeft), cmds->backRight);
 if (maxCmd > 1.0f) {
  cmds->frontLeft /= maxCmd;
  cmds->frontRight /= maxCmd;
  cmds->backLeft /= maxCmd;
  cmds->backRight /= maxCmd;
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

void clearFlightMotors(void) {
  for (uint32_t ch = 0; ch < NUM_FLY_MOTORS; ++ch) {
    flyMotorServos[ch].writeMicroseconds(0);
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