#include "CommandParser.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "Config.h"
#include "Imu.h"
#include "MotorControl.h"
#include "Sweep.h"

static char commandBuffer[COMMAND_BUFFER_SIZE];
static uint8_t commandLength = 0;

// Report the constants this binary was compiled with.
//
// Nothing else on the wire can tell a host whether the board is running the
// sketch that is currently on disk. Telemetry looks identical either way, and
// this robot has now lost time twice to a source tree that had been fixed and a
// board that had not been reflashed -- the second time a stale ENCODER_SIGN
// pinned a wheel at full duty while the fixed source said it could not happen.
// A host that reads this line can refuse to give advice computed from constants
// the robot is not actually using.
//
// The signs are int8_t, which is 'signed char': printing one directly emits a
// character, not a number, so they are cast to int.
static void printBuildConfig() {
  Serial.print(F("INFO,CFG,counts="));
  Serial.print(MEASURED_COUNTS_PER_REV[LEFT], 1);
  Serial.print('/');
  Serial.print(MEASURED_COUNTS_PER_REV[RIGHT], 1);
  Serial.print(F(",dia="));
  Serial.print(WHEEL_DIAMETER_MM, 2);
  Serial.print(F(",track="));
  Serial.print(TRACK_WIDTH_MM, 1);
  Serial.print(F(",maxlin="));
  Serial.print(MAX_LINEAR_MM_S, 1);
  Serial.print(F(",maxang="));
  Serial.print(MAX_ANGULAR_DEG_S, 1);
  Serial.print(F(",maxwheel="));
  Serial.print(MAX_WHEEL_MM_S, 1);
  Serial.print(F(",maxpwm="));
  Serial.print((int)MAX_DRIVE_PWM);
  Serial.print(F(",enc="));
  Serial.print((int)ENCODER_SIGN[LEFT]);
  Serial.print('/');
  Serial.print((int)ENCODER_SIGN[RIGHT]);
  Serial.print(F(",mot="));
  Serial.print((int)MOTOR_DIRECTION_SIGN[LEFT]);
  Serial.print('/');
  Serial.println((int)MOTOR_DIRECTION_SIGN[RIGHT]);
}

// Parse and execute one complete command line (see the top-of-sketch comment
// for the command set). Tokenizes in place with strtok_r and replies with
// ACK/ERR.
static void processCommand(char *line) {
  char *savePtr = NULL;
  char *command = strtok_r(line, ",", &savePtr);
  if (command == NULL) return;

  // Make command comparison case insensitive.
  for (char *p = command; *p; ++p) {
    if (*p >= 'a' && *p <= 'z') *p = *p - ('a' - 'A');
  }

  if (strcmp(command, "VEL") == 0) {
    char *linearText = strtok_r(NULL, ",", &savePtr);
    char *angularText = strtok_r(NULL, ",", &savePtr);
    if (linearText && angularText && !emergencyStopLatched) {
      setVelocityCommand(atof(linearText), atof(angularText));
      Serial.println(F("ACK,VEL"));
    } else if (emergencyStopLatched) {
      Serial.println(F("ERR,ESTOP_LATCHED"));
    } else {
      Serial.println(F("ERR,VEL_FORMAT"));
    }
  }
  else if (strcmp(command, "FORWARD") == 0) {
    char *value = strtok_r(NULL, ",", &savePtr);
    if (!emergencyStopLatched) setVelocityCommand(value ? fabs(atof(value)) : DEFAULT_TEST_LINEAR_MM_S, 0.0f);
  }
  else if (strcmp(command, "REVERSE") == 0) {
    char *value = strtok_r(NULL, ",", &savePtr);
    if (!emergencyStopLatched) setVelocityCommand(-(value ? fabs(atof(value)) : DEFAULT_TEST_LINEAR_MM_S), 0.0f);
  }
  else if (strcmp(command, "LEFT") == 0) {
    char *value = strtok_r(NULL, ",", &savePtr);
    if (!emergencyStopLatched) setVelocityCommand(0.0f, value ? fabs(atof(value)) : DEFAULT_TEST_ANGULAR_DEG_S);
  }
  else if (strcmp(command, "RIGHT") == 0) {
    char *value = strtok_r(NULL, ",", &savePtr);
    if (!emergencyStopLatched) setVelocityCommand(0.0f, -(value ? fabs(atof(value)) : DEFAULT_TEST_ANGULAR_DEG_S));
  }
  else if (strcmp(command, "STOP") == 0) {
    stopCommand(false);
    refreshCommandWatchdog();
    Serial.println(F("ACK,STOP"));
  }
  else if (strcmp(command, "ESTOP") == 0) {
    emergencyStopLatched = true;
    stopCommand(true);
    Serial.println(F("ACK,ESTOP_LATCHED"));
  }
  else if (strcmp(command, "CLEAR_ESTOP") == 0) {
    emergencyStopLatched = false;
    stopCommand(false);
    refreshCommandWatchdog();
    Serial.println(F("ACK,ESTOP_CLEARED"));
  }
  else if (strcmp(command, "RESET_ODOM") == 0) {
    stopCommand(false);
    resetOdometry();
    Serial.println(F("ACK,ODOM_RESET"));
  }
  else if (strcmp(command, "ZERO_YAW") == 0) {
    zeroYaw();
    Serial.println(F("ACK,YAW_ZEROED"));
  }
  else if (strcmp(command, "CAL_IMU") == 0) {
    stopCommand(true);
    Serial.println(F("INFO,KEEP_ROBOT_STILL_CALIBRATING_IMU"));
    calibrateGyro();
    reinitFusion();
    zeroYaw();
    Serial.println(F("ACK,IMU_CALIBRATED"));
  }
  else if (strcmp(command, "MAG") == 0) {
    char *value = strtok_r(NULL, ",", &savePtr);
    const bool wanted = value && atoi(value) != 0;
    if (wanted && !magnetometerPresent) {
      Serial.println(F("ERR,NO_MAGNETOMETER"));
    } else {
      useMagnetometer = wanted;
      // Yaw meaning changes (absolute vs relative), so rebuild the filter.
      reinitFusion();
      zeroYaw();
      printImuInfo();
      Serial.println(F("ACK,MAG"));
    }
  }
  else if (strcmp(command, "IMU") == 0) {
    printImuInfo();
  }
  else if (strcmp(command, "CFG") == 0) {
    printBuildConfig();
  }
  else if (strcmp(command, "PID") == 0) {
    char *pText = strtok_r(NULL, ",", &savePtr);
    char *iText = strtok_r(NULL, ",", &savePtr);
    char *dText = strtok_r(NULL, ",", &savePtr);
    char *lamText = strtok_r(NULL, ",", &savePtr);
    char *muText = strtok_r(NULL, ",", &savePtr);
    if (pText && iText && dText) {
      const float kp = atof(pText);
      const float ki = atof(iText);
      const float kd = atof(dText);
      const float lambda = lamText ? atof(lamText) : 1.0f;
      const float mu = muText ? atof(muText) : 1.0f;
      for (uint8_t i = 0; i < MOTOR_COUNT; ++i) wheelPID[i].configure(kp, ki, kd, lambda, mu);
      Serial.println(F("ACK,PID_ALL"));
    } else {
      Serial.println(F("ERR,PID_FORMAT"));
    }
  }
  else if (strcmp(command, "PIDM") == 0) {
    char *motorText = strtok_r(NULL, ",", &savePtr);
    char *pText = strtok_r(NULL, ",", &savePtr);
    char *iText = strtok_r(NULL, ",", &savePtr);
    char *dText = strtok_r(NULL, ",", &savePtr);
    char *lamText = strtok_r(NULL, ",", &savePtr);
    char *muText = strtok_r(NULL, ",", &savePtr);
    const int index = motorText ? atoi(motorText) - 1 : -1;
    if (index >= 0 && index < MOTOR_COUNT && pText && iText && dText) {
      const float lambda = lamText ? atof(lamText) : 1.0f;
      const float mu = muText ? atof(muText) : 1.0f;
      wheelPID[index].configure(atof(pText), atof(iText), atof(dText), lambda, mu);
      Serial.println(F("ACK,PID_MOTOR"));
    } else {
      Serial.println(F("ERR,PIDM_FORMAT"));
    }
  }
  else if (strcmp(command, "SWEEP") == 0) {
    char *motorText = strtok_r(NULL, ",", &savePtr);
    char *stepText = strtok_r(NULL, ",", &savePtr);
    char *dwellText = strtok_r(NULL, ",", &savePtr);
    char *dirText = strtok_r(NULL, ",", &savePtr);
    const int index = motorText ? atoi(motorText) - 1 : -1;
    const int step = stepText ? atoi(stepText) : SWEEP_DEFAULT_STEP;
    const int dwell = dwellText ? atoi(dwellText) : SWEEP_DEFAULT_DWELL_MS;
    // step is bounded above as well as below: atoi() yields an int, and a
    // value like 256 truncates to 0 in the uint8_t parameter, which makes the
    // sweep loop never advance -- an infinite loop with the wheel energised.
    if (index >= 0 && index < MOTOR_COUNT && step > 0 && step <= SWEEP_MAX_STEP &&
        dwell >= SWEEP_MIN_DWELL_MS) {
      runSweep((uint8_t)index, (uint8_t)step, (uint16_t)dwell,
               dirText && (dirText[0] == 'R' || dirText[0] == 'r'));
    } else {
      Serial.println(F("ERR,SWEEP_FORMAT,MOTOR_1_2_STEP_1_50_DWELL_MIN_100"));
    }
  }
  else if (strcmp(command, "PING") == 0) {
    Serial.println(F("PONG"));
  }
  else {
    Serial.print(F("ERR,UNKNOWN_COMMAND,"));
    Serial.println(command);
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r' || c == ':') {
      if (commandLength > 0) {
        commandBuffer[commandLength] = '\0';
        processCommand(commandBuffer);
        commandLength = 0;
      }
    } else if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = c;
    } else {
      commandLength = 0;
      Serial.println(F("ERR,COMMAND_TOO_LONG"));
    }
  }
}

void printReady() {
  Serial.println(F("READY,mini_ROS2_low_level_v1"));
  // Emitted unprompted at boot so a host that is only listening still learns
  // which constants this board is running.
  printBuildConfig();
  printImuInfo();
}
