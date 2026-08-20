#include "CommandParser.h"

#include <stdlib.h>
#include <string.h>

#include "MiniRobotMotors.h"

#include "AccelCal.h"
#include "Config.h"
#include "GyroCal.h"
#include "ImuDriver.h"
#include "Interactive.h"
#include "MagCal.h"
#include "MotorNoise.h"
#include "Results.h"

static char commandBuffer[32];
static uint8_t commandLength = 0;

// The guided run. Each stage prompts for what it needs, so the whole thing is
// one command and a few minutes of moving the robot around. A stage that fails
// stops the run: every later stage assumes the earlier ones measured something.
static void runFullCalibration() {
  Serial.println(F("INFO,FULL_CALIBRATION,FOLLOW_THE_ASK_LINES"));
  Serial.println(F("INFO,SEND_X_AT_ANY_PROMPT_TO_STOP"));

  runHealthCheck();

  if (!calibrateGyroBias()) {
    Serial.println(F("WARN,CAL_STOPPED_AT_GYRO"));
    return;
  }
  if (!calibrateAccelAllFaces()) {
    Serial.println(F("WARN,CAL_STOPPED_AT_ACCEL"));
    return;
  }

  // Not a prerequisite for anything else, so a failed yaw check does not stop
  // the run -- it just leaves IMU_YAW_SIGN out of the paste block.
  runYawSignCheck();

  if (magPresent) {
    calibrateMagnetometer(magDurationMs);
  } else {
    Serial.println(F("INFO,NO_MAGNETOMETER,SKIPPING_MAG"));
  }

  printResultTable();
  printPasteBlock();
  Serial.println(F("ACK,CAL_DONE"));
}

static void processCommand(char *line) {
  char *savePtr = NULL;
  char *command = strtok_r(line, ",", &savePtr);
  if (command == NULL) return;

  for (char *p = command; *p; ++p) {
    if (*p >= 'a' && *p <= 'z') *p = *p - ('a' - 'A');
  }

  if (strcmp(command, "CAL") == 0) {
    runFullCalibration();
  }
  else if (strcmp(command, "CHECK") == 0) {
    runHealthCheck();
  }
  else if (strcmp(command, "GYRO") == 0) {
    calibrateGyroBias();
  }
  else if (strcmp(command, "ACCEL") == 0) {
    calibrateAccelAllFaces();
  }
  else if (strcmp(command, "FACE") == 0) {
    if (measureAccelFace()) printMissingFaces();
  }
  else if (strcmp(command, "MAG") == 0) {
    char *secondsText = strtok_r(NULL, ",", &savePtr);
    uint32_t duration = magDurationMs;
    if (secondsText) {
      const int seconds = atoi(secondsText);
      if (seconds < 10 || seconds > 120) {
        Serial.println(F("ERR,MAG_SECONDS_10_120"));
        return;
      }
      duration = (uint32_t)seconds * 1000UL;
    }
    calibrateMagnetometer(duration);
  }
  else if (strcmp(command, "MOTORS") == 0) {
    runMotorInterferenceTest();
  }
  else if (strcmp(command, "YAWSIGN") == 0) {
    runYawSignCheck();
  }
  else if (strcmp(command, "LIVE") == 0) {
    char *secondsText = strtok_r(NULL, ",", &savePtr);
    const int seconds = secondsText ? atoi(secondsText) : 30;
    streamLive((uint32_t)constrain(seconds, 1, 300) * 1000UL);
  }
  else if (strcmp(command, "RAW") == 0) {
    char *secondsText = strtok_r(NULL, ",", &savePtr);
    const int seconds = secondsText ? atoi(secondsText) : 30;
    streamRaw((uint32_t)constrain(seconds, 1, 300) * 1000UL);
  }
  else if (strcmp(command, "RESULT") == 0) {
    printResultTable();
    printPasteBlock();
  }
  else if (strcmp(command, "SAMPLES") == 0) {
    char *gyroText = strtok_r(NULL, ",", &savePtr);
    char *accelText = strtok_r(NULL, ",", &savePtr);
    const int gyroCount = gyroText ? atoi(gyroText) : -1;
    const int accelCount = accelText ? atoi(accelText) : -1;
    // The gyro run is halved, so an odd or tiny count would make the drift
    // number meaningless.
    if (gyroCount < 200 || gyroCount > 6000 || accelCount < 50 || accelCount > 2000) {
      Serial.println(F("ERR,SAMPLES_GYRO_200_6000_ACCEL_50_2000"));
      return;
    }
    gyroSampleCount = (uint16_t)gyroCount;
    accelSampleCount = (uint16_t)accelCount;
    printConfig();
  }
  else if (strcmp(command, "MAGTIME") == 0) {
    char *secondsText = strtok_r(NULL, ",", &savePtr);
    const int seconds = secondsText ? atoi(secondsText) : -1;
    if (seconds < 10 || seconds > 120) {
      Serial.println(F("ERR,MAGTIME_SECONDS_10_120"));
      return;
    }
    magDurationMs = (uint32_t)seconds * 1000UL;
    printConfig();
  }
  else if (strcmp(command, "CONFIG") == 0) {
    printConfig();
  }
  else if (strcmp(command, "STOP") == 0) {
    motorsCoastAll();
    Serial.println(F("ACK,STOP"));
  }
  else if (strcmp(command, "HELP") == 0 || strcmp(command, "?") == 0) {
    printHelp();
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
