#include "CommandParser.h"

#include <stdlib.h>
#include <string.h>

#include <MiniRobotMotors.h>

#include "Config.h"
#include "FitResults.h"
#include "Interactive.h"
#include "Sweep.h"
#include "Verify.h"

static char commandBuffer[48];
static uint8_t commandLength = 0;

static void processCommand(char *line) {
  char *savePtr = NULL;
  char *command = strtok_r(line, ",", &savePtr);
  if (command == NULL) return;

  for (char *p = command; *p; ++p) {
    if (*p >= 'a' && *p <= 'z') *p = *p - ('a' - 'A');
  }

  if (strcmp(command, "CAL") == 0) {
    char *motorText = strtok_r(NULL, ",", &savePtr);
    if (motorText) {
      const int index = atoi(motorText) - 1;
      if (index < 0 || index >= MOTOR_COUNT) {
        Serial.println(F("ERR,CAL_MOTOR_RANGE"));
        return;
      }
      calibrateMotor((uint8_t)index);
    } else {
      for (uint8_t i = 0; i < MOTOR_COUNT; ++i) calibrateMotor(i);
    }
    printResultTable();
    Serial.println(F("ACK,CAL_DONE"));
  }
  else if (strcmp(command, "SWEEP") == 0) {
    char *motorText = strtok_r(NULL, ",", &savePtr);
    char *dirText = strtok_r(NULL, ",", &savePtr);
    const int index = motorText ? atoi(motorText) - 1 : -1;
    if (index < 0 || index >= MOTOR_COUNT) {
      Serial.println(F("ERR,SWEEP_FORMAT"));
      return;
    }
    sweepAndFit((uint8_t)index, dirText && (dirText[0] == 'R' || dirText[0] == 'r'));
    Serial.println(F("ACK,SWEEP_DONE"));
  }
  else if (strcmp(command, "VERIFY") == 0) {
    char *motorText = strtok_r(NULL, ",", &savePtr);
    if (motorText) {
      const int index = atoi(motorText) - 1;
      if (index < 0 || index >= MOTOR_COUNT) {
        Serial.println(F("ERR,VERIFY_MOTOR_RANGE"));
        return;
      }
      verifyMotor((uint8_t)index);
    } else {
      for (uint8_t i = 0; i < MOTOR_COUNT; ++i) verifyMotor(i);
    }
    Serial.println(F("ACK,VERIFY_DONE"));
  }
  else if (strcmp(command, "RESULT") == 0) {
    printResultTable();
  }
  else if (strcmp(command, "HOLD") == 0) {
    char *motorText = strtok_r(NULL, ",", &savePtr);
    char *pwmText = strtok_r(NULL, ",", &savePtr);
    char *dirText = strtok_r(NULL, ",", &savePtr);
    const int index = motorText ? atoi(motorText) - 1 : -1;
    const int pwm = pwmText ? atoi(pwmText) : -1;
    if (index < 0 || index >= MOTOR_COUNT || pwm < 0 || pwm > MAX_DRIVE_PWM) {
      Serial.println(F("ERR,HOLD_FORMAT"));
      return;
    }
    holdAndStream((uint8_t)index, (uint16_t)pwm,
                  dirText && (dirText[0] == 'R' || dirText[0] == 'r'), HOLD_MAX_MS);
  }
  else if (strcmp(command, "TICKS") == 0) {
    char *secondsText = strtok_r(NULL, ",", &savePtr);
    const int seconds = secondsText ? atoi(secondsText) : 30;
    streamTicks((uint32_t)constrain(seconds, 1, 300) * 1000UL);
  }
  else if (strcmp(command, "STEP") == 0) {
    char *stepText = strtok_r(NULL, ",", &savePtr);
    const int step = stepText ? atoi(stepText) : 0;
    if (step < 1 || step > 50) {
      Serial.println(F("ERR,STEP_RANGE_1_50"));
      return;
    }
    sweepStepPWM = (uint16_t)step;
    printConfig();
  }
  else if (strcmp(command, "RANGE") == 0) {
    char *minText = strtok_r(NULL, ",", &savePtr);
    char *maxText = strtok_r(NULL, ",", &savePtr);
    const int lo = minText ? atoi(minText) : -1;
    const int hi = maxText ? atoi(maxText) : -1;
    // At least 30 PWM of span, or there is nothing to fit a line through.
    if (lo < 0 || hi > MAX_DRIVE_PWM || hi - lo < 30) {
      Serial.print(F("ERR,RANGE_MUST_BE_0.."));
      Serial.println(MAX_DRIVE_PWM);
      return;
    }
    sweepMinPWM = (uint16_t)lo;
    sweepMaxPWM = (uint16_t)hi;
    printConfig();
  }
  else if (strcmp(command, "DWELL") == 0) {
    char *settleText = strtok_r(NULL, ",", &savePtr);
    char *windowText = strtok_r(NULL, ",", &savePtr);
    const int settle = settleText ? atoi(settleText) : -1;
    const int window = windowText ? atoi(windowText) : -1;
    // A window shorter than ~200 ms quantises badly: at low PWM only a handful
    // of ticks land inside it, and the fit inherits that noise.
    if (settle < 50 || settle > 5000 || window < 200 || window > 5000) {
      Serial.println(F("ERR,DWELL_SETTLE_50_5000_WINDOW_200_5000"));
      return;
    }
    settleMs = (uint16_t)settle;
    windowMs = (uint16_t)window;
    printConfig();
  }
  else if (strcmp(command, "PASSES") == 0) {
    char *passText = strtok_r(NULL, ",", &savePtr);
    const int passes = passText ? atoi(passText) : 0;
    if (passes < 1 || passes > 4) {
      Serial.println(F("ERR,PASSES_RANGE_1_4"));
      return;
    }
    sweepPasses = (uint8_t)passes;
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
