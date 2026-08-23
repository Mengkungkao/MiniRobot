#include "Commands.h"

#include <stdlib.h>
#include <string.h>

#include "MiniRobotMotors.h"

#include "Config.h"
#include "Direction.h"
#include "Fit.h"
#include "Measure.h"
#include "Sweep.h"

void printConfig() {
  // Geometry and signs first, because they decide what every number below
  // MEANS, and a bench sketch flashed from a stale checkout is otherwise
  // indistinguishable from a current one.
  //
  // This robot lost a calibration run to exactly that. The firmware carried the
  // corrected ENCODER_SIGN for the right wheel and this sketch, flashed
  // earlier, did not -- so a sweep labelled FORWARD drove the wheel one way
  // here and the other way there, and every motor 2 point came back rejected
  // with no hint that the two binaries disagreed. Printing them at boot costs
  // a few hundred bytes of a sketch that is half empty and turns a silent
  // wrong answer into a visible one.
  Serial.print(F("INFO,GEOMETRY,counts="));
  Serial.print(MEASURED_COUNTS_PER_REV[LEFT], 1);
  Serial.print('/');
  Serial.print(MEASURED_COUNTS_PER_REV[RIGHT], 1);
  Serial.print(F(",dia="));
  Serial.print(WHEEL_DIAMETER_MM, 2);
  Serial.print(F(",maxpwm="));
  Serial.println((int)MAX_DRIVE_PWM);
  printSigns();
  Serial.println(F("INFO,COMPARE_WITH,CFG,ON_arduino_mini_base_controller,THEY_MUST_AGREE"));

  Serial.print(F("INFO,CONFIG,range="));
  Serial.print(sweepMinPWM);
  Serial.print(F(".."));
  Serial.print(sweepMaxPWM);
  Serial.print(F(",step="));
  Serial.print(sweepStepPWM);
  Serial.print(F(",settle_ms="));
  Serial.print(settleMs);
  Serial.print(F(",window_ms="));
  Serial.print(windowMs);
  Serial.print(F(",passes="));
  Serial.println(sweepPasses);

  const uint32_t perDirectionS =
    ((uint32_t)sweepStepCount() * sweepPasses * (uint32_t)(settleMs + windowMs)) / 1000UL;
  Serial.print(F("INFO,ESTIMATED_SECONDS,per_direction="));
  Serial.print(perDirectionS);
  Serial.print(F(",full_CAL="));
  Serial.println(perDirectionS * 4);
}

void printHelp() {
  Serial.println(F("CMD,DIR[,motor]            commission BOTH direction signs from one look"));
  Serial.println(F("CMD,CHECK                  drive each wheel forward, check the encoder agrees"));
  Serial.println(F("CMD,SIGN                   print the commissioning signs"));
  Serial.println(F("CMD,CAL[,motor]            calibrate (default: both motors, both directions)"));
  Serial.println(F("CMD,SWEEP,motor[,R]        one motor, one direction"));
  Serial.println(F("CMD,VERIFY[,motor]         open-loop check of the fitted constants"));
  Serial.println(F("CMD,RESULT                 reprint fits and the paste block"));
  Serial.println(F("CMD,HOLD,motor,pwm[,R]     hold one PWM, stream speed"));
  Serial.println(F("CMD,TICKS[,seconds]        stream raw encoder counts"));
  Serial.println(F("CMD,STEP,pwm_step          set sweep step"));
  Serial.println(F("CMD,RANGE,min_pwm,max_pwm  set sweep range"));
  Serial.println(F("CMD,DWELL,settle_ms,win_ms set per-step timing"));
  Serial.println(F("CMD,PASSES,1..4            set passes per direction"));
  Serial.println(F("CMD,CONFIG                 print settings and run-time estimate"));
  Serial.println(F("CMD,STOP                   coast both motors"));
  Serial.println(F("INFO,ANY_CHARACTER_ABORTS_A_RUNNING_SWEEP"));
}

static char commandBuffer[48];
static uint8_t commandLength = 0;

// Parse an optional trailing motor number. Returns the 0-based index, -1 for
// "not given" (meaning both), or -2 for out of range.
static int8_t parseOptionalMotor(char *text) {
  if (text == NULL) return -1;
  const int index = atoi(text) - 1;
  if (index < 0 || index >= MOTOR_COUNT) return -2;
  return (int8_t)index;
}

static bool isReverseFlag(const char *text) {
  return text != NULL && (text[0] == 'R' || text[0] == 'r');
}

static void processCommand(char *line) {
  char *savePtr = NULL;
  char *command = strtok_r(line, ",", &savePtr);
  if (command == NULL) return;

  for (char *p = command; *p; ++p) {
    if (*p >= 'a' && *p <= 'z') *p = *p - ('a' - 'A');
  }

  if (strcmp(command, "DIR") == 0) {
    const int8_t index = parseOptionalMotor(strtok_r(NULL, ",", &savePtr));
    if (index == -2) {
      Serial.println(F("ERR,DIR_MOTOR_RANGE"));
      return;
    }
    commissionDirections(index);
  }
  else if (strcmp(command, "CHECK") == 0) {
    runDirectionCheck();
  }
  else if (strcmp(command, "SIGN") == 0) {
    printSigns();
  }
  else if (strcmp(command, "CAL") == 0) {
    const int8_t index = parseOptionalMotor(strtok_r(NULL, ",", &savePtr));
    if (index == -2) {
      Serial.println(F("ERR,CAL_MOTOR_RANGE"));
      return;
    }
    if (index >= 0) {
      calibrateMotor((uint8_t)index);
    } else {
      for (uint8_t i = 0; i < MOTOR_COUNT; ++i) calibrateMotor(i);
    }
    printResultTable();
    Serial.println(F("ACK,CAL_DONE"));
  }
  else if (strcmp(command, "SWEEP") == 0) {
    const int8_t index = parseOptionalMotor(strtok_r(NULL, ",", &savePtr));
    char *dirText = strtok_r(NULL, ",", &savePtr);
    if (index < 0) {
      Serial.println(F("ERR,SWEEP_FORMAT,SWEEP,<motor>[,R]"));
      return;
    }
    sweepAndFit((uint8_t)index, isReverseFlag(dirText));
    Serial.println(F("ACK,SWEEP_DONE"));
  }
  else if (strcmp(command, "VERIFY") == 0) {
    const int8_t index = parseOptionalMotor(strtok_r(NULL, ",", &savePtr));
    if (index == -2) {
      Serial.println(F("ERR,VERIFY_MOTOR_RANGE"));
      return;
    }
    if (index >= 0) {
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
    const int8_t index = parseOptionalMotor(strtok_r(NULL, ",", &savePtr));
    char *pwmText = strtok_r(NULL, ",", &savePtr);
    char *dirText = strtok_r(NULL, ",", &savePtr);
    const int pwm = pwmText ? atoi(pwmText) : -1;
    if (index < 0 || pwm < 0 || pwm > MAX_DRIVE_PWM) {
      Serial.println(F("ERR,HOLD_FORMAT,HOLD,<motor>,<pwm>[,R]"));
      return;
    }
    holdAndStream((uint8_t)index, (uint16_t)pwm, isReverseFlag(dirText), HOLD_MAX_MS);
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
