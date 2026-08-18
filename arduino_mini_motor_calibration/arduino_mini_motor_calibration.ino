/*
  mini ROS2 base -- motor feed-forward calibration bench sketch

  WHAT THIS IS FOR
  ------------------------------------------------------------
  arduino_mini_base_controller drives each wheel with a feed-forward term plus
  a PID correction. The feed-forward inverts a per-wheel, per-direction
  straight-line model of the motor:

      |speed_mm_s| = slope * pwm + intercept

  This sketch measures that line on the real hardware and least-squares fits
  it, so the four commissioning arrays in the main firmware
  (arduino_mini_base_controller/Config.h)

      FF_SLOPE_MM_S_PER_PWM_FWD[]   FF_INTERCEPT_MM_S_FWD[]
      FF_SLOPE_MM_S_PER_PWM_REV[]   FF_INTERCEPT_MM_S_REV[]

  come from data instead of a guess. Brushed gearmotors differ from each other
  by tens of percent and are direction-asymmetric; that asymmetry is what makes
  a robot drive straight forward and curve in reverse.

  It is a bench tool: no IMU, no odometry, no PID, no ROS protocol. Flash it,
  calibrate, paste the printed constants into the firmware's Config.h, then
  flash the main firmware back.

  The pin map, wheel geometry and commissioning signs are NOT duplicated here.
  They come from the shared MiniRobotBase library (libraries/MiniRobotBase/),
  which the firmware uses too, so the two can no longer disagree about what a
  tick is worth.

  Sketch layout (the IDE compiles every .cpp/.h in this folder):
    Config.h            baud, fit thresholds, sweep settings
    LinearFit.h         Welford streaming least-squares
    FitResults.h/.cpp   the results table and the paste-ready constants block
    CalUtil.h/.cpp      abort handling, timing, one speed measurement
    Sweep.h/.cpp        the PWM sweep and the fit
    Verify.h/.cpp       open-loop check of the fitted constants
    Interactive.h/.cpp  HOLD, TICKS, CONFIG, HELP
    CommandParser.h/.cpp  the serial command set
    arduino_mini_motor_calibration.ino  setup()/loop() only

  HOW TO USE IT
  ------------------------------------------------------------
  1. Put the robot on blocks so BOTH WHEELS SPIN FREE. Every measurement here
     is open loop and runs up to MAX_DRIVE_PWM.
  2. Use a charged battery, and keep it charged across the whole run. The fit
     is only valid at the bus voltage it was measured at, so a sagging battery
     bakes a wrong slope into the firmware.
  3. Flash this sketch and open a serial terminal at 115200 baud, line ending
     "Newline" (carriage return works too).
  4. Check MEASURED_COUNTS_PER_REV in libraries/MiniRobotBase/src/
     MiniRobotGeometry.h is right for your encoders. Everything printed here is
     in mm/s derived from it, so if it is wrong the whole fit is wrong by the
     same factor. Verify with:  TICKS,20
     then turn one wheel by hand exactly one full turn and read the total.
  5. Run:  CAL
     A few minutes: both wheels, both directions, two passes each.
  6. Paste the printed "const float FF_..." block into
     arduino_mini_base_controller/Config.h, replacing the four existing lines.
  7. Optional but recommended:  VERIFY
     drives a few target speeds through the freshly fitted feed-forward and
     reports the open-loop error. Under ~10% is a good fit; PID cleans up the
     rest.

  Sending ANY character while a sweep is running aborts it and coasts the
  motors.

  COMMANDS
  ------------------------------------------------------------
    CAL                        calibrate both motors, both directions
    CAL,<motor>                one motor (1 = left, 2 = right), both directions
    SWEEP,<motor>[,R]          one motor, one direction (R = reverse)
    VERIFY                     open-loop check of the fitted constants
    VERIFY,<motor>             ... for one motor
    RESULT                     reprint the last fit table and constants block
    HOLD,<motor>,<pwm>[,R]     hold one PWM and stream the measured speed
    TICKS[,<seconds>]          stream raw encoder counts (counts-per-rev check)
    STEP,<pwm_step>            PWM step size             (default 10)
    RANGE,<min_pwm>,<max_pwm>  PWM sweep range           (default 30..210)
    DWELL,<settle_ms>,<win_ms> per-step timing           (default 400, 600)
    PASSES,<1_to_4>            sweep passes per direction (default 2: up, down)
    CONFIG                     print the current sweep settings
    STOP                       coast both motors
    HELP                       print this list

  OUTPUT LINES
  ------------------------------------------------------------
    C,<motor>,<F|R>,<pwm>,<mm_s>,<USED|STALL>   one measured point
    R,<motor>,<F|R>,<slope>,<intercept>,<r2>,<n>,<deadzone_pwm>,<mm_s_at_max>
    V,<motor>,<F|R>,<target_mm_s>,<measured_mm_s>,<error_pct>
    TICKS,<left>,<right>
    INFO,... / WARN,... / ERR,... / ACK,...

  The C rows are plain CSV, so you can also capture them and fit offline if you
  would rather look at the scatter yourself.

  WIRING -- see libraries/MiniRobotBase/src/MiniRobotPins.h, which is the one
  copy both this sketch and the firmware read. The IMU is not used here and can
  stay connected or be left off.
*/

#include <MiniRobotEncoders.h>
#include <MiniRobotMotors.h>

#include "CommandParser.h"
#include "Config.h"
#include "FitResults.h"
#include "Interactive.h"

void setup() {
  Serial.begin(SERIAL_BAUD);

  motorsBegin();     // outputs, ~31 kHz PWM, STBY high
  encodersBegin();
  resetFitResults();

  motorsCoastAll();
  delay(200);

  Serial.println();
  Serial.println(F("READY,mini_motor_ff_calibration_v1"));
  Serial.println(F("WARN,PUT_THE_ROBOT_ON_BLOCKS_BOTH_WHEELS_MUST_SPIN_FREE"));
  Serial.println(F("INFO,SEND_CAL_TO_START_OR_HELP_FOR_COMMANDS"));
  printConfig();
}

void loop() {
  readSerialCommands();
}
