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

  It also commissions the two direction signs in MiniRobotGeometry.h, because
  a fit is meaningless until "forward" means the same thing to the encoder, to
  the motor and to the robot. See DIR below.

  It is a bench tool and nothing more. It does exactly five things: drive the
  TB6612 channels, run motors 1 and 2, work out which way each one turns, read
  the encoders, and print the measured points plus the fitted forward and
  reverse curves. No IMU, no odometry, no PID, no ROS protocol, no closed-loop
  anything. Flash it, calibrate, paste the printed constants into the firmware,
  then flash the main firmware back.

  The pin map, wheel geometry and commissioning signs live in the MiniRobot*
  files in this folder, which are kept byte-identical to the copies in
  arduino_mini_base_controller/. The two MUST agree about what a tick is worth:
  this sketch reports mm/s scaled by those numbers and the firmware consumes the
  fit scaled by the same numbers. If you edit one copy, copy it to the other.

  Sketch layout (the IDE compiles every .cpp/.h in this folder):
    Config.h            baud, fit thresholds, CPR, DIR and sweep settings
    Counts.h/.cpp       the session counts-per-rev and CPR
    Direction.h/.cpp    the session signs and DIR
    Measure.h/.cpp      abort handling, one speed measurement
    Fit.h/.cpp          least squares, the results table, the paste block
    Sweep.h/.cpp        the PWM sweep and CAL
    Commands.h/.cpp     the serial command set, CONFIG and HELP
    arduino_mini_motor_calibration.ino  setup()/loop() only

  HOW TO USE IT
  ------------------------------------------------------------
  1. Put the robot on blocks so BOTH WHEELS SPIN FREE. Every measurement here
     is open loop and runs up to MAX_DRIVE_PWM.
  2. Use a charged battery, and keep it charged across the whole run. The fit
     is only valid at the bus voltage it was measured at, so a sagging battery
     bakes a wrong slope into the firmware.
  3. Flash this sketch and open a serial terminal at 500000 baud. Any line
     ending works -- "Newline", "Carriage return" or "Both NL & CR".
  4. Run:  CPR
     FIRST, before anything else is measured. It asks you to turn each wheel a
     known number of full turns by hand, counts the ticks, and divides.

     Counts per revolution is the scale factor under every other number here
     and in the firmware. Get it wrong and the whole fit is wrong by exactly
     the same factor, silently: the robot outruns or under-runs its command
     while telemetry, odometry and the PID all agree it is on target. Nothing
     downstream can detect it, which is why this goes first.

     It has to be by hand. The number is counts per turn of the WHEEL, after
     the gearbox, and nothing on the board knows the gear ratio -- driving the
     motor would only count ticks against a clock, which says nothing about how
     far the wheel actually went.

     Mark the tyre and the chassis so a full turn is unambiguous, turn the
     wheel, press Enter. Default is 5 turns; more is better, because the error
     is how precisely you stop on the mark and that divides down. CPR,10 for a
     careful job, CPR,1 for a quick re-check.
  5. Run:  DIR
     Thirty seconds, and it is not optional.

     DIR spins each wheel at a fixed driver polarity, ignoring both signs, and
     asks you which way the robot tried to roll. From that one answer it
     derives BOTH signs at once -- which polarity means forward
     (MOTOR_DIRECTION_SIGN) and which way the encoder counts when it does
     (ENCODER_SIGN) -- then drives the wheel forward through them to confirm.

     Answer by watching the WHEEL, not the numbers. The encoder cannot see the
     robot: it only knows its own count went up or down, and both signs sit
     between that count and the robot frame. Flip either one alone and the
     reported speed changes sign; flip BOTH and it does not, but the wheel now
     turns the other way. That pair is the failure DIR exists to prevent -- it
     reports a healthy positive speed while the robot drives away backwards,
     the fit succeeds, and the PID agrees with itself the whole time.

     Whatever the signs end up as is printed ready to paste into
     MiniRobotGeometry.h in BOTH sketch folders. Nothing is persisted, so a
     session that ends without pasting them measured a robot that does not
     exist yet.
  6. Sweep the range you actually drive in. The firmware clamps each wheel to
     MAX_WHEEL_MM_S, so if a wheel reaches 450 mm/s at PWM 105 the robot will
     never ask for more than about PWM 60, and a 10..210 sweep spends most of
     its points where the robot never operates. Narrow it:  RANGE,8,60
  7. Run:  CAL
     A few minutes: both wheels, both directions, two passes each.
  8. Paste the printed "const float FF_..." block into
     arduino_mini_base_controller/Config.h, replacing the four existing lines.
     Never paste a block containing 0.000f -- that wheel has no fit.

  Sending any character while a sweep is running aborts it and coasts the
  motors. Bare line endings do not count, so a terminal that appends CR+LF
  cannot abort the very run it just started. CPR and DIR are the exception:
  they are asking you questions, so they read what you type as the answer.
  Abort either by answering X.

  COMMANDS
  ------------------------------------------------------------
    CPR                        hand-spin both wheels, count ticks per revolution
    CPR,<turns>                ... asking for that many full turns (default 5)
    CPR,<motor>,<turns>        ... for one wheel only
    DIR                        commission both signs from one look per wheel
    DIR,<motor>                ... for one wheel only
    SIGN                       print the commissioning signs
    CAL                        sweep and fit both motors, both directions
    CAL,<motor>                one motor (1 = left, 2 = right), both directions
    SWEEP,<motor>[,R]          one motor, one direction (R = reverse)
    RESULT                     reprint the last fit table and constants block
    STEP,<pwm_step>            PWM step size             (default 10)
    RANGE,<min_pwm>,<max_pwm>  PWM sweep range           (default 10..210)
    DWELL,<settle_ms>,<win_ms> per-step timing           (default 400, 600)
    PASSES,<1_to_4>            sweep passes per direction (default 2: up, down)
    CONFIG                     print the current sweep settings
    STOP                       coast both motors
    HELP                       print this list

  OUTPUT LINES
  ------------------------------------------------------------
    C,<motor>,<F|R>,<pwm>,<mm_s>,<USED|STALL>   one measured point
    R,<motor>,<F|R>,<slope>,<intercept>,<r2>,<n>,<deadzone_pwm>,<mm_s_at_max>
    DIR,<motor>,F,<pwm>,<mm_s>,<OK|FAILED>      DIR's post-commission self check
    ASK,...                                     waiting for an operator answer
    SIGN,<motor>,ENC,<sign>,MOT,<sign>
    CPR,LIVE,<left>,<right>                     running totals while you spin
    CPR,<motor>,TICKS,<n>,TURNS,<n>,COUNTS_PER_REV,<n>
    CPR,<motor>,COUNTS_PER_REV,<n>,MM_PER_COUNT,<n>
    INFO,... / WARN,... / ERR,... / ACK,...

  The C rows are plain CSV, so you can also capture them and fit offline if you
  would rather look at the scatter yourself.

  WIRING -- see MiniRobotPins.h in this folder, which is byte-identical to the
  firmware's copy. The IMU is not used here and can stay connected or be left
  off.
*/

#include "MiniRobotEncoders.h"
#include "MiniRobotMotors.h"

#include "Commands.h"
#include "Config.h"
#include "Counts.h"
#include "Direction.h"
#include "Fit.h"

void setup() {
  Serial.begin(SERIAL_BAUD);

  countsBegin();     // session copy of MEASURED_COUNTS_PER_REV
  signsBegin();      // session copies of ENCODER_SIGN / MOTOR_DIRECTION_SIGN
  motorsBegin();     // outputs, ~31 kHz PWM, STBY high
  encodersBegin();
  resetFitResults();

  motorsCoastAll();
  delay(200);

  Serial.println();
  Serial.println(F("READY,mini_motor_ff_calibration_v2"));
  Serial.println(F("WARN,PUT_THE_ROBOT_ON_BLOCKS_BOTH_WHEELS_MUST_SPIN_FREE"));
  Serial.println(F("INFO,RUN,DIR,FIRST_THEN,CAL,OR,HELP,FOR_COMMANDS"));
  printConfig();
}

void loop() {
  readSerialCommands();
}
