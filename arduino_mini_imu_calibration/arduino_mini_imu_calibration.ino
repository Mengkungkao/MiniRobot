/*
  mini ROS2 base -- IMU calibration bench sketch (MPU9250 / MPU9255 / MPU6500)

  WHAT THIS IS FOR
  ------------------------------------------------------------
  arduino_mini_base_controller runs a Mahony filter over this IMU and publishes
  the fused heading as the robot's yaw. Four things it depends on are not
  measured anywhere in the firmware:

    gyro zero-rate offset   the firmware re-measures this at every boot, but
                            nothing tells you whether the sensor is quiet
                            enough for that measurement to hold
    accelerometer bias      the firmware applies none at all; the accelerometer
    and scale               is the filter's only gravity reference, so its
                            errors go straight into roll, pitch and the yaw
                            they are computed against
    hard/soft iron          the magOffset and magScale statics in Imu.cpp ship
                            at 0 and 1, i.e. uncalibrated, which is why
                            nine-axis fusion is off by default
    IMU_YAW_SIGN            a commissioning value in Config.h, currently a guess
                            until someone turns the robot and looks

  This sketch measures all four on the real hardware and prints them ready to
  paste. It also answers the question the firmware's comments keep deferring --
  whether the magnetometer is usable on this robot at all -- by spinning the
  wheels and watching what the motor current does to the field.

  It is a bench tool: no odometry, no PID, no ROS protocol. Flash it, calibrate,
  paste the constants into the firmware, then flash the firmware back.

  IT MUST CONFIGURE THE SENSOR THE WAY THE FIRMWARE DOES. Full-scale range sets
  what one LSB is worth, and the DLPF sets how much noise survives averaging, so
  a bias measured at one setting does not describe a sensor running at another.
  ImuDriver.cpp holds the same +/-2 g, +/-250 deg/s, 41 Hz DLPF, 200 Hz setup as
  Imu.cpp. Change one, change the other.

  The pin map and the motor helpers live in the MiniRobot* files in this folder,
  kept byte-identical to the copies in arduino_mini_base_controller/ and
  arduino_mini_motor_calibration/. Only MOTORS uses them.

  Sketch layout (the IDE compiles every .cpp/.h in this folder):
    Config.h              sample counts, thresholds, warning limits
    Stats.h               Welford streaming mean/variance/min/max
    ImuDriver.h/.cpp      detection, register setup, scaled reads
    CalUtil.h/.cpp        abort handling, prompts, stillness, sampling
    GyroCal.h/.cpp        zero-rate offset, noise, drift
    AccelCal.h/.cpp       six-face bias and scale
    MagCal.h/.cpp         hard and soft iron, plus a check pass
    MotorNoise.h/.cpp     what the wheels do to the magnetometer
    Interactive.h/.cpp    CHECK, YAWSIGN, LIVE, RAW, CONFIG, HELP
    Results.h/.cpp        the results table and the paste blocks
    CommandParser.h/.cpp  the serial command set
    arduino_mini_imu_calibration.ino  setup()/loop() only

  HOW TO USE IT
  ------------------------------------------------------------
  1. Flash this sketch and open a serial terminal at 115200 baud. Any line
     ending works -- "Newline", "Carriage return" or "Both NL & CR".
  2. Send CHECK. It names the part, measures the read rate, and reports whether
     gravity and the local field come out the right size. If it says NONE, the
     IMU is not on the bus: SDA on A4, SCL on A5, and power to match the module.
  3. Leave the robot powered for a few minutes before calibrating. The gyro
     bias moves while the die warms up, and a bias measured cold is one the
     sensor has already left behind -- GYRO says so if it sees it happening.
  4. Send CAL and follow the ASK lines. It walks through the health check, the
     gyro, the six accelerometer faces, the yaw sign and the magnetometer, then
     prints everything ready to paste. Ten minutes, most of it spent moving the
     robot around. The stages also exist as separate commands if you only want
     one of them.
  5. Paste the printed blocks into the firmware. The magnetometer lines replace
     two existing lines in Imu.cpp; the accelerometer lines are new code, so the
     block prints the three lines of readAccelGyro() they belong in as well.
  6. Send MOTORS, with the robot on blocks, before you ever enable nine-axis
     fusion. If it reports the field moving by more than a few degrees with the
     wheels spinning, the magnetometer calibration is fine and still unusable:
     heading would become a function of throttle. Leave MAG fusion off.
  7. Send LIVE and watch the corrected numbers with the robot sitting still.
     Roll and pitch should sit within a fraction of a degree, and the field
     magnitude should barely change as you turn the robot.

  Any character aborts a running measurement. At an ASK prompt, X cancels and
  anything else (including a bare Enter) continues.

  COMMANDS
  ------------------------------------------------------------
    CAL                    guided full run: CHECK, GYRO, ACCEL, YAWSIGN, MAG
    CHECK                  quick health check of the sensor
    GYRO                   zero-rate offset, noise and drift
    ACCEL                  six-face bias and scale
    FACE                   measure one face where the robot sits now
    MAG[,seconds]          hard and soft iron, figure-of-eight
    MOTORS                 what the wheels do to the magnetometer
    YAWSIGN                work out IMU_YAW_SIGN
    LIVE[,seconds]         stream corrected values
    RAW[,seconds]          stream raw LSB counts
    RESULT                 reprint the table and the paste blocks
    SAMPLES,<gyro>,<accel> set sample counts
    MAGTIME,<seconds>      set the magnetometer sweep length
    CONFIG                 print settings and time estimates
    STOP                   coast both motors
    HELP                   print this list

  OUTPUT LINES
  ------------------------------------------------------------
    G,<axis>,<bias_deg_s>,<noise_deg_s>,<drift_deg_s>
    AF,<+/-axis>,<x_g>,<y_g>,<z_g>,<OK|NOT_FLAT>     one measured face
    A,<axis>,<bias_g>,<scale>                        the fitted result
    M,<seconds>,<samples>,<spanX>,<spanY>,<spanZ>    magnetometer progress
    M,<axis>,<offset_uT>,<scale>[,<span_uT>]
    MI,<pwm>,<field_deg>,<field_uT>,<gyro_z_noise>   motor interference
    CHECK,<what>,<values...>,<verdict>
    L,... / RAW,...                                  live streams
    HDR,... names the columns of whatever follows
    ASK,... wants an answer / INFO,... / WARN,... / ERR,... / ACK,...

  WIRING -- IMU on the I2C pins, SDA to A4 and SCL to A5, VCC to 3.3V or 5V
  depending on the module, GND common. MOTORS additionally needs the TB6612FNG
  and a charged battery; every other command works with the IMU alone.
*/

#include "MiniRobotMotors.h"

#include "AccelCal.h"
#include "CommandParser.h"
#include "Config.h"
#include "ImuDriver.h"
#include "Interactive.h"
#include "Results.h"

void setup() {
  Serial.begin(SERIAL_BAUD);

  // The wheels only turn for MOTORS, but the driver is brought up here so it
  // is in a known coasted state rather than floating from the last sketch.
  motorsBegin();
  motorsCoastAll();

  resetResults();
  resetAccelFaces();

  Serial.println();
  Serial.println(F("READY,mini_imu_calibration_v1"));

  if (!imuBegin()) {
    Serial.println(F("ERR,NO_IMU_FOUND,CHECK_SDA_A4_SCL_A5_AND_POWER"));
    Serial.println(F("INFO,COMMANDS_STILL_RESPOND,FIX_THE_WIRING_AND_RESET"));
  }

  imuPrintInfo();
  Serial.println(F("INFO,SEND_CHECK_FIRST_THEN_CAL,OR_HELP_FOR_COMMANDS"));
  Serial.println(F("INFO,LET_THE_IMU_WARM_UP_FOR_A_FEW_MINUTES_BEFORE_CALIBRATING"));
  printConfig();
}

void loop() {
  readSerialCommands();
}
