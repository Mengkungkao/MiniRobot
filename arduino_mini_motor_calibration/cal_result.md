# Motor calibration results

Run on 2026-08-25 with the robot **on blocks**, both wheels
free. Produced by `arduino_mini_motor_calibration` and written up by
`scripts/analyse_motor_calibration.py`.

To reproduce:

```bash
bash scripts/build_arduino.sh arduino_mini_motor_calibration
python3 scripts/flash_arduino.py build/arduino_mini_motor_calibration/*.hex
# then over serial at 115200 baud:
#   RANGE,4,72   STEP,2   DWELL,400,600   PASSES,2   CAL
python3 scripts/analyse_motor_calibration.py cal_raw.log -o cal_result.md
```

## Session

| | |
| --- | --- |
| geometry | `counts=520.0/520.0,dia=47.00,maxpwm=210` |
| sweep | `range=4..72,step=2,settle_ms=400,window_ms=600,passes=2` |
| motor 1 signs | `ENCODER_SIGN -1`, `MOTOR_DIRECTION_SIGN -1` |
| motor 2 signs | `ENCODER_SIGN +1`, `MOTOR_DIRECTION_SIGN -1` |
| samples | 280 (252 used, 28 below the stall threshold) |
| run time | 283.2 s |

The signs above must match what the base controller reports for `CFG`. They
decide what every number below **means**: a sweep labelled forward that
actually drove the wheel backwards fits a line to the wrong quadrant.

## Fits

`speed_mm_s = slope * pwm + intercept`, fitted over the points above the
stall threshold. Two fits are shown per row: the one the sketch computed
on-board and pasted into `Config.h`, and an independent least-squares fit
of the same points done here. They should agree to the last digit; a
disagreement means one of the two is reading the data wrong.

| motor | direction | slope | intercept | r² | n | dead zone PWM | mm/s at PWM 210 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | forward (board) | 6.4639 | -44.31 | 1.0000 | 64 | 6.9 | 1313 |
| 1 | forward (refit) | 6.4639 | -44.31 | 1.0000 | 64 | 6.9 | 1313 |
| 1 | reverse (board) | 6.2624 | -44.38 | 1.0000 | 64 | 7.1 | 1271 |
| 1 | reverse (refit) | 6.2624 | -44.38 | 1.0000 | 64 | 7.1 | 1271 |
| 2 | forward (board) | 6.2343 | -44.78 | 0.9999 | 63 | 7.2 | 1264 |
| 2 | forward (refit) | 6.2343 | -44.79 | 0.9999 | 63 | 7.2 | 1264 |
| 2 | reverse (board) | 6.4385 | -49.07 | 1.0000 | 61 | 7.6 | 1303 |
| 2 | reverse (refit) | 6.4385 | -49.07 | 1.0000 | 61 | 7.6 | 1303 |

## Where the line stops describing the motor

The fit's dead zone is where the fitted line crosses zero speed. The
measured breakaway is the lowest PWM that actually produced motion. The
gap between them is real and is why `FF_BREAKAWAY_PWM` exists in the
firmware: inside it the bridge is energised and the wheel is not turning.

| motor | direction | fit says zero at PWM | last stalled PWM | first moving PWM | first speed | verdict |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | forward | 6.9 | 8 | 10 | 21.3 mm/s | clean |
| 1 | reverse | 7.1 | 8 | 10 | 16.1 mm/s | clean |
| 2 | forward | 7.2 | 10 | 10 | 17.5 mm/s | contested — see below |
| 2 | reverse | 7.6 | 12 | 12 | 29.8 mm/s | contested — see below |

**Contested** means the two passes disagreed at the same PWM: going up,
the wheel was still stalled; coming down, it was still turning. That is
stiction, and it means breakaway is a band rather than a number. The
firmware's `FF_BREAKAWAY_PWM` has to sit at the top of that band, because
a wheel starting from rest is always on the ascending side of it.

## Breakaway floor for the firmware

`FF_BREAKAWAY_PWM` in `arduino_mini_base_controller/Config.h` is the output
floor applied to a wheel that is commanded and not turning. The number that
belongs there is the **ascending** pass's first moving PWM: a wheel starting
from rest has to break stiction, which is the ascending case. Taking it from
the descending pass, or from the fit's extrapolated dead zone, gives a floor
below the PWM at which the wheel actually starts — and a wheel commanded into
that gap draws current and does nothing until the stall boost rescues it.

It is one value per wheel, not per direction, so each wheel takes its worse
direction.

| motor | forward | reverse | floor needed |
| --- | --- | --- | --- |
| 1 | 10 | 10 | **10** |
| 2 | 12 | 14 | **14** |

```cpp
const float FF_BREAKAWAY_PWM[MOTOR_COUNT] = {10.0f, 14.0f};
```

The two wheels differ, which is the argument against a single shared
floor: setting both to the higher value makes the easier wheel lurch at
every start, and both to the lower leaves the stiffer one dead.

## Residuals

How far the real motor departs from the straight line, over the fitted
range. Small and unstructured means linear is a fair model; a large or
systematically curved residual means it is not, and the feed-forward will
be wrong at the ends of the range even though r² looks perfect.

| motor | direction | RMS residual | worst residual | worst as % of range |
| --- | --- | --- | --- | --- |
| 1 | forward | 0.82 mm/s | 2.38 mm/s | 0.6% |
| 1 | reverse | 0.60 mm/s | 2.16 mm/s | 0.6% |
| 2 | forward | 1.12 mm/s | 2.37 mm/s | 0.6% |
| 2 | reverse | 0.60 mm/s | 1.62 mm/s | 0.4% |

## Hysteresis: sweeping up vs sweeping down

Each direction is swept twice, ascending then descending. A gearmotor
settles at a slightly different speed at the same PWM depending on which
side it approached from, and the fit lands in the middle of that loop.
A large gap here is backlash or a dragging gearbox, not measurement noise.

| motor | direction | mean up − down | largest gap | at PWM |
| --- | --- | --- | --- | --- |
| 1 | forward | +0.81 mm/s | +3.31 mm/s | 10 |
| 1 | reverse | +0.01 mm/s | +1.42 mm/s | 14 |
| 2 | forward | -2.01 mm/s | -17.48 mm/s | 10 |
| 2 | reverse | -1.38 mm/s | -29.81 mm/s | 12 |

## Comparisons that matter to the robot

- **Motor 1, forward vs reverse:** slopes differ by 3.1%. This is why the firmware keeps a separate fit per direction; one averaged line would under-drive one way and over-drive the other.
- **Motor 2, forward vs reverse:** slopes differ by 3.2%. This is why the firmware keeps a separate fit per direction; one averaged line would under-drive one way and over-drive the other.
- **Motor 1 vs motor 2, forward:** slopes differ by 3.6%. Same command, different PWM needed — which is what the per-wheel PID trims out, and why a shared fit would make the robot curve.
- **Motor 1 vs motor 2, reverse:** slopes differ by 2.7%. Same command, different PWM needed — which is what the per-wheel PID trims out, and why a shared fit would make the robot curve.

## Curve

```text
   421 |                                                               1
   397 |                                                             * *
   372 |                                                         * * *  
   347 |                                                     * * *      
   322 |                                                  ** *          
   297 |                                              * * *             
   273 |                                          1 * *                 
   248 |                                      1 * *                     
   223 |                                   1 **                         
   198 |                               1 * *                            
   174 |                           1 * *                                
   149 |                        1* *                                    
   124 |                    1 * *                                       
    99 |                  * *                                           
    74 |              * *                                               
    50 |           **                                                   
    25 |       * *                                                      
     0 |** * * 4                                                        
       +----------------------------------------------------------------
        4                                                             72
        PWM

  1 = motor 1 forward   2 = motor 1 reverse   3 = motor 2 forward   4 = motor 2 reverse
  * = two or more curves in the same cell
```

## Speed at each PWM step

Both passes averaged, so this is the line the fit actually sees. Blank
where the wheel did not turn.

| PWM | m1 forward | m1 reverse | m2 forward | m2 reverse |
| ---: | ---: | ---: | ---: | ---: |
| 4 | stall | stall | stall | stall |
| 6 | stall | stall | stall | stall |
| 8 | stall | stall | stall | stall |
| 10 | 19.6 | 16.1 | 8.7 | 7.3 |
| 12 | 33.4 | 31.2 | 30.3 | 14.9 |
| 14 | 46.5 | 43.8 | 43.0 | 41.6 |
| 16 | 59.2 | 56.3 | 55.6 | 54.7 |
| 18 | 72.1 | 68.6 | 67.8 | 67.1 |
| 20 | 84.7 | 81.2 | 80.2 | 80.2 |
| 22 | 97.7 | 93.6 | 92.9 | 92.8 |
| 24 | 111.1 | 105.8 | 105.0 | 105.6 |
| 26 | 123.5 | 118.8 | 117.4 | 118.1 |
| 28 | 136.5 | 131.1 | 129.9 | 130.9 |
| 30 | 149.3 | 143.5 | 141.7 | 143.6 |
| 32 | 161.9 | 156.5 | 154.3 | 156.4 |
| 34 | 175.5 | 168.5 | 166.7 | 169.6 |
| 36 | 188.1 | 181.3 | 179.1 | 181.8 |
| 38 | 201.7 | 194.2 | 191.4 | 195.0 |
| 40 | 214.7 | 205.7 | 204.6 | 207.9 |
| 42 | 227.3 | 218.5 | 216.8 | 220.8 |
| 44 | 240.2 | 231.1 | 228.6 | 233.8 |
| 46 | 253.7 | 243.0 | 241.9 | 247.1 |
| 48 | 266.5 | 256.5 | 254.6 | 259.6 |
| 50 | 279.2 | 268.9 | 266.2 | 272.6 |
| 52 | 292.0 | 281.1 | 279.0 | 285.4 |
| 54 | 304.8 | 293.7 | 291.5 | 298.9 |
| 56 | 318.2 | 306.4 | 304.3 | 311.6 |
| 58 | 330.8 | 318.7 | 316.6 | 324.6 |
| 60 | 343.8 | 331.2 | 329.4 | 337.1 |
| 62 | 356.7 | 343.6 | 341.7 | 350.2 |
| 64 | 369.5 | 355.9 | 354.6 | 363.4 |
| 66 | 382.2 | 369.1 | 366.9 | 376.4 |
| 68 | 394.0 | 381.6 | 379.7 | 389.0 |
| 70 | 407.5 | 394.1 | 392.2 | 402.4 |
| 72 | 420.8 | 406.9 | 404.8 | 414.6 |

## Every sample

One row per measurement, in the order they were taken. `pass` 0 is the
ascending sweep and 1 the descending one. This is the block to load into
a plotting tool.

```csv
motor,direction,pass,pwm,speed_mm_s,used
1,forward,0,4,0.00,0
1,forward,0,6,0.00,0
1,forward,0,8,0.00,0
1,forward,0,10,21.26,1
1,forward,0,12,35.02,1
1,forward,0,14,47.72,1
1,forward,0,16,60.10,1
1,forward,0,18,73.23,1
1,forward,0,20,85.66,1
1,forward,0,22,98.91,1
1,forward,0,24,111.69,1
1,forward,0,26,124.47,1
1,forward,0,28,136.54,1
1,forward,0,30,149.55,1
1,forward,0,32,162.33,1
1,forward,0,34,175.76,1
1,forward,0,36,187.88,1
1,forward,0,38,202.22,1
1,forward,0,40,214.97,1
1,forward,0,42,227.16,1
1,forward,0,44,240.41,1
1,forward,0,46,253.71,1
1,forward,0,48,266.44,1
1,forward,0,50,279.69,1
1,forward,0,52,292.94,1
1,forward,0,54,305.25,1
1,forward,0,56,318.50,1
1,forward,0,58,331.28,1
1,forward,0,60,344.05,1
1,forward,0,62,356.71,1
1,forward,0,64,369.14,1
1,forward,0,66,381.92,1
1,forward,0,68,394.69,1
1,forward,0,70,407.47,1
1,forward,0,72,420.25,1
1,forward,1,72,421.44,1
1,forward,1,70,407.47,1
1,forward,1,68,393.27,1
1,forward,1,66,382.39,1
1,forward,1,64,369.94,1
1,forward,1,62,356.71,1
1,forward,1,60,343.58,1
1,forward,1,58,330.33,1
1,forward,1,56,317.97,1
1,forward,1,54,304.30,1
1,forward,1,52,291.05,1
1,forward,1,50,278.75,1
1,forward,1,48,266.47,1
1,forward,1,46,253.66,1
1,forward,1,44,239.94,1
1,forward,1,42,227.35,1
1,forward,1,40,214.50,1
1,forward,1,38,201.27,1
1,forward,1,36,188.35,1
1,forward,1,34,175.28,1
1,forward,1,32,161.38,1
1,forward,1,30,149.07,1
1,forward,1,28,136.54,1
1,forward,1,26,122.57,1
1,forward,1,24,110.56,1
1,forward,1,22,96.54,1
1,forward,1,20,83.77,1
1,forward,1,18,70.87,1
1,forward,1,16,58.21,1
1,forward,1,14,45.36,1
1,forward,1,12,31.71,1
1,forward,1,10,17.95,1
1,forward,1,8,0.00,0
1,forward,1,6,0.00,0
1,forward,1,4,0.00,0
1,reverse,0,4,0.00,0
1,reverse,0,6,0.00,0
1,reverse,0,8,0.00,0
1,reverse,0,10,16.09,1
1,reverse,0,12,31.66,1
1,reverse,0,14,44.49,1
1,reverse,0,16,56.79,1
1,reverse,0,18,68.51,1
1,reverse,0,20,81.40,1
1,reverse,0,22,94.02,1
1,reverse,0,24,105.54,1
1,reverse,0,26,119.06,1
1,reverse,0,28,131.09,1
1,reverse,0,30,143.63,1
1,reverse,0,32,156.39,1
1,reverse,0,34,168.48,1
1,reverse,0,36,180.78,1
1,reverse,0,38,193.71,1
1,reverse,0,40,205.05,1
1,reverse,0,42,218.17,1
1,reverse,0,44,231.12,1
1,reverse,0,46,242.78,1
1,reverse,0,48,256.55,1
1,reverse,0,50,268.39,1
1,reverse,0,52,281.12,1
1,reverse,0,54,293.89,1
1,reverse,0,56,306.19,1
1,reverse,0,58,318.50,1
1,reverse,0,60,331.67,1
1,reverse,0,62,343.58,1
1,reverse,0,64,356.36,1
1,reverse,0,66,368.66,1
1,reverse,0,68,380.97,1
1,reverse,0,70,394.51,1
1,reverse,0,72,407.27,1
1,reverse,1,72,406.52,1
1,reverse,1,70,393.75,1
1,reverse,1,68,382.22,1
1,reverse,1,66,369.47,1
1,reverse,1,64,355.41,1
1,reverse,1,62,343.58,1
1,reverse,1,60,330.80,1
1,reverse,1,58,318.91,1
1,reverse,1,56,306.63,1
1,reverse,1,54,293.42,1
1,reverse,1,52,281.12,1
1,reverse,1,50,269.31,1
1,reverse,1,48,256.50,1
1,reverse,1,46,243.25,1
1,reverse,1,44,231.04,1
1,reverse,1,42,218.75,1
1,reverse,1,40,206.34,1
1,reverse,1,38,194.66,1
1,reverse,1,36,181.73,1
1,reverse,1,34,168.48,1
1,reverse,1,32,156.60,1
1,reverse,1,30,143.40,1
1,reverse,1,28,131.09,1
1,reverse,1,26,118.59,1
1,reverse,1,24,106.01,1
1,reverse,1,22,93.23,1
1,reverse,1,20,80.93,1
1,reverse,1,18,68.62,1
1,reverse,1,16,55.75,1
1,reverse,1,14,43.07,1
1,reverse,1,12,30.71,1
1,reverse,1,10,16.09,1
1,reverse,1,8,0.00,0
1,reverse,1,6,0.00,0
1,reverse,1,4,0.00,0
2,forward,0,4,0.00,0
2,forward,0,6,0.00,0
2,forward,0,8,0.00,0
2,forward,0,10,0.00,0
2,forward,0,12,29.81,1
2,forward,0,14,42.99,1
2,forward,0,16,55.37,1
2,forward,0,18,67.56,1
2,forward,0,20,79.51,1
2,forward,0,22,92.60,1
2,forward,0,24,104.12,1
2,forward,0,26,116.42,1
2,forward,0,28,128.98,1
2,forward,0,30,140.56,1
2,forward,0,32,153.33,1
2,forward,0,34,165.84,1
2,forward,0,36,177.94,1
2,forward,0,38,190.25,1
2,forward,0,40,203.16,1
2,forward,0,42,214.86,1
2,forward,0,44,227.16,1
2,forward,0,46,240.48,1
2,forward,0,48,253.19,1
2,forward,0,50,264.58,1
2,forward,0,52,277.81,1
2,forward,0,54,290.10,1
2,forward,0,56,302.88,1
2,forward,0,58,315.19,1
2,forward,0,60,328.84,1
2,forward,0,62,340.74,1
2,forward,0,64,353.99,1
2,forward,0,66,366.30,1
2,forward,0,68,379.86,1
2,forward,0,70,392.62,1
2,forward,0,72,405.38,1
2,forward,1,72,404.16,1
2,forward,1,70,391.85,1
2,forward,1,68,379.55,1
2,forward,1,66,367.58,1
2,forward,1,64,355.29,1
2,forward,1,62,342.64,1
2,forward,1,60,329.86,1
2,forward,1,58,318.03,1
2,forward,1,56,305.69,1
2,forward,1,54,292.93,1
2,forward,1,52,280.17,1
2,forward,1,50,267.86,1
2,forward,1,48,256.08,1
2,forward,1,46,243.32,1
2,forward,1,44,230.00,1
2,forward,1,42,218.75,1
2,forward,1,40,205.99,1
2,forward,1,38,192.61,1
2,forward,1,36,180.31,1
2,forward,1,34,167.53,1
2,forward,1,32,155.23,1
2,forward,1,30,142.92,1
2,forward,1,28,130.87,1
2,forward,1,26,118.31,1
2,forward,1,24,105.83,1
2,forward,1,22,93.23,1
2,forward,1,20,80.93,1
2,forward,1,18,68.04,1
2,forward,1,16,55.84,1
2,forward,1,14,42.99,1
2,forward,1,12,30.76,1
2,forward,1,10,17.48,1
2,forward,1,8,0.00,0
2,forward,1,6,0.00,0
2,forward,1,4,0.00,0
2,reverse,0,4,0.00,0
2,reverse,0,6,0.00,0
2,reverse,0,8,0.00,0
2,reverse,0,10,0.00,0
2,reverse,0,12,0.00,0
2,reverse,0,14,41.17,1
2,reverse,0,16,54.42,1
2,reverse,0,18,66.62,1
2,reverse,0,20,79.98,1
2,reverse,0,22,92.28,1
2,reverse,0,24,105.36,1
2,reverse,0,26,117.84,1
2,reverse,0,28,130.40,1
2,reverse,0,30,142.92,1
2,reverse,0,32,156.17,1
2,reverse,0,34,169.14,1
2,reverse,0,36,181.43,1
2,reverse,0,38,194.51,1
2,reverse,0,40,208.01,1
2,reverse,0,42,221.01,1
2,reverse,0,44,234.26,1
2,reverse,0,46,247.04,1
2,reverse,0,48,259.86,1
2,reverse,0,50,272.59,1
2,reverse,0,52,285.84,1
2,reverse,0,54,299.10,1
2,reverse,0,56,311.87,1
2,reverse,0,58,325.06,1
2,reverse,0,60,337.90,1
2,reverse,0,62,350.68,1
2,reverse,0,64,362.98,1
2,reverse,0,66,376.24,1
2,reverse,0,68,389.01,1
2,reverse,0,70,402.54,1
2,reverse,0,72,414.57,1
2,reverse,1,72,414.57,1
2,reverse,1,70,402.26,1
2,reverse,1,68,389.01,1
2,reverse,1,66,376.55,1
2,reverse,1,64,363.80,1
2,reverse,1,62,349.73,1
2,reverse,1,60,336.40,1
2,reverse,1,58,324.18,1
2,reverse,1,56,311.31,1
2,reverse,1,54,298.60,1
2,reverse,1,52,284.90,1
2,reverse,1,50,272.59,1
2,reverse,1,48,259.38,1
2,reverse,1,46,247.10,1
2,reverse,1,44,233.31,1
2,reverse,1,42,220.54,1
2,reverse,1,40,207.88,1
2,reverse,1,38,195.45,1
2,reverse,1,36,182.20,1
2,reverse,1,34,170.09,1
2,reverse,1,32,156.65,1
2,reverse,1,30,144.34,1
2,reverse,1,28,131.35,1
2,reverse,1,26,118.31,1
2,reverse,1,24,105.83,1
2,reverse,1,22,93.23,1
2,reverse,1,20,80.45,1
2,reverse,1,18,67.56,1
2,reverse,1,16,54.90,1
2,reverse,1,14,42.05,1
2,reverse,1,12,29.81,1
2,reverse,1,10,14.65,0
2,reverse,1,8,0.00,0
2,reverse,1,6,0.00,0
2,reverse,1,4,0.00,0
```

## Paste into `arduino_mini_base_controller/Config.h`

```cpp
const float FF_SLOPE_MM_S_PER_PWM_FWD[MOTOR_COUNT] = {6.4639f, 6.2343f};
const float FF_INTERCEPT_MM_S_FWD[MOTOR_COUNT] = {-44.31f, -44.78f};
const float FF_SLOPE_MM_S_PER_PWM_REV[MOTOR_COUNT] = {6.2624f, 6.4385f};
const float FF_INTERCEPT_MM_S_REV[MOTOR_COUNT] = {-44.38f, -49.07f};
```

After pasting, rebuild and reflash the base controller, then confirm the
board is running them:

```bash
bash scripts/build_arduino.sh arduino_mini_base_controller
python3 scripts/flash_arduino.py build/arduino_mini_base_controller/*.hex
python3 scripts/validate_motor_control.py --tests link,ladder
```

