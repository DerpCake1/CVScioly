## Project Overview
### Key Components
- **src/main.cpp**: Main entry point for the robot, button and mode control
- **src/config.cpp**: Parameters and movement sequences
- **lib/Travel**: High-level movement sequencing and control
- **lib/Robot**: Motor control and movement execution
- **lib/IMU**: IMU integration and angle calculation
- **lib/Logger**: serial monitor logging and LCD screen display
- **lib/JY901**: IMU library

### Workflow
1. main.cpp loads parameters and sequences
1. Travel manages sequence execution
1. Robot controls stepper motors for movement, and IMU for angle feedback


## Setup

1. Install Visual Studio Code
https://visualstudio.microsoft.com/

1. Install PlatformIO IDE extension on VS Code
https://docs.platformio.org/en/latest/integration/ide/vscode.html#installation

1. Windows(not required for Mac)
Install ESP32 CP210x Driver: https://randomnerdtutorials.com/install-esp32-esp8266-usb-drivers-cp210x-windows/


## Operation Modes

1. **Test**
    - The robot will move with the test sequence defined in `src/config.cpp`

1. **Dry Run** (setDryRun(true))
    - Simulated movements
    - Timed turns

1. **Track Run**
    - The robot will move with the track sequence defined in `src/config.cpp`

## Quick Start

1. Copy configuration:
   ```bash
   cp src/config.cpp.example src/config.cpp
   ```

1. Adjust parameters in `src/config.cpp`
    - See Tuning Guide below

1. Modify `testSequence` or `runSequence` in `src/config.cpp` to define the sequence to run

1. Build and upload:
   - Click the upload button in the platformio IDE

5. Press the mode button on the robot to select the mode, place the robot on the track and press the start button to start the run

## Tuning Guide - `src/config.cpp`

1. **Basic Setup**
   - Set correct WHEEL_DISTANCE:  distance(in mmm) between the 2 driving wheels
   - Enable IMU-based turns: useIMU = true
   - Disable IMU-based turns: useIMU = false - switch to step-based turns

1. **Calibration for straight movement**

   This is to ensure your robot moves in a straight line.

   Steps to test and calibrate:
    - Update `testSequence` in `src/config.cpp` with `.f(2000).b(2000)`
    - Place the robot on a flat surface. Mark its starting position and orientation
    - Run the test sequence in TEST mode
    - Check the robot's final position and orientation, and compare with the starting position and orientation.
    - Loosen the screws on the stepper motor brackets and adjust them to align the wheels.
    - Repeat the process until the robot moves staight.

1. **Calibration for turns**

   Basics:
   - `Corrected target angle = target angle * LEFT_TURN_COMPENSATION or RIGHT_TURN_COMPENSATION`
   - The robot control code uses the corrected target angle to calculate the actual angle to turn.
   - For each turn, the angle read from the IMU is displayed in the serial monitor and on the LCD screen. This angle value(from IMU) should be close to the corrected target angle.
   - When the angle read from the IMU is over `0.03` degree off the corrected target angle, the color of the angle displayed on the LCD screen will be red.

   How to test the turns and adjust the compensation:
     - Update `testSequence` in `src/config.cpp` with 8 x 90° left turns (e.g. `.l().l().l().l().l().l().l().l()`)
     - Place the robot on a flat surface and mark its starting position and orientation
     - Run the test sequence in TEST mode
     - Monitor the angles displayed on the LCD screen after each turn
     - After sequence completion, check if robot returned to starting position/orientation
     - If left turns are off:
       - Increase `LEFT_TURN_COMPENSATION` if turns are too small
       - Decrease `LEFT_TURN_COMPENSATION` if turns are too large
     - If right turns are off:
       - Increase `RIGHT_TURN_COMPENSATION` if turns are too small  
       - Decrease `RIGHT_TURN_COMPENSATION` if turns are too large
     - Repeat above steps until robot reliably returns to the starting position and orientation.
     - Repeat the same process for 90° right turns (e.g. `.r().r().r().r().r().r().r().r()`)

1. **General PID Tuning for turns**
    - Start with only KP
    - Increase until slight oscillation
    - Add KD to reduce overshoot
    - Add KI if needed for precision


1. **Dry Run Timing for turns**

    The DRY_RUN mode allows testing the timing of a sequence without physically turning the robot. In this mode, the robot pauses for predefined durations from `DRY_RUN_STOP_TIMES` instead of executing actual turns. This helps validate sequence timing before running on the track.

    How to test the turns and adjust the timing:
    - Update `testSequence` in `src/config.cpp` with 8 x 90° left turns (e.g. `.l().l().l().l().l().l().l().l()`), and `totalStopTime` set to `0`.
    - Run the test sequence in TEST mode with cable connected to the robot, open serial monitor to monitor the times for each turn
    - Calculate the average time for each turn: `average_time = total run time / 8`
    - Update the `DRY_RUN_STOP_TIMES` array in `src/config.cpp` with the average time for each turn
    - Repeat above steps for all 45, 90, 135, 180 degree turns

## Troubleshooting

## Competition

### Preparation
1. Copy this `Robot-pio` folder to a USB drive or SD card
1. Make sure you are able to open the `Robot-pio` folder from the USB drive or SD card in VS Code
1. Make sure you are able to build and upload the code to the robot from VS Code using the USB drive or SD card
1. Fully charge the batteries, batteries should remain seperated from the robot
1. Put the robot, batteries, USB cable, USB drive or SD card, tools, extra parts in a box

### Competition
#### Setup time
1. Plug the USB drive or SD card into the robot
1. Open VS Code and select the `Robot-pio` folder from your USB drive or SD card
1. Open `src/config.cpp`, modify the `runSequence` to the sequence you want to run
1. Hold the robot, press the mode button to select the "Dry Run" mode
1. Read the total run time from the serial monitor or the LCD screen, if the run time is below the target run time, modify the `totalStopTime` in `src/config.cpp`, which is the `target run time - the total run time from dry run`.
1. Do another dry run to check if the run time is close to the target run time, if not, adjust the `totalStopTime` in `src/config.cpp` again.
#### Track time
1. Aftert the first successful run, adjust the distance of the final forward or backward movements, to make the robot stop closer to the target point.
1. Adjust the `totalStopTime` in `src/config.cpp` if needed
