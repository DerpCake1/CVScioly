#include "config.h"

// IMU Configuration, set to false to disable IMU
const bool useIMU = true;

// Dry run configuration - angle to stop time mapping (values in milliseconds)
const std::unordered_map<double, unsigned long> DRY_RUN_STOP_TIMES = {
    {45, 970},  // 45°
    {90, 1250},  // 90°
    {135, 1900}, // 135°
    {180, 2200}  // 180°
};

// Robot physical parameters
const double WHEEL_DISTANCE = 155.8; // Distance between wheels in mm

// Turn compensation factors
const double LEFT_TURN_COMPENSATION = 90 / 90.0;  // Correction for left turns
const double RIGHT_TURN_COMPENSATION = 90/ 90.0; // Correction for right turns

// PID control parameters
const double TURN_PID_KP = 161;
const double TURN_PID_KI = 0.0;
const double TURN_PID_KD = 0.0;
const double h = sqrt(2);

// test configuration
CommandSequence testSequence = CommandSequence()
                                .r()
                                .r()
                                .r()
                                .r();
                        
                                //    .r()
                                //    .r()
                                //    .r()
                                //    .stop(3000)
                                //    .r()
                                //    .r()
                                //    .r()
                                //    .r()
                                //    .l()
                                //    .l()
                                //    .l()
                                //    .l()
                                //    .stop(3000)
                                //    .l()
                                //    .l()
                                //    .l()
                                //    .l()
                                //    .stop(5000);
                                   

// competition configuration
// Define total stop time
const unsigned long totalStopTime = 15000; // ms
// example run - gullso
CommandSequence runSequence = CommandSequence()
                                .f(42)
                                .l(45)
                                .f(750*h)
                                .r()
                                .f(1000*h)
                                .l(45)
                                .f(750)
                                .l()
                                .f(1300)
                                .b(300)
                                .l()
                                .f(800)
                                .b(300)
                                .l()
                                .f(300)
                                .b(800)
                                .r()
                                .f(1000+300)
                                .b(300+1000)
                                .l()
                                .f(500-42);

                                // .f(42)
                                // .l(45)
                                // .f((750+500)*h)
                                // .b(500*h)
                                // .l()
                                // .f(500*h)
                                // .r()
                                // .f(300*h)
                                // .b(300*h)
                                // .r()
                                // .f(500*h)
                                // .r(45)
                            
                                // .f(1000+250)
                                // .r()
                                // .f(300)
                                // .b(300+1000)
                                // .r()
                                // .f(2000-42);






                                //   .f(750 + 42)
                                //   .r()
                                //   .f(500)
                                //   .r()
                                //   .f(300)
                                //   .b(300)
                                //   .l()
                                //   .f(500)
                                //   .l()
                                //   .f(500)
                                //   .r()
                                //   .f(500)
                                //   .l()
                                //   .f(300)
                                //   .b(300)
                                //   .l()
                                //   .f(500)
                                //   .l()
                                //   .f(500)
                                //   .r()
                                //   .f(1000)
                                //   .r()
                                //   .f(500)
                                //   .r()
                                //   .f(300)
                                //   .b(300)
                                //   .r()
                                //   .f(1000)
                                //   .r()
                                //   .f(500)
                                //   .r()
                                //   .f(300)
                                //   .b(300)
                                //   .r()
                                //   .f(500)
                                //   .l()
                                //   .f(1000)
                                //   .r()
                                //   .f(500)
                                //   .l(98)
                                //   .f(570-42);

