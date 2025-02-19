#ifndef CONFIG_H
#define CONFIG_H

#include "Commands.h"
#include <unordered_map>

// Declare external variables
extern const unsigned long totalStopTime;
extern const bool useIMU;

// Dry run configuration
extern const std::unordered_map<double, unsigned long> DRY_RUN_STOP_TIMES;

// Robot physical parameters
extern const double WHEEL_DISTANCE; // mm

// Turn compensation factors
extern const double LEFT_TURN_COMPENSATION;
extern const double RIGHT_TURN_COMPENSATION;

// PID control parameters
extern const double TURN_PID_KP;
extern const double TURN_PID_KI;
extern const double TURN_PID_KD;

extern CommandSequence runSequence;
extern CommandSequence testSequence;

#endif
