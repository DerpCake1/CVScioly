#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <AccelStepper.h>
#include "Logger.h"

// define pins
// left stepper
#define DIR1 26
#define STEP1 27
#define EN1 14

// right stepper
#define DIR2 32
#define STEP2 33
#define EN2 25

// stepper parameters
#define STEPS_PER_REVOLUTION 200
// micro stepping - TMC2208
/*
 * MS1  MS2  STEPS
 * HIGH LOW  1/2
 * LOW  HIGH 1/4
 * LOW  LOW  1/8
 * HIGH HIGH 1/16 - use this
 */
#define MICRO_STEPS 16
// speed
// move
#define MOVE_SPEED (400 * MICRO_STEPS)
#define MOVE_ACCEL (MOVE_SPEED * 1.5)

// turn
#define TURN_SPEED (MOVE_SPEED * 0.5)
#define TURN_ACCEL (TURN_SPEED * 1.5)

// Robot parameters
// 200mm wheel circumference
#define WHEEL_CIRCUMFERENCE 200
// 140mm wheel distance - distance between 2 wheels
#define WHEEL_DISTANCE 140

// movement timeout - 10 seconds
#define MOVEMENT_TIMEOUT 10000
// max distance - 1000mm
#define MAX_DISTANCE 1000

// max angle - 360 degrees
#define MAX_ANGLE 360.0

class Robot
{
private:
  AccelStepper _leftStepper;
  AccelStepper _rightStepper;

public:
  Robot()
      : _leftStepper(AccelStepper::DRIVER, STEP1, DIR1),
        _rightStepper(AccelStepper::DRIVER, STEP2, DIR2)
  {
    // init stepper EN pins
    pinMode(EN1, OUTPUT);
    pinMode(EN2, OUTPUT);
    turnOffSteppers();
  }

  void turnOnSteppers()
  {
    logger.info("Turning on steppers");
    digitalWrite(EN1, LOW);
    digitalWrite(EN2, LOW);
  }

  void turnOffSteppers()
  {
    logger.info("Turning off steppers");
    digitalWrite(EN1, HIGH);
    digitalWrite(EN2, HIGH);
    _leftStepper.setCurrentPosition(0);
    _rightStepper.setCurrentPosition(0);
  }

  void move(long distance)
  {
    if (abs(distance) > MAX_DISTANCE)
    {
      logger.error("Distance exceeds maximum limit");
      return;
    }
    long target_pos = distance * MICRO_STEPS;
    logger.info("Moving to %dmm (%d steps)", distance, target_pos);

    _leftStepper.setCurrentPosition(0);
    _rightStepper.setCurrentPosition(0);

    _leftStepper.setMaxSpeed(MOVE_SPEED);
    _rightStepper.setMaxSpeed(MOVE_SPEED);
    _leftStepper.setAcceleration(MOVE_ACCEL);
    _rightStepper.setAcceleration(MOVE_ACCEL);

    _leftStepper.moveTo(target_pos);
    _rightStepper.moveTo(-target_pos);

    unsigned long startTime = millis();
    while (_leftStepper.currentPosition() != target_pos)
    {
      if (millis() - startTime > MOVEMENT_TIMEOUT)
      {
        logger.error("Movement timeout");
        break;
      }
      _leftStepper.run();
      _rightStepper.run();
    }
  }

  void turn(double angle = 90.0)
  {
    if (abs(angle) > MAX_ANGLE)
    {
      logger.error("Angle exceeds maximum limit");
      return;
    }
    logger.info("Turning %D degrees", angle);

    // Calculate steps for turning: (angle/360°) * (π*wheel_distance) * (steps_per_mm)
    double distance_per_wheel = (angle / 360.0) * (PI * WHEEL_DISTANCE);
    long steps = distance_per_wheel * (STEPS_PER_REVOLUTION * MICRO_STEPS / WHEEL_CIRCUMFERENCE);

    _leftStepper.setCurrentPosition(0);
    _rightStepper.setCurrentPosition(0);

    _leftStepper.setMaxSpeed(TURN_SPEED);
    _rightStepper.setMaxSpeed(TURN_SPEED);
    _leftStepper.setAcceleration(TURN_ACCEL);
    _rightStepper.setAcceleration(TURN_ACCEL);

    _leftStepper.moveTo(steps);
    _rightStepper.moveTo(steps);

    unsigned long startTime = millis();
    while (_leftStepper.currentPosition() != steps)
    {
      if (millis() - startTime > MOVEMENT_TIMEOUT)
      {
        logger.error("Turn timeout");
        break;
      }
      _leftStepper.run();
      _rightStepper.run();
    }
  }

  void stop(unsigned long duration)
  {
    logger.info("Stopping %u ms", duration);
    delay(duration);
  }
};

#endif