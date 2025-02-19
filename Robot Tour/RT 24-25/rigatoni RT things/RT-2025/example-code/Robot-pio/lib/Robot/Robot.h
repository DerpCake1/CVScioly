#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <AccelStepper.h>
#include <PID_v1.h>
#include "Logger.h"
#include "IMU.h"
#include "config.h"

// Hardware Configuration
#define LEFT_STEPPER_DIR_PIN 26
#define LEFT_STEPPER_STEP_PIN 27
#define LEFT_STEPPER_EN_PIN 14

#define RIGHT_STEPPER_DIR_PIN 32
#define RIGHT_STEPPER_STEP_PIN 33
#define RIGHT_STEPPER_EN_PIN 25

#define LASER1 16
#define LASER2 17

// Wheel circumference in mm
#define WHEEL_CIRCUMFERENCE 200.0

// Stepper Configuration
#define STEPS_PER_REVOLUTION 200
#define MICRO_STEPS 16 // Using 1/16 microstepping

// Movement Parameters
#define MOVE_SPEED (400 * MICRO_STEPS)
#define MOVE_ACCEL (MOVE_SPEED * 1.5)

// turn speed and acceleration, do not change these values
#define TURN_SPEED (200 * MICRO_STEPS)
#define TURN_ACCEL (TURN_SPEED * 2.0)

// minimum stop time after turning, do not change this value
#define MIN_STOP_TIME 50 // ms

// PID Parameters for turning
#define TURN_PID_MIN_SPEED (1 * MICRO_STEPS)
#define TURN_PID_SAMPLE_TIME 10 // ms

// Safety Limits
#define MOVEMENT_TIMEOUT 8000 // ms
#define MAX_DISTANCE 2500     // mm
#define MAX_ANGLE 360.0       // degrees

// IMU Task Configuration
#define IMU_TASK_CORE 0
#define IMU_TASK_PRIORITY 0
#define IMU_TASK_STACK_SIZE 10000

class Robot
{
private:
    // Hardware Components
    AccelStepper _leftStepper;
    AccelStepper _rightStepper;
    IMU _imu;

    // State Variables
    bool _useIMU = true;
    volatile bool resetAngle = false;
    volatile bool imuOn = false;
    static volatile double currentAngle;
    static TaskHandle_t imuTaskHandle;
    static SemaphoreHandle_t imuMutex;

    // IMU Task Management
    static void imuTask(void *parameter);
    void startIMUTask();

    // Movement Calculations
    long calculateTurnSteps(double angle);
    bool checkMovementLimits(long distance, double angle);
    void configureSteppers(long speed, long acceleration);

    // Movement Implementation Details
    void executeStepperMovement(long leftSteps, long rightSteps);
    void executeIMUGuidedTurn(double targetAngle, int direction);
    void executeStepBasedTurn(double angle);

public:
    // Constructor & Destructor
    Robot(bool useIMU = true)
        : _leftStepper(AccelStepper::DRIVER, LEFT_STEPPER_STEP_PIN, LEFT_STEPPER_DIR_PIN),
          _rightStepper(AccelStepper::DRIVER, RIGHT_STEPPER_STEP_PIN, RIGHT_STEPPER_DIR_PIN),
          _useIMU(useIMU)
    {
        pinMode(LEFT_STEPPER_EN_PIN, OUTPUT);
        pinMode(RIGHT_STEPPER_EN_PIN, OUTPUT);
        pinMode(LASER1, OUTPUT);
        pinMode(LASER2, OUTPUT);
        turnOffSteppers();
        currentAngle = 0.0;

        // Create mutex for IMU data access
        if (imuMutex == NULL)
        {
            imuMutex = xSemaphoreCreateMutex();
        }
    }

    ~Robot()
    {
        if (imuMutex != NULL)
        {
            vSemaphoreDelete(imuMutex);
        }
    }

    // Basic Control
    void turnOnSteppers();
    void turnOffSteppers();
    void setUseIMU(bool useIMU);
    void startIMU();

    // Movement Commands
    void move(long distance);
    void turn(double angle);
    void stop(unsigned long duration);

    // IMU-based Movement
    void turnWithIMU(double angle = 90.0);
    void turnWithoutIMU(double angle = 90.0);

    // Laser Functions
    void startLasers();
    void stopLasers();
};

// Member variables
volatile double Robot::currentAngle = 0.0;
TaskHandle_t Robot::imuTaskHandle = NULL;
SemaphoreHandle_t Robot::imuMutex = NULL;

// Implementation of private helper methods
inline long Robot::calculateTurnSteps(double angle)
{
    double distance_per_wheel = (angle / 360.0) * (PI * WHEEL_DISTANCE);
    return round(distance_per_wheel * (STEPS_PER_REVOLUTION * MICRO_STEPS / WHEEL_CIRCUMFERENCE));
}

inline bool Robot::checkMovementLimits(long distance, double angle)
{
    if (abs(distance) > MAX_DISTANCE)
    {
        logger.error("Distance exceeds maximum limit!!!");
        return false;
    }
    if (abs(angle) > MAX_ANGLE)
    {
        logger.error("Angle exceeds maximum limit!!!");
        return false;
    }
    return true;
}

inline void Robot::configureSteppers(long speed, long acceleration)
{
    _leftStepper.setMaxSpeed(speed);
    _rightStepper.setMaxSpeed(speed);
    _leftStepper.setAcceleration(acceleration);
    _rightStepper.setAcceleration(acceleration);
}

// Implementation of public interface
void Robot::turnOnSteppers()
{
    logger.info("Turning on steppers");
    digitalWrite(LEFT_STEPPER_EN_PIN, LOW);
    digitalWrite(RIGHT_STEPPER_EN_PIN, LOW);
}

void Robot::turnOffSteppers()
{
    logger.info("Turning off steppers");
    digitalWrite(LEFT_STEPPER_EN_PIN, HIGH);
    digitalWrite(RIGHT_STEPPER_EN_PIN, HIGH);
    _leftStepper.setCurrentPosition(0);
    _rightStepper.setCurrentPosition(0);
}

void Robot::setUseIMU(bool useIMU)
{
    _useIMU = useIMU;
}

void Robot::startIMU()
{
    if (!_useIMU)
        return;

    logger.info("Starting IMU");
    _imu.Start();
    _imu.ResetAngle();
    currentAngle = 0.0;
    imuOn = false;
    startIMUTask();
}

// IMU Task Management Methods
void Robot::startIMUTask()
{
    if (imuTaskHandle != NULL)
    {
        return; // Task already running
    }

    xTaskCreatePinnedToCore(
        imuTask,             // Task function
        "IMU_Task",          // Task name
        IMU_TASK_STACK_SIZE, // Stack size
        this,                // Task parameters
        IMU_TASK_PRIORITY,   // Priority
        &imuTaskHandle,      // Task handle
        IMU_TASK_CORE        // Core ID
    );
    logger.info("IMU task started on core %d", IMU_TASK_CORE);
}

void Robot::imuTask(void *parameter)
{
    Robot *robot = (Robot *)parameter;

    while (true)
    {
        xSemaphoreTake(robot->imuMutex, portMAX_DELAY);
        if (robot->resetAngle)
        {
            currentAngle = 0.0;
            robot->_imu.ResetAngle();
            robot->resetAngle = false;
        }
        if (robot->imuOn)
        {
            currentAngle = robot->_imu.GetAngle();
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        xSemaphoreGive(robot->imuMutex);
    }
}

// Movement Implementation Methods
void Robot::executeStepperMovement(long leftSteps, long rightSteps)
{
    _leftStepper.setCurrentPosition(0);
    _rightStepper.setCurrentPosition(0);

    _leftStepper.moveTo(leftSteps);
    _rightStepper.moveTo(rightSteps);

    unsigned long startTime = millis();
    while (_leftStepper.currentPosition() != _leftStepper.targetPosition())
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

void Robot::move(long distance)
{
    if (!checkMovementLimits(distance, 0))
        return;

    unsigned long startTime = millis();
    long steps = distance * MICRO_STEPS;
    logger.info("Moving %d mm(%l steps)", distance, steps);

    configureSteppers(MOVE_SPEED, MOVE_ACCEL);
    executeStepperMovement(steps, -steps); // Right motor is inverted
    delay(MIN_STOP_TIME);
    unsigned long totalTime = millis() - startTime;
    logger.info("Move time: %u ms", totalTime);
}

void Robot::turn(double angle)
{
    unsigned long startTime = millis();
    if (_useIMU)
    {
        turnWithIMU(angle);
    }
    else
    {
        turnWithoutIMU(angle);
    }
    unsigned long totalTime = millis() - startTime;
    logger.info("Turn time: %u ms", totalTime);
}

void Robot::turnWithIMU(double angle)
{
    if (!_useIMU)
    {
        logger.warn("IMU is disabled, using non-IMU turn");
        turnWithoutIMU(angle);
        return;
    }

    if (!checkMovementLimits(0, angle))
        return;

    logger.info("Turning %D degrees with IMU", angle);
    // reset the angle to 0
    // xSemaphoreTake(imuMutex, portMAX_DELAY);
    resetAngle = true;
    currentAngle = 0.0;
    imuOn = true;
    // xSemaphoreGive(imuMutex);
    delay(5);
    // correct the angle for the left and right turns
    double correctedAngle = angle * (angle < 0 ? LEFT_TURN_COMPENSATION : RIGHT_TURN_COMPENSATION);
    logger.info("Corrected angle: %D degrees", correctedAngle);

    double targetAngle = abs(correctedAngle);
    long steps = calculateTurnSteps(targetAngle);
    int direction = (angle > 0) ? -1 : 1;

    // stage 1, move to 92% of target angle
    double stage1Percent = 0.92;
    long stage1Steps = steps * stage1Percent;
    logger.info("Stage 1: Turning to %D degrees(%l steps)", correctedAngle * stage1Percent, stage1Steps);

    configureSteppers(TURN_SPEED, TURN_ACCEL);

    _leftStepper.setCurrentPosition(0);
    _rightStepper.setCurrentPosition(0);

    _leftStepper.move(steps * direction);
    _rightStepper.move(steps * direction);

    unsigned long startTime = millis();
    while (_leftStepper.currentPosition() != stage1Steps * direction)
    {
        if (millis() - startTime > MOVEMENT_TIMEOUT)
        {
            logger.error("Movement timeout");
            break;
        }
        _leftStepper.run();
        _rightStepper.run();
    }
    double stage1Speed = _leftStepper.speed();

    logger.info("Stage 2: PID control to %D degrees", targetAngle);
    logger.info("Stage 2 start speed: %D", stage1Speed);

    // Initialize PID variables
    double pidInput = currentAngle;
    double pidOutput = abs(stage1Speed);
    double pidSetpoint = targetAngle;

    // Create and configure PID with adjusted parameters
    PID pid(&pidInput, &pidOutput, &pidSetpoint,
            TURN_PID_KP,
            TURN_PID_KI,
            TURN_PID_KD,
            DIRECT);
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(TURN_PID_MIN_SPEED, abs(stage1Speed));
    pid.SetSampleTime(TURN_PID_SAMPLE_TIME);

    // Adjusted thresholds
    const double ANGLE_THRESHOLD = 0.04;
    double lastAngle = pidInput;

    unsigned long count = 0;
    unsigned long lastLog = millis();
    unsigned long aCount = 0;
    while (true)
    {
        if (millis() - startTime > MOVEMENT_TIMEOUT)
        {
            logger.error("Turn timeout in PID control");
            break;
        }

        pidInput = currentAngle;
        // Get current angle and check if we're within threshold
        double angleError = targetAngle - pidInput;
        if (angleError <= ANGLE_THRESHOLD)
        {
            break;
        }

        if (lastAngle != pidInput)
        {
            lastAngle = pidInput;
            aCount++;
        }

        // Update PID input and compute new output
        pid.Compute();

        // Scale down speed as we approach target
        // Start scaling when within 2.2 degrees
        double speedScale = _min(1.0, angleError / 2.2);
        double adjustedSpeed = _max(TURN_PID_MIN_SPEED, pidOutput * speedScale);

        // Apply speeds
        _leftStepper.setSpeed(direction * adjustedSpeed);
        _rightStepper.setSpeed(direction * adjustedSpeed);

        // Run motors
        _leftStepper.runSpeed();
        _rightStepper.runSpeed();

        if (millis() - lastLog > 50)
        {
            logger.info("Angle: %D, Speed: %D, error: %D",
                        pidInput, adjustedSpeed * direction, angleError);
            lastLog = millis();
        }

        count++;
    }

    // Immediately stop motors
    _leftStepper.setSpeed(0);
    _rightStepper.setSpeed(0);
    _leftStepper.runSpeed();
    _rightStepper.runSpeed();

    unsigned long imuCount = _imu.GetCount();

    delay(MIN_STOP_TIME);
    double finalAngle = currentAngle;
    imuOn = false;
    // error too big
    if (abs(finalAngle - targetAngle) > 0.03)
    {
        logger.lcdSet(COLOR_RED);
        logger.lcdPrintf("Angle:\n%.2f", finalAngle);
        logger.error("Final Angle: %D, steps: %l, count: %u, angle change count: %u, imu count: %u",
                     finalAngle, _leftStepper.currentPosition(), count, aCount, imuCount);
    }
    else
    {
        logger.lcdSet(COLOR_CYAN);
        logger.lcdPrintf("Angle:\n%.2f", finalAngle);
        logger.info("Final Angle: %D, steps: %l, count: %u, angle change count: %u, imu count: %u",
                    finalAngle, _leftStepper.currentPosition(), count, aCount, imuCount);
    }
}

void Robot::turnWithoutIMU(double angle)
{
    if (!checkMovementLimits(0, angle))
        return;

    long steps = calculateTurnSteps(angle);
    logger.info("Turning %D degrees(%ld steps) without IMU", angle, steps);

    configureSteppers(TURN_SPEED, TURN_ACCEL);
    executeStepperMovement(-steps, -steps);
    delay(MIN_STOP_TIME);
}

void Robot::stop(unsigned long duration)
{
    logger.info("Stopping for %u ms", duration);
    delay(duration);
}

void Robot::startLasers()
{
    digitalWrite(LASER1, HIGH);
    digitalWrite(LASER2, HIGH);
}

void Robot::stopLasers()
{
    digitalWrite(LASER1, LOW);
    digitalWrite(LASER2, LOW);
}

#endif
