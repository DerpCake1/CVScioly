#include <AccelStepper.h>     // Accel Stepper libray

// define pins
// left stepper
#define DIR1 26
#define STEP1 27
#define EN1 14

// right stepper
#define DIR2 32
#define STEP2 33
#define EN2 25

// micro stepping - TMC2208
/*
 * MS1  MS2  STEPS
 * HIGH LOW  1/2
 * LOW  HIGH 1/4
 * LOW  LOW  1/8
 * HIGH HIGH 1/16 - use this
 */
#define MICRO_STEPS 16

#define MOVE_SPEED (400 * MICRO_STEPS)
#define MOVE_ACCEL (MOVE_SPEED * 1.2)

AccelStepper stepper1(AccelStepper::DRIVER, STEP1, DIR1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP2, DIR2);

void initSteppers() {
      // Sets the pins as Outputs
    pinMode(EN1, OUTPUT);
    pinMode(EN2, OUTPUT);
}

void enableSteppers() {
  digitalWrite(EN1, LOW);
  digitalWrite(EN2, LOW);
}

void disableSteppers() {
  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);
}

void forward(long distance=500) {
  long target_pos = distance * MICRO_STEPS;

  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);

  stepper1.setMaxSpeed(MOVE_SPEED);
  stepper2.setMaxSpeed(MOVE_SPEED);
  stepper1.setAcceleration(MOVE_ACCEL);
  stepper2.setAcceleration(MOVE_ACCEL);

  stepper1.moveTo(target_pos);
  stepper2.moveTo(-target_pos);

  while (stepper1.currentPosition() != target_pos) {
    stepper1.run();
    stepper2.run();
  }

}

void backward(long distance=500) {
  forward(-distance);
}

void setup()
{
  initSteppers();
  delay(1000);
  enableSteppers();
  delay(1000);
  forward(1000);
  delay(1000);
  forward(-1000);
  disableSteppers();
}

void loop()
{

}
