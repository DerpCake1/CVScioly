/*
 *   Basic example code for controlling a stepper without library
 *   CVHS RT 2025
 */

// define pins
// left motor
#define DIR1 26
#define STEP1 27
#define EN1 14

// right motor
#define DIR2 32
#define STEP2 33
#define EN2 25

// micro stepping - TMC2208
/*
 * MS1  MS2  STEPS
 * HIGH LOW  1/2
 * LOW  HIGH 1/4
 * LOW  LOW  1/8
 * HIGH HIGH 1/16
 */
#define STEPS 4

void setup()
{
    // Sets the all pins as Outputs
    pinMode(STEP1, OUTPUT);
    pinMode(DIR1, OUTPUT);
    pinMode(EN1, OUTPUT);
    pinMode(STEP2, OUTPUT);
    pinMode(DIR2, OUTPUT);
    pinMode(EN2, OUTPUT);

    // disable motor - send HIGH signal
    digitalWrite(EN1, HIGH);
    digitalWrite(EN2, HIGH);
    // delay 1 second
    delay(1000);

    // enable motors - sing LOW signal
    digitalWrite(EN1, LOW);
    digitalWrite(EN2, LOW);

    // delay 1 second
    delay(1000);
    // move in a particular direction
    digitalWrite(DIR1, HIGH);
    digitalWrite(DIR2, LOW);
    // Make (200 * STEPS) pulses for making one full cycle rotation
    for (int i = 0; i < 200 * STEPS; i++)
    {
        digitalWrite(STEP1, HIGH);
        digitalWrite(STEP2, HIGH);
        // by changing this time delay between the steps we can change the rotation speed
        delayMicroseconds(700);
        digitalWrite(STEP1, LOW);
        digitalWrite(STEP2, LOW);
        delayMicroseconds(700);
    }

    // One second delay
    delay(1000);

    // Change the rotations direction
    digitalWrite(DIR1, LOW);
    digitalWrite(DIR2, HIGH);
    // Make (400 * STEPS) pulses for making two full cycle rotation
    for (int i = 0; i < 400 * STEPS; i++)
    {
        digitalWrite(STEP1, HIGH);
        digitalWrite(STEP2, HIGH);
        delayMicroseconds(500);
        digitalWrite(STEP1, LOW);
        digitalWrite(STEP2, LOW);
        delayMicroseconds(500);
    }

    // One second delay
    delay(1000);

    // disable motors
    digitalWrite(EN1, HIGH);
    digitalWrite(EN2, HIGH);
}

void loop() {}