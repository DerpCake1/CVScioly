#include <Encoder.h>
#include <PID_v1.h>

// ===== PINs =================
// motor A - left motor
#define ENA 5 //PWM
#define INT1 4
#define INT2 7
// encoder
#define ENCA1 2 // HW interrupt - encoder channel A
#define ENCA2 10 // encoder channel B

//motor B - right motor
#define ENB 6 //PWM
#define INT3 8
#define INT4 9
// encoder
#define ENCB1 3 // HW interrupt - encoder channel A
#define ENCB2 11 // encoder channel B

// ===== Move function aliases
#define l(x) left(x)
#define r(x) right(x)
#define f(x) forward(x)
#define b(x) backward(x)

// ====== modify here =========
// uncomment #define DO_NOT_USE_ENCODER_LIB for MC520/MG513 180 RPM/60:1 motors
// #define DO_NOT_USE_ENCODER_LIB

// motor
// motor PWM
#define MIN_SPEED 5
#define MAX_SPEED 250
#define MOVE_SPEED 225 // motor A - left
#define ROTATE_SPEED 190 // motor A - left

// Encoder
// PPR - Pulses per revolution of the motor shaft
//  n20 6v 75 RPM motor, ppr 7,  gear ratio - 210:1, cpr = 28, CPRW 2x resolution: 2940, 4x resolution: 5880
//  JGB37-520 76 RPM motor, ppr 11, gear ratio - 131:1, cpr = 44, CPRW 2x resolution: 2882, CPR 4x resolution: 5764
//  JGB37-520 110 RPM motor, ppr 11, gear ratio - 90:1, cpr 44, CPRW 2x resolution: 1980, CPR 4x resolution: 3960
//  MC520/MG513 180 RPM motor, ppr 13,  gear ratio - 60:1, cpr = 52, CPRW 2x resolution: 1560, 4 x resolution: 3120
#define PPR 11
// Gear ratio
#define GEAR_RATIO 90

// D - wheel diameter in mm
// All teams - 68 mm wheel, 67.6 mm
// n20 robot - 33.4 mm
#define WHEEL_D 63.662
#define WHEEL_C 200
// R - center to the wheel
// n20 robot: 60.8
// Onion 2WD - 82.5 norm = 85, 77.5
#define ROBOT_R 76.5

// PID control
// Onion 2WD,  0.35,0,0/3.0,0.02,0.02
// good values 0.21,0,0
#define KP_A 0.35
#define KI_A 0
#define KD_A 0

#define KP_B 3.0
#define KI_B 0.02
#define KD_B 0.02

// debug message interval(in ms)
// Note this affects the PID control
#define DEBUG_INTERVAL 250
//==============================

// Encoder - do not modify
// CPR - Counts per revolution of the motor shaft
#ifndef DO_NOT_USE_ENCODER_LIB
// Quadrature encoder counts per revolution of the motor shaft
// with encoder lib
#define CPR (PPR * 4)
#else
#define CPR (PPR * 2)
#endif
// Counts per revolution of the wheels
#define CPRW (CPR * GEAR_RATIO)
#define MAX_STOP_TIME 2000

//======= global variables =====
// ms range 0 - 2800 ms.
// set this in the track run functions
unsigned long stop_time = 0; 

bool debug = true;
bool run_test = false;

double pwm_a = 0.0;
double pwm_b = 0.0;
double pwm_b_dt = 0.0;

unsigned main_count = 0; // pwm control loop count

// encoder
// pusle count
#ifndef DO_NOT_USE_ENCODER_LIB
double count_a = 0.0;
double count_b = 0.0;
#else
volatile double count_a = 0.0;
volatile double count_b = 0.0;
#endif

double count_a_error = 0.0;
double count_b_error = 0.0;
double pid_b_setpoint = 0.0;

double target_count_a = 0.0;
double target_count_b = 0.0;
double last_count_a = -999.0;
double last_count_b = -999.0;

#ifndef DO_NOT_USE_ENCODER_LIB
Encoder enc_a(ENCA1, ENCA2);
Encoder enc_b(ENCB1, ENCB2);
#endif

// PID
// pid A -  Position control (pulse count)
PID pid_a(&count_a, &pwm_a, &target_count_a, KP_A, KI_A, KD_A, DIRECT);

// pid B - follower, sync with encoder A
PID pid_b(&count_b_error, &pwm_b_dt, &pid_b_setpoint, KP_B, KI_B, KD_B, DIRECT);

// time stamps
unsigned long t_started; // time when the robot starts moving

// ======================

// convert distance in mm to pulse count
double distance2Counts(double distance) {
  return distance * CPRW / WHEEL_C;
}
// convert angular degree to pulse count - for robot in place rotation
double angle2Counts(double degree) {
  return distance2Counts(2.0 * PI * ROBOT_R * (degree / 360.0));
}

void setupMotors() {

	// motors
	pinMode(ENA, OUTPUT);
	pinMode(INT1, OUTPUT);
	pinMode(INT2, OUTPUT);
	pinMode(ENB, OUTPUT);
	pinMode(INT3, OUTPUT);
	pinMode(INT4, OUTPUT);

  pinMode(ENCA1, INPUT);
	pinMode(ENCA2, INPUT);
  pinMode(ENCB1, INPUT);
	pinMode(ENCB2, INPUT);
  // button
  pinMode(A0, INPUT);

  // PID
  pid_a.SetMode(AUTOMATIC);
  pid_a.SetOutputLimits(-MAX_SPEED, MAX_SPEED);
  pid_a.SetSampleTime(10);

  pid_b.SetMode(AUTOMATIC);
  pid_b.SetOutputLimits(-MAX_SPEED, MAX_SPEED);
  pid_b.SetSampleTime(10);

#ifdef DO_NOT_USE_ENCODER_LIB
  attachInterrupt(digitalPinToInterrupt(ENCA1), readEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB1), readEncoderB, CHANGE);
#endif

}

#ifdef DO_NOT_USE_ENCODER_LIB
void readEncoderA(){
  if(digitalRead(ENCA1) == HIGH){
    if(digitalRead(ENCA2) == HIGH){
      count_a --;
    } else {
      count_a ++;
    }
  } else {
    if(digitalRead(ENCA2) == LOW){
      count_a --;
    } else {
      count_a ++;
    }
  }
}

void readEncoderB(){
  if(digitalRead(ENCB1) == HIGH){
    if(digitalRead(ENCB2) == HIGH){
      count_b ++;
     } else {
      count_b --;
     }
   } else {
    if(digitalRead(ENCB2) == LOW){
      count_b ++;
    } else {
      count_b --;
    }
  }
}
#endif

void updateEncoders() {
#ifndef DO_NOT_USE_ENCODER_LIB
  count_a = -enc_a.read();
  count_b = enc_b.read();
#endif
}

void printDebugMessages(bool force = false) {
  if (force || (last_count_a != count_a || last_count_b != count_b)) {
    Serial.print("target_count_a:");
    Serial.print(abs(target_count_a));
    Serial.print(" target_count_b:");
    Serial.print(abs(target_count_b));
    Serial.print(" pwm_a:");
    Serial.print((pwm_a * 10));
    Serial.print(" pwm_b:");
    Serial.print((pwm_b * 10));
    Serial.print(" pwm_b_dt:");
    Serial.print((pwm_b_dt * 10));
    Serial.print(" count_a:");
    Serial.print(abs(count_a));
    Serial.print(" count_b:");
    Serial.print(abs(count_b));
    Serial.print(" count_a_error:");
    Serial.print((count_a_error));
    Serial.print(" count_b_error:");
    Serial.print((count_b_error));
    Serial.print(" count:");
    Serial.print(main_count);
    Serial.println();
    last_count_a = count_a;
    last_count_b = count_b;
  }
}

void setMotors(double a, double b) {

  digitalWrite(INT1, (a >= 0) ? LOW : HIGH);
  digitalWrite(INT2, (a <= 0) ? LOW : HIGH);

  digitalWrite(INT3, (b >= 0) ? LOW : HIGH);
  digitalWrite(INT4, (b <= 0) ? LOW : HIGH);

  analogWrite(ENA, abs(round(a)));
  analogWrite(ENB, abs(round(b)));
}

void stop() {
  // stop
  digitalWrite(INT1, LOW);
  digitalWrite(INT2, LOW);
  digitalWrite(INT3, LOW);
  digitalWrite(INT4, LOW);
  pwm_a = 0.0;
  pwm_b = 0.0;
  printDebugMessages();
  Serial.println("Stopped.");
}

void move() {

  int dir_a, dir_b;

  updateEncoders();

  // directions, 1 forward, -1 backward
  dir_a = target_count_a > count_a ? 1 : -1;
  dir_b = target_count_b > count_b ? 1 : -1;

  // set PWM limits for PID controllers
  // pid_a
  if (dir_a == 1) {
    if (dir_a == dir_b) {
      pid_a.SetOutputLimits(MIN_SPEED, MOVE_SPEED);
    } else {
      pid_a.SetOutputLimits(MIN_SPEED, ROTATE_SPEED);
    }
  } else {
    if (dir_a == dir_b) {
      pid_a.SetOutputLimits(-MOVE_SPEED, -MIN_SPEED);
    } else {
      pid_a.SetOutputLimits(-ROTATE_SPEED, -MIN_SPEED);
    }
  }
  // pid_b
  if (dir_a == dir_b) {
    pid_b.SetOutputLimits(-(MAX_SPEED - MOVE_SPEED), (MAX_SPEED - MOVE_SPEED));
  } else {
    pid_b.SetOutputLimits(-(MAX_SPEED - ROTATE_SPEED), (MAX_SPEED - ROTATE_SPEED));
  }

  bool break_check = false;
  double error = 0.0;
  double last_error = 0.0;

  unsigned long now;
  unsigned long t = millis(); // start time for this move
  unsigned long t1 = t; // for debug messages
  unsigned long t_break = t; // for breaking control loop

  // PID control loop
  while (true) {
    
    main_count ++; // control loop count
    now = millis(); // time stamp
    updateEncoders();

    // when the count_a has stopped changing for over 100 ms, we consider the robot has approached the target
    error = abs(target_count_a - count_a);
    if (error == last_error) {
      if (break_check && now - t_break > 100) {
        printDebugMessages(true);
        break;
      }
      if (!break_check) {
        break_check = true;
        t_break = now;
      }
    } else {
      t_break = now;
      last_error = error;
      break_check = false;
    }

    count_b_error = dir_a == dir_b ? count_b - count_a : count_b + count_a;
    count_a_error = target_count_a - count_a;

    pid_a.Compute();
    pid_b.Compute();

    pwm_b = dir_a == dir_b ? pwm_a + pwm_b_dt : -pwm_a + pwm_b_dt;
    pwm_b = constrain(pwm_b, -MAX_SPEED, MAX_SPEED);

    // set motors
    setMotors(pwm_a, pwm_b);

    // debug, print debug message every DEBUG_INTERNVAL ms
    if (debug && now - t1 >= DEBUG_INTERVAL) {
      printDebugMessages();
      t1 = now;
    }
  
  } // end control loop

  // stop
  stop();
  now = millis(); // time stamp
  Serial.print("move_time:");
  Serial.print((now - t)/1000.0);
  Serial.print(" total_time:");
  Serial.println((now - t_started)/1000.0);
  Serial.println();
  delay(min(stop_time, MAX_STOP_TIME));
  reset();
}

void reset() {
#ifndef DO_NOT_USE_ENCODER_LIB
  enc_a.write(0);
  enc_b.write(0);
#else
  count_a = 0.0;
  count_b = 0.0;
#endif
  updateEncoders();
  // reset target counts
  target_count_a = 0.0;
  target_count_b = 0.0;
  // reset PID control loop count
  main_count = 0;
}

void left(double degree = 86) { // 91
  Serial.print("Turning:");
  Serial.println(-degree);
  target_count_a = angle2Counts(-degree);
  target_count_b = angle2Counts(degree);
  move();
}

void right(double degree = 85.5) { // 89
  left(-degree);
}

void forward(double distance) {
  Serial.print("Moving:");
  Serial.println(distance);
  target_count_a = distance2Counts(distance);
  target_count_b = distance2Counts(distance);
  move();
}

void backward(double distance) {
  forward(-distance);
}

void startButton() {
  // wait for start button
  Serial.println();
  printDebugMessages(true);
  Serial.println("Press button once to start a run, twice to test movements.");
  printDebugMessages(true);

  int button_count = 0;

  // one button push
  while (digitalRead(A0) == LOW) {};
  while (digitalRead(A0) == HIGH) {};

  button_count ++;
  // 300ms interval
  delay(300);

  // another button push
  unsigned long t = millis();
  while(millis() - t < 2700) {
      if (digitalRead(A0) == HIGH) {
        button_count ++;
        break;
      }
  }
  if (button_count > 1) {
    run_test = true;
    Serial.println("Starting a movement test in 3 seconds.");
    delay(3000);
  } else {
    run_test = false;
    Serial.println("Starting a track run.");
  }
}

// ======== Test =============
void test() {
  stop_time = 4000;
  
  for (int i = 0; i < 4; i++) {
  r();
   }

  for (int i = 0; i < 4; i++) {
  l();
   }
  
  // f(1000);
  // f(500);
  // b(500);
  // b(1000);

  
}
// ========= Tracks ============
// sample track from the RT rules 2024
// https://www.soinc.org/sites/default/files/uploaded_files/Robot%20Tour%202024%20-%20Event%20Poster%20-%2020230905.pdf
// Comp stuff
// for beginning and end distances, add 150 + 12.5 to account for dowel distance and half width of 2.5 cm tape
// stop time = difference of time from empty run to

void sampleTrack0() {
  stop_time = 1700;
  forward(400);
  left();
  forward(500);
  right();
  forward(500);
  right();
  forward(500);
  left();
  forward(500);
  right();
  forward(1000);
  right();
  forward(300);
  backward(300);
  right();
  forward(500);
  right();
  forward(300);
  backward(300);
  left();
  forward(1000);
  right();
  forward(500);
  right();
  forward(390);
}

void sampleTrack1() { // +150 - 150
  stop_time = 1700;
  forward(1850);
  backward(300+150);
  left();
  forward(1500);
  right();
  forward(200);
  backward(200);
  right();
  forward(500);
  right();
  stop_time = 0;
  forward(1000);
  right();
  forward(300);
  backward(300);
  right();
  forward(900-150);
}

// RT 2024 sample tracks - regional level
// https://drive.google.com/file/d/1H9-s7VGudvA5Ume6mTJXbMIRGqsepRAO/view?usp=sharing

// 1
void sampleTrack2() {
 
  
}

void trackMiraLoma() {
  stop_time = 2500;
  forward(250+150);
  forward(500);
  forward(500);
  forward(500);
  left();
  left();
  forward(500);
  forward(500);
  right();
  forward(500-150);
}
void trackGullSO2() {
  stop_time = 0;
  
}

void trackMiraLoma2() {
  stop_time = 2500;
  forward(250+150);
  forward(500);
  forward(500);
  forward(500);
  left();
  left();
  forward(500);
  forward(500);
  right();
  forward(500-150);
}
void SampleTrack5() {
  stop_time = 0;
  f(250+150);
  l();
  f(500);
  r();
  f(500);
  r();
  f(500);
  l();
  f(500);
  f(300);
  b(300);
  l();
  f(500);
  r();
  f(500);
  l();
  f(500);
  l();
  f(500);
  r();
  f(500);
  r();
  f(300);
  b(300);
  r();
  f(500);
  r();
  f(500);
  r();
  f(500);
  l();
  f(500);
  l();
  f(500-150);
}

void SampleTrack_117 ()
{
  f(150+250);
  l();
  f(500);
  r();
  f(500);
  r();
  f(1000);
  r();
  f(500);
  l();
  f(500);
  l();
  f(1250);
  b(250);
  l();
  f(500);
  r();
  f(500);
  l();
  f(900);
  b(900);
  l();
  f(500);
  r();
  f(500-150);
}

void AggieSO ()
{
stop_time = 0;
f(250+150);
l();
f(500);
r();
f(500);
l();
f(500);
l(89);
f(250);
b(250);
r();
r();
f(500+350);
b(500+350);
r();
f(500+500);
l();
f(400);
r();
f(500);
l();
f(300);
b(300);
l();
f(500+500);
r();
f(500-150);
}

// 250 + 75 + 25 -  <distance between dowel and robot center, or 150 for us> for in and outs
void BARSO() {
  stop_time = 0;
  f(250+150);
  l();
  f(500);
  r();
  f(500);
  l();
  f(500);
  l();
  f(500);
  r();
  f(250+75+25-150);
  b(250+75+25-150);
  r();
  f(1000);
  l();
  f(500);
  r();
  f(500);
  r();
  f(320);
  b(320);
  r();
  f(500);
  l();
  f(1000);
  l();
  f(500);
  r();
  f(320);
  b(220);
  r();
  f(500-150);
}

void BARSOEASY() {
stop_time = 0;
  f(250+150);
  l();
  f(500);
  r();
  f(500);
  l();
  f(500);
  l();
  f(500);
  r();
  f(250+75+25-150);
  b(250+75+25-150);
  r();
  f(1000);
  l();
  f(500);
  r();
  f(500);
  r();
  f(250+75+25-150);
  b(250+75+25-150);
  r();
  f(500);
  l();
  f(1000-150);
  }



void setup() {
  Serial.begin(9600);
  // motors and encoders
  setupMotors();

  // start button
  startButton();
}

void loop() {

  t_started = millis();

  // test
  if (run_test) {
    test();
  // track
  } else {
    // modify here
    // comment out the sampleTrack and uncomment trackMiraLoma();
    BARSO();
    //SampleTrack_117();
    //trackMiraLoma();
  }
  // end
  unsigned long run_time = millis() - t_started;
  Serial.print("Total_run_time:");
  Serial.println(run_time/1000.0);
  while(true) {
    updateEncoders();
    printDebugMessages();
    delay(100);
  }
  // sleep forever
  while(true) {};
}