// first, you need to install the motor driver for L293D motor driver
// Go to Sketch -> Include Library -> Manage Library -> Search for Adafruit Motor Shield V1
#include <NewPing.h>
#include <Servo.h>
#include <AFMotor.h>

// ======================= Sensors & Pins =======================
#define TRIGGER_PIN A0
#define ECHO_PIN    A1
#define max_distance 50

// IR sensors
#define irLeft  A3
#define irRight A2

// ======================= Motor Config =========================
#define MAX_SPEED 255
#define MAX_SPEED_OFFSET 20

// Motor inversion (true = inverted, false = normal)
const bool INVERT_LEFT_MOTORS  = true;   // Motors 1 & 2
const bool INVERT_RIGHT_MOTORS = false;  // Motors 3 & 4

// IR polarity inversion (set to true if your wiring/sensors read inverted)
const bool INVERT_LEFT_IR  = false;
const bool INVERT_RIGHT_IR = false;

// ======================= Servo / Ultrasonic ===================
// Make forward direction tunable so you can micro‑align the HC‑SR04.
// Start with 90 and tweak ± a few degrees until it points straight ahead.
const int SERVO_PIN      = 10;
const int SERVO_FORWARD  = 53;  // <— TUNE THIS
const int SERVO_LEFT     = SERVO_FORWARD + 30; // scan left angle
const int SERVO_RIGHT    = SERVO_FORWARD - 30; // scan right angle

// ======================= Avoidance Tunables ===================
const int OBSTACLE_CM       = 15;   // trigger distance for avoidance
const int AVOID_BACK_MS     = 500;  // back off time
const int AVOID_TURN_MS     = 500;  // pivot time toward chosen side
const int AVOID_FORWARD_MS  = 1450;  // drive forward time after turning

// Line re-acquisition (simple + tunable)
const int REACQ_TURN_MS     = 900;  // pivot into the OPPOSITE direction
const unsigned long REACQ_MAX_MS = 3500; // give up after this many ms if no line

Servo servo;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, max_distance);

// Motors (Adafruit Motor Shield V1)
AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);

// ---------- Helpers (prototypes) ----------
void lookForward();
int  getDistance();
int  lookLeft();
int  lookRight();
void Stop();
void runLeftMotors(int direction);
void runRightMotors(int direction);
void moveForward();
void moveBackward();
void moveRight();
void moveLeft();
void avoidManeuver(bool preferLeft);
bool onLine();
void objectAvoid();

void setup() {
  Serial.begin(9600);
  pinMode(irLeft, INPUT);
  pinMode(irRight, INPUT);

  servo.attach(SERVO_PIN);
  lookForward();                 // Ensure ultrasonic faces forward at startup
  delay(300);

  motor1.setSpeed(130);
  motor2.setSpeed(130);
  motor3.setSpeed(130);
  motor4.setSpeed(130);
}

// Read IRs with optional inversion
int readLeftIR()  { int v = digitalRead(irLeft);  return INVERT_LEFT_IR  ? !v : v; }
int readRightIR() { int v = digitalRead(irRight); return INVERT_RIGHT_IR ? !v : v; }

void loop() {
  // Line-follow decisions (kept simple)
  if (readLeftIR() == 0 && readRightIR() == 0) {
    objectAvoid();
    // forward handled inside objectAvoid when clear
  }
  else if (readLeftIR() == 0 && readRightIR() == 1) {
    objectAvoid();
    moveLeft();
  }
  else if (readLeftIR() == 1 && readRightIR() == 0) {
    objectAvoid();
    moveRight();
  }
  else if (readLeftIR() == 1 && readRightIR() == 1) {
    Stop();
    lookForward();  // keep ultrasonic facing forward while stopped
  }
}

// Ensure the ultrasonic faces straight ahead
void lookForward() {
  servo.write(SERVO_FORWARD);
}

void objectAvoid() {
  lookForward(); // start with the sensor forward

  int d = getDistance();
  if (d <= OBSTACLE_CM) {
    // Obstacle close => stop and scan
    Stop();

    int L = lookLeft();
    int R = lookRight();
    delay(100);

    bool preferLeft = (L >= R); // choose the freer side
    avoidManeuver(preferLeft);

    // exiting obstacle-avoid => face forward again
    lookForward();
  } else {
    // Clear path => go forward and keep sensor forward
    lookForward();
    moveForward();
  }
}

int getDistance() {
  delay(50);
  int cm = sonar.ping_cm();
  if (cm == 0) cm = 100; // treat out-of-range as far
  return cm;
}

int lookLeft () {
  servo.write(SERVO_LEFT);
  delay(500);
  int dist = getDistance();
  delay(100);
  lookForward();   // RECENTER after scan
  Serial.print("Left:");
  Serial.println(dist);
  return dist;
}

int lookRight() {
  servo.write(SERVO_RIGHT);
  delay(500);
  int dist = getDistance();
  delay(100);
  lookForward();   // RECENTER after scan
  Serial.print("Right:");
  Serial.println(dist);
  return dist;
}

// A line is considered found when either IR sees it (tune by flipping the IR inversion constants above).
bool onLine() {
  return (readLeftIR() == 0 || readRightIR() == 0);
}

void avoidManeuver(bool preferLeft) {
  // 1) Back off
  moveBackward();            delay(AVOID_BACK_MS);

  // 2) Pivot toward the freer side
  if (preferLeft) moveLeft(); else moveRight();
                             delay(AVOID_TURN_MS);

  // 3) Drive forward past the obstacle
  moveForward();             delay(AVOID_FORWARD_MS);

  // 4) Pivot into the OPPOSITE direction to re-approach the line edge
  if (preferLeft) moveRight(); else moveLeft();
                             delay(REACQ_TURN_MS);

  // 5) Keep rolling forward until the line is detected (or timeout)
  unsigned long start = millis();
  while (millis() - start < REACQ_MAX_MS) {
    moveForward();
    if (onLine()) break;  // let the main loop take over line following
    delay(10);
  }
}

void Stop() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

// Run left motors with inversion handling
void runLeftMotors(int direction) {
  int actual = direction;
  if (INVERT_LEFT_MOTORS) {
    if (direction == FORWARD) actual = BACKWARD;
    else if (direction == BACKWARD) actual = FORWARD;
  }
  motor1.run(actual);
  motor2.run(actual);
}

// Run right motors with inversion handling
void runRightMotors(int direction) {
  int actual = direction;
  if (INVERT_RIGHT_MOTORS) {
    if (direction == FORWARD) actual = BACKWARD;
    else if (direction == BACKWARD) actual = FORWARD;
  }
  motor3.run(actual);
  motor4.run(actual);
}

void moveForward()  { runLeftMotors(FORWARD);  runRightMotors(FORWARD); }
void moveBackward() { runLeftMotors(BACKWARD); runRightMotors(BACKWARD); }
void moveRight()    { runLeftMotors(BACKWARD); runRightMotors(FORWARD); }
void moveLeft()     { runLeftMotors(FORWARD);  runRightMotors(BACKWARD); }
