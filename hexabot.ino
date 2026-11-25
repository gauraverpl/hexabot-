/*
  Hexabot baseline controller
  Board: Arduino Mega 2560
  Actuators: 6 legs x 4 hobby servos = 24 servos
  Sensor: HC-SR04 ultrasonic module
  Bluetooth: serial module on Serial1 (Mega RX1=19, TX1=18)

  Bluetooth commands (send one character):
    F = walk forward, B = walk backward, L/R = turn, S = stop/stand
    A = automatic forward + obstacle avoidance, M = manual mode

  IMPORTANT:
  - This is a configurable starting point, not universal leg inverse kinematics.
    Set SERVO_PINS, NEUTRAL, and SERVO_DIR for your wiring/mechanics, then tune
    the stride/lift constants with the robot supported off the ground.
  - Do not power 24 servos from the Mega. Use a suitable external servo supply
    and connect its ground to Arduino GND. Add a fuse/current limiting as needed.
*/
#include <Servo.h>

const uint8_t LEG_COUNT = 6;
const uint8_t JOINTS_PER_LEG = 4;

// Legs ordered front-left, middle-left, rear-left, front-right, middle-right,
// rear-right. Each row is coxa/hip, femur, tibia, foot servo.
const uint8_t SERVO_PINS[LEG_COUNT][JOINTS_PER_LEG] = {
  {22, 23, 24, 25}, {26, 27, 28, 29}, {30, 31, 32, 33},
  {34, 35, 36, 37}, {38, 39, 40, 41}, {42, 43, 44, 45}
};

// Change these directions if a joint moves opposite to the desired direction.
const int8_t SERVO_DIR[LEG_COUNT][JOINTS_PER_LEG] = {
  { 1, 1, 1, 1}, { 1, 1, 1, 1}, { 1, 1, 1, 1},
  {-1,-1,-1,-1}, {-1,-1,-1,-1}, {-1,-1,-1,-1}
};

// Calibrate each servo's safe standing position (degrees).
const uint8_t NEUTRAL[LEG_COUNT][JOINTS_PER_LEG] = {
  {90, 90, 90, 90}, {90, 90, 90, 90}, {90, 90, 90, 90},
  {90, 90, 90, 90}, {90, 90, 90, 90}, {90, 90, 90, 90}
};

const uint8_t TRIG_PIN = 46;
const uint8_t ECHO_PIN = 47;
const uint16_t OBSTACLE_CM = 25;
const uint16_t SENSOR_INTERVAL_MS = 100;
const uint16_t SERVO_UPDATE_MS = 20;
const uint16_t AUTO_REVERSE_MS = 450;
const uint16_t AUTO_TURN_MS = 650;

// Tune these values to suit the geometry and weight of the robot.
const float STRIDE_DEG = 18.0f;
const float LIFT_DEG = 12.0f;
const float KNEE_LIFT_DEG = 18.0f;
const float CYCLE_SECONDS = 1.2f;
const float SWING_FRACTION = 0.25f;

Servo legServo[LEG_COUNT][JOINTS_PER_LEG];

// Tripod groups: front-left + rear-left + middle-right; the other three form
// the alternate group. This is the common alternating-tripod walking pattern.
const uint8_t TRIPOD_PHASE[LEG_COUNT] = {0, 1, 0, 1, 0, 1};
const int8_t LEG_SIDE[LEG_COUNT] = {-1, -1, -1, 1, 1, 1};

enum Mode : uint8_t { MANUAL_MODE, AUTO_MODE };
enum AutoState : uint8_t { AUTO_WALK, AUTO_BACKUP, AUTO_TURN };
Mode mode = MANUAL_MODE;
AutoState autoState = AUTO_WALK;

// Manual/auto movement values: forward is -1, reverse +1, turn is -1 or +1.
int8_t drive = 0;
int8_t turn = 0;
uint32_t stateStartedAt = 0;
uint32_t lastServoUpdate = 0;
uint32_t lastSensorRead = 0;
uint32_t gaitStartedAt = 0;
int distanceCm = 999;

int clampAngle(int value) {
  if (value < 0) return 0;
  if (value > 180) return 180;
  return value;
}

void setJoint(uint8_t leg, uint8_t joint, float offsetDegrees) {
  const int angle = (int)NEUTRAL[leg][joint] +
                    SERVO_DIR[leg][joint] * (int)offsetDegrees;
  legServo[leg][joint].write(clampAngle(angle));
}

void stand() {
  drive = 0;
  turn = 0;
  for (uint8_t leg = 0; leg < LEG_COUNT; ++leg) {
    for (uint8_t joint = 0; joint < JOINTS_PER_LEG; ++joint) {
      legServo[leg][joint].write(NEUTRAL[leg][joint]);
    }
  }
}

void beginServos() {
  for (uint8_t leg = 0; leg < LEG_COUNT; ++leg) {
    for (uint8_t joint = 0; joint < JOINTS_PER_LEG; ++joint) {
      legServo[leg][joint].attach(SERVO_PINS[leg][joint]);
      legServo[leg][joint].write(NEUTRAL[leg][joint]);
    }
  }
}

// Oscillating tripod gait using configurable joint offsets. This provides a
// simple walking baseline; tune offsets/directions for the actual linkage.
void updateGait(uint32_t now) {
  if (drive == 0 && turn == 0) {
    stand();
    return;
  }

  float phase = fmod((now - gaitStartedAt) / (CYCLE_SECONDS * 1000.0f), 1.0f);
  for (uint8_t leg = 0; leg < LEG_COUNT; ++leg) {
    float p = phase + (TRIPOD_PHASE[leg] ? 0.5f : 0.0f);
    if (p >= 1.0f) p -= 1.0f;

    const bool swing = p < SWING_FRACTION;
    float stride;
    float lift;
    if (swing) {
      const float t = p / SWING_FRACTION;
      stride = -1.0f + 2.0f * t;
      lift = sin(t * PI);
    } else {
      const float t = (p - SWING_FRACTION) / (1.0f - SWING_FRACTION);
      stride = 1.0f - 2.0f * t;
      lift = 0.0f;
    }

    float legDrive = (float)drive;
    if (turn != 0) legDrive += (float)turn * LEG_SIDE[leg];
    const float forwardOffset = stride * STRIDE_DEG * legDrive;
    const float liftOffset = lift * LIFT_DEG;

    // Joint order: coxa/hip, femur, tibia, foot. The femur/tibia/foot
    // offsets raise the leg in swing and return it to its standing pose.
    setJoint(leg, 0, forwardOffset);
    setJoint(leg, 1, liftOffset - forwardOffset * 0.35f);
    setJoint(leg, 2, -lift * KNEE_LIFT_DEG + forwardOffset * 0.20f);
    setJoint(leg, 3, lift * (LIFT_DEG - KNEE_LIFT_DEG * 0.35f));
  }
}

int readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  const unsigned long duration = pulseIn(ECHO_PIN, HIGH, 25000UL);
  if (duration == 0) return 999; // no echo: treat as clear/far away
  return (int)(duration / 58UL);
}

void startAutoAvoidance(uint32_t now) {
  // Stop the forward gait, back up briefly, then turn and resume forward.
  autoState = AUTO_BACKUP;
  stateStartedAt = now;
  drive = 1;
  turn = 0;
}

void updateAuto(uint32_t now) {
  if (autoState == AUTO_WALK) {
    drive = -1;
    turn = 0;
    if (distanceCm > 0 && distanceCm <= OBSTACLE_CM) {
      startAutoAvoidance(now);
    }
  } else if (autoState == AUTO_BACKUP) {
    drive = 1;
    turn = 0;
    if (now - stateStartedAt >= AUTO_REVERSE_MS) {
      autoState = AUTO_TURN;
      stateStartedAt = now;
      drive = 0;
      // Alternate turn direction each avoidance event.
      static int8_t nextTurn = 1;
      turn = nextTurn;
      nextTurn = -nextTurn;
    }
  } else { // AUTO_TURN
    drive = 0;
    if (now - stateStartedAt >= AUTO_TURN_MS) {
      autoState = AUTO_WALK;
      drive = -1;
      turn = 0;
    }
  }
}

void handleBluetooth(char command) {
  if (command >= 'a' && command <= 'z') command -= ('a' - 'A');
  switch (command) {
    case 'A':
      mode = AUTO_MODE;
      autoState = AUTO_WALK;
      drive = -1;
      turn = 0;
      stateStartedAt = millis();
      break;
    case 'M':
      mode = MANUAL_MODE;
      stand();
      break;
    case 'F': mode = MANUAL_MODE; drive = -1; turn = 0; break;
    case 'B': mode = MANUAL_MODE; drive = 1; turn = 0; break;
    case 'L': mode = MANUAL_MODE; drive = 0; turn = -1; break;
    case 'R': mode = MANUAL_MODE; drive = 0; turn = 1; break;
    case 'S': mode = MANUAL_MODE; stand(); break;
    default: break; // Ignore CR/LF and unknown characters.
  }
}

void setup() {
  Serial.begin(115200);   // USB diagnostics
  Serial1.begin(9600);    // Bluetooth module; configure its baud rate as needed
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  beginServos();
  gaitStartedAt = millis();
  Serial.println(F("Hexabot ready. Bluetooth: F B L R S A M"));
}

void loop() {
  while (Serial1.available() > 0) {
    handleBluetooth((char)Serial1.read());
  }

  const uint32_t now = millis();
  if (now - lastSensorRead >= SENSOR_INTERVAL_MS) {
    lastSensorRead = now;
    distanceCm = readDistanceCm();
    Serial.print(F("Distance cm: "));
    Serial.println(distanceCm);
  }

  if (mode == AUTO_MODE) updateAuto(now);

  if (now - lastServoUpdate >= SERVO_UPDATE_MS) {
    lastServoUpdate = now;
    updateGait(now);
  }
}
