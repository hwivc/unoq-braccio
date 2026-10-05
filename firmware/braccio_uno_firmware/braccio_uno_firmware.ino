/*
  Braccio serial firmware for an Arduino UNO (R3 or R4) with the Braccio shield.

  The UNO only drives the six servos. Everything else (ROS 2, cameras,
  inverse kinematics) runs on the Linux machine at the other end of the USB
  cable, which talks to this sketch with one text line per command at
  115200 baud:

    M <base> <shoulder> <elbow> <wrist_ver> <wrist_rot> <gripper>
        Move to these servo angles (integer degrees). Values are clamped to
        the joint limits. Replies OK at once, then DONE when the arm arrives.
        A new M during a move retargets smoothly from wherever the arm is.
    S   Status:  STAT pos=..  target=..  moving=0|1  speed=..  moves=..  uptime_ms=..
    V <deg_per_s>   Speed of the joint that moves furthest (10-180). Replies OK.
    H   Hold: stop where the arm is now. Replies OK.
    I   Identify: replies with the READY banner.

  Anything else replies ERR <reason>. The sketch prints the READY banner once
  the servos are powered, about 6 s after reset.

  Motion is non-blocking: every joint is interpolated so all of them arrive
  together, and the serial port is read the whole time. The official Braccio
  library is not used, because its ServoMovement() blocks for the whole move
  and caps the gripper at 73 degrees, which does not close this gripper.

  Shield pins (same as the Braccio library): M1 base 11, M2 shoulder 10,
  M3 elbow 9, M4 wrist vertical 6, M5 wrist rotation 5, M6 gripper 3, soft
  start 12. Power the servos from the shield's own 5 V supply, never from USB.
*/

#include <Servo.h>
#include <stdlib.h>

const int JOINTS = 6;
const uint8_t SERVO_PINS[JOINTS] = {11, 10, 9, 6, 5, 3};
const uint8_t SOFT_START_PIN = 12;

// Keep in sync with JOINT_LIMITS and POSES["rest"] in braccio_model.py.
const int MIN_LIMITS[JOINTS] = {0, 15, 0, 0, 0, 10};
const int MAX_LIMITS[JOINTS] = {180, 165, 180, 180, 180, 110};
const int START_POSE[JOINTS] = {90, 45, 180, 180, 90, 10};

const int SPEED_MIN = 10;
const int SPEED_MAX = 180;
const int SPEED_DEFAULT = 60;

const char BANNER[] = "READY BRACCIO_UNO 1";

Servo servos[JOINTS];
int current[JOINTS];
int moveFrom[JOINTS];
int target[JOINTS];
bool moving = false;
unsigned long moveStartMs = 0;
unsigned long moveDurationMs = 0;
int speedDegPerS = SPEED_DEFAULT;
unsigned long moveCount = 0;

char line[48];
uint8_t lineLength = 0;
bool lineTooLong = false;

int clampJoint(int index, long value) {
  if (value < MIN_LIMITS[index]) {
    return MIN_LIMITS[index];
  }
  if (value > MAX_LIMITS[index]) {
    return MAX_LIMITS[index];
  }
  return (int)value;
}

// Ramp the shield's servo power up gently instead of switching it on hard,
// so the arm does not jump at power-on. Same timing as the Braccio library:
// a software PWM on the soft-start pin for 6 s, then fully on.
void softStart() {
  unsigned long start = millis();
  while (millis() - start < 2000) {
    digitalWrite(SOFT_START_PIN, HIGH);
    delayMicroseconds(80);
    digitalWrite(SOFT_START_PIN, LOW);
    delayMicroseconds(450);
  }
  while (millis() - start < 6000) {
    digitalWrite(SOFT_START_PIN, HIGH);
    delayMicroseconds(75);
    digitalWrite(SOFT_START_PIN, LOW);
    delayMicroseconds(430);
  }
  digitalWrite(SOFT_START_PIN, HIGH);
}

// Start a move to goal[] from wherever the arm is now. The joint with the
// furthest to go sets the duration, so every joint arrives at the same time.
void startMove(const long goal[]) {
  int largest = 0;
  for (int i = 0; i < JOINTS; i++) {
    moveFrom[i] = current[i];
    target[i] = clampJoint(i, goal[i]);
    largest = max(largest, abs(target[i] - current[i]));
  }
  moveDurationMs = (unsigned long)largest * 1000UL / (unsigned long)speedDegPerS;
  moveStartMs = millis();
  moving = true;
  moveCount++;
}

void updateMove() {
  if (!moving) {
    return;
  }
  unsigned long elapsed = millis() - moveStartMs;
  bool finished = elapsed >= moveDurationMs;
  for (int i = 0; i < JOINTS; i++) {
    int position = target[i];
    if (!finished) {
      long delta = (long)(target[i] - moveFrom[i]);
      position = moveFrom[i] + (int)(delta * (long)elapsed / (long)moveDurationMs);
    }
    if (position != current[i]) {
      current[i] = position;
      servos[i].write(position);
    }
  }
  if (finished) {
    moving = false;
    Serial.println(F("DONE"));
  }
}

// Parse exactly `count` whitespace-separated integers from text.
bool parseInts(const char *text, long values[], int count) {
  const char *cursor = text;
  for (int i = 0; i < count; i++) {
    char *end;
    values[i] = strtol(cursor, &end, 10);
    if (end == cursor) {
      return false;
    }
    cursor = end;
  }
  while (*cursor == ' ') {
    cursor++;
  }
  return *cursor == '\0';
}

void printList(const int values[]) {
  for (int i = 0; i < JOINTS; i++) {
    if (i > 0) {
      Serial.print(',');
    }
    Serial.print(values[i]);
  }
}

void printStatus() {
  Serial.print(F("STAT pos="));
  printList(current);
  Serial.print(F(" target="));
  printList(target);
  Serial.print(F(" moving="));
  Serial.print(moving ? 1 : 0);
  Serial.print(F(" speed="));
  Serial.print(speedDegPerS);
  Serial.print(F(" moves="));
  Serial.print(moveCount);
  Serial.print(F(" uptime_ms="));
  Serial.println(millis());
}

void handleLine(const char *text) {
  const char command = text[0];
  const char *args = text + 1;
  if (command == 'M' && *args == ' ') {
    long goal[JOINTS];
    if (!parseInts(args, goal, JOINTS)) {
      Serial.println(F("ERR bad_move"));
      return;
    }
    startMove(goal);
    Serial.println(F("OK"));
  } else if (command == 'S' && *args == '\0') {
    printStatus();
  } else if (command == 'V' && *args == ' ') {
    long speed;
    if (!parseInts(args, &speed, 1) || speed < SPEED_MIN || speed > SPEED_MAX) {
      Serial.println(F("ERR bad_speed"));
      return;
    }
    speedDegPerS = (int)speed;
    Serial.println(F("OK"));
  } else if (command == 'H' && *args == '\0') {
    for (int i = 0; i < JOINTS; i++) {
      target[i] = current[i];
    }
    moving = false;
    Serial.println(F("OK"));
  } else if (command == 'I' && *args == '\0') {
    Serial.println(BANNER);
  } else {
    Serial.println(F("ERR unknown_command"));
  }
}

void readSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      if (lineTooLong) {
        Serial.println(F("ERR line_too_long"));
      } else if (lineLength > 0) {
        line[lineLength] = '\0';
        handleLine(line);
      }
      lineLength = 0;
      lineTooLong = false;
    } else if (lineLength < sizeof(line) - 1) {
      line[lineLength++] = c;
    } else {
      lineTooLong = true;
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Servo power off while the servos are attached and given the start pose,
  // then ramped on, so they wake up already holding it.
  pinMode(SOFT_START_PIN, OUTPUT);
  digitalWrite(SOFT_START_PIN, LOW);
  for (int i = 0; i < JOINTS; i++) {
    current[i] = START_POSE[i];
    target[i] = START_POSE[i];
    servos[i].attach(SERVO_PINS[i]);
    servos[i].write(current[i]);
  }
  softStart();

  Serial.println(BANNER);
}

void loop() {
  readSerial();
  updateMove();
}
