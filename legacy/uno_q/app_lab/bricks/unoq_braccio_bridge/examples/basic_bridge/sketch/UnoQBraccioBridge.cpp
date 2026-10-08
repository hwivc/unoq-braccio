#include "UnoQBraccioBridge.h"

#include <Servo.h>

namespace {
const int JOINTS = 6;
const int MIN_LIMITS[JOINTS] = {0, 15, 0, 0, 0, 10};
const int MAX_LIMITS[JOINTS] = {180, 165, 180, 180, 180, 110};
const int SERVO_PINS[JOINTS] = {11, 10, 9, 6, 5, 3};
const int SOFT_START_PIN = 12;
Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_ver;
Servo wrist_rot;
Servo gripper;
int current[JOINTS] = {90, 45, 180, 180, 90, 10};
int clampJoint(int index, int value) {
  return max(MIN_LIMITS[index], min(MAX_LIMITS[index], value));
}
void writeCurrent() {
  base.write(current[0]);
  shoulder.write(current[1]);
  elbow.write(current[2]);
  wrist_ver.write(current[3]);
  wrist_rot.write(current[4]);
  gripper.write(current[5]);
}
}

void setupBraccioBridge() {
  pinMode(SOFT_START_PIN, OUTPUT);
  digitalWrite(SOFT_START_PIN, HIGH);
  base.attach(SERVO_PINS[0]);
  shoulder.attach(SERVO_PINS[1]);
  elbow.attach(SERVO_PINS[2]);
  wrist_ver.attach(SERVO_PINS[3]);
  wrist_rot.attach(SERVO_PINS[4]);
  gripper.attach(SERVO_PINS[5]);
  writeCurrent();
}

bool move_braccio(int b, int s, int e, int wv, int wr, int g) {
  int target[JOINTS] = {
    clampJoint(0, b),
    clampJoint(1, s),
    clampJoint(2, e),
    clampJoint(3, wv),
    clampJoint(4, wr),
    clampJoint(5, g)
  };
  for (int step = 0; step < 110; step++) {
    bool done = true;
    for (int i = 0; i < JOINTS; i++) {
      if (current[i] < target[i]) {
        current[i]++;
        done = false;
      } else if (current[i] > target[i]) {
        current[i]--;
        done = false;
      }
    }
    writeCurrent();
    if (done) {
      break;
    }
    delay(20);
  }
  return true;
}
