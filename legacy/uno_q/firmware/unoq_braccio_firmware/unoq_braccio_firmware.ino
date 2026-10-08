#include <Braccio.h>
#include <Servo.h>

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_ver;
Servo wrist_rot;
Servo gripper;

const int JOINTS = 6;
const int MIN_LIMITS[JOINTS] = {0, 15, 0, 0, 0, 10};
const int MAX_LIMITS[JOINTS] = {180, 165, 180, 180, 180, 110};
int target[JOINTS] = {90, 45, 180, 180, 90, 10};
unsigned long moveCount = 0;
unsigned long lastMoveMs = 0;
unsigned long lastCommandMs = 0;

String inputLine;

int clampJoint(int index, int value) {
  if (value < MIN_LIMITS[index]) {
    return MIN_LIMITS[index];
  }
  if (value > MAX_LIMITS[index]) {
    return MAX_LIMITS[index];
  }
  return value;
}

void moveToTarget() {
  unsigned long startMs = millis();
  Braccio.ServoMovement(
    20,
    target[0],
    target[1],
    target[2],
    target[3],
    target[4],
    target[5]
  );
  lastMoveMs = millis() - startMs;
  lastCommandMs = millis();
  moveCount++;
}

bool parseMove(String line) {
  line.trim();
  if (!line.startsWith("M ")) {
    return false;
  }

  int values[JOINTS];
  int start = 2;
  for (int i = 0; i < JOINTS; i++) {
    int space = line.indexOf(' ', start);
    String token = space == -1 ? line.substring(start) : line.substring(start, space);
    if (token.length() == 0) {
      return false;
    }
    values[i] = token.toInt();
    start = space + 1;
    if (space == -1 && i < JOINTS - 1) {
      return false;
    }
  }

  for (int i = 0; i < JOINTS; i++) {
    target[i] = clampJoint(i, values[i]);
  }
  return true;
}

void printStatus() {
  Serial.print("STAT uptime_ms=");
  Serial.print(millis());
  Serial.print(" move_count=");
  Serial.print(moveCount);
  Serial.print(" last_move_ms=");
  Serial.print(lastMoveMs);
  Serial.print(" last_command_ms=");
  Serial.print(lastCommandMs);
  Serial.print(" target=");
  for (int i = 0; i < JOINTS; i++) {
    if (i > 0) {
      Serial.print(",");
    }
    Serial.print(target[i]);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Braccio.begin();
  moveToTarget();
  Serial.println("READY UNOQ_BRACCIO");
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      inputLine.trim();
      if (inputLine == "S") {
        printStatus();
      } else if (parseMove(inputLine)) {
        moveToTarget();
        Serial.println("OK");
      } else {
        Serial.println("ERR");
      }
      inputLine = "";
    } else if (c != '\r') {
      inputLine += c;
    }
  }
}
