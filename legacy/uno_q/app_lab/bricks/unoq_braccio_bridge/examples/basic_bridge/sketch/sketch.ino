#include <Arduino_RouterBridge.h>
#include "UnoQBraccioBridge.h"

void setup() {
  Bridge.begin();
  Bridge.provide("move_braccio", move_braccio);
  setupBraccioBridge();
  move_braccio(90, 45, 180, 180, 90, 10);
}

void loop() {
  delay(1000);
}
