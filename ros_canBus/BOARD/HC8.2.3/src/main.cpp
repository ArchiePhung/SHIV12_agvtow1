#include <Arduino.h>
#include "maincontrol.h"

MainControl Main;

void setup() {
  LOG_BEGIN(57600);
  Main.MainInit();
}

void loop(){
  Main.Run();
}
