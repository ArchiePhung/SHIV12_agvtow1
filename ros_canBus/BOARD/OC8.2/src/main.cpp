#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include "CAN_manager.h"
#include "OC_controller.h"

CAN_manager* OC_CAN = new CAN_manager(CAN_BAUD_SPEED,CAN_TX,CAN_RX,CAN_FRAME,CAN_ID,CAN_SEND_SIZE);
OC_controller* OC_Ctrl = new OC_controller(OC_CAN);

void setup() {
  OC_Ctrl->init_main();
  delay(50);
  OC_CAN->CAN_prepare();
  delay(50);
}

void loop() {
  OC_Ctrl->OC_loop();
  // -
  // OC_Ctrl->debuge();
  // OC_Ctrl->LiftUp();
  // Serial.println((String) digitalRead(SENSOR_1) + ' '+ digitalRead(SENSOR_2) + ' '+digitalRead(SENSOR_3) + ' ' + digitalRead(SENSOR_4)+ ' ' + digitalRead(SENSOR_5) + ' ' + digitalRead(SENSOR_6) + ' ' + digitalRead(SENSOR_7));
}