#include <Arduino.h>
#include "CAN_manager.h"
#include "Main_controller.h"


CAN_manager* MainCAN = new CAN_manager(CAN_BAUD_SPEED, CAN_TX, CAN_RX, CAN_FRAME, CAN_ID, CAN_SEND_SIZE);
Main_controller* MainCtrl = new Main_controller(MainCAN);

void setup() {
    MainCtrl->init_main();
    delay(50);
    // -
    MainCAN->CAN_prepare();
    delay(100);

    // MainCtrl->setCoefficient_voltage(0.634, 501.59);
    // MainCtrl->setCoefficient_current(5, -14350);

    MainCtrl->setCoefficient_voltage(0.6948, 217.4741);
    MainCtrl->setCoefficient_current(6.4103, -19384.615);

}

void loop() {
    MainCtrl->loopMainCtr();
}
