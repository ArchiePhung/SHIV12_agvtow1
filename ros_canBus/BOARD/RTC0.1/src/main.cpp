#include <Arduino.h>
#include "CAN_manager.h"
#include "Main_controller.h"

#include "ros.h"
#include "ros/time.h"

#include "message_pkg/CAN_send.h"
#include "message_pkg/CAN_received.h"
#include "message_pkg/CAN_status.h"

#define FREQUENCY_PUB_POWER_INFO 4
#define FREQUENCY_PUB_OC_INFO    4
#define FREQUENCY_PUB_HC_INFO    24

void create_task();
void CAN_send_Callback(const message_pkg::CAN_send &data);
void check_spinOnce();

ros::NodeHandle nodeHandle;
message_pkg::CAN_send CAN_send;
message_pkg::CAN_received CAN_received;
message_pkg::CAN_received CAN_status;

ros::Publisher CAN_received_pub("CAN_received", &CAN_received);
ros::Subscriber<message_pkg::CAN_send> CAN_send_sub("CAN_send", CAN_send_Callback);
// ros::Publisher CAN_status_pub("CAN_status", &CAN_status);

unsigned long saveTime_pub = 0;
unsigned long saveTime_sub = 0;
unsigned long debugTime = 0;

TaskHandle_t Task_Run_CAN; // Task_Run_CAN
CAN_manager* MainCAN = new CAN_manager(CAN_BAUD_SPEED, CAN_TX, CAN_RX, CAN_FRAME, CAN_ID, CAN_SEND_SIZE);
Main_controller* MainCtrl = new Main_controller(MainCAN);

int sts_spinOnce = 0;
void CAN_send_Callback(const message_pkg::CAN_send &data)
{
    CAN_send = data; 
    // - Control main
    MainCtrl->send_CAN->id = CAN_send.id;
    MainCtrl->send_CAN->byte0 = CAN_send.byte0;
    MainCtrl->send_CAN->byte1 = CAN_send.byte1;
    MainCtrl->send_CAN->byte2 = CAN_send.byte2;
    MainCtrl->send_CAN->byte3 = CAN_send.byte3;
    MainCtrl->send_CAN->byte4 = CAN_send.byte4;
    MainCtrl->send_CAN->byte5 = CAN_send.byte5;
    MainCtrl->send_CAN->byte6 = CAN_send.byte6;
    MainCtrl->send_CAN->byte7 = CAN_send.byte7;
	MainCtrl->is_sendCAN = true;
	MainCtrl->sts_LED_ROS_RECEIVED = true;
} 

void CanHandle_Run( void* pvParameters ){
    for(;;){
        MainCtrl->Run_Send_CAN();
        vTaskDelay(4);
    }
}

bool communication_check(){
	if (!nodeHandle.connected()){ // Mất kết nối với ROS -> phát yêu cầu dừng động cơ -> Restart Esp.
		// delay(200);
        // ESP.restart();
		// MainCtrl->led_control(1, 0, 1, 1);
		return 1;
		
	}else{
		// MainCtrl->led_control(0, 1, 1, 1);
	}
	return 0;
}

void setup() {
    MainCtrl->init_main();
	MainCtrl->led_start();
    delay(10);
    // - ROS
    nodeHandle.initNode();
    nodeHandle.getHardware()->setBaud(57600); 
    delay(10);
    nodeHandle.advertise(CAN_received_pub);
    nodeHandle.subscribe(CAN_send_sub);
	// nodeHandle.advertise(CAN_status_pub);
	delay(10);
    // -
    MainCAN->CAN_prepare();
    delay(2000);

    create_task();
    // - ROS
    while (!nodeHandle.connected())
    {   
        MainCtrl->led_loop();
        // MainCtrl->led_blink();
        delay(10);
        nodeHandle.spinOnce();
    }
}

void loop() {
	// communication_check();
    /* -- -- -- -- CAN RECEIVED -- -- -- -- */
    MainCtrl->CANReceiveHandle();
    
    CAN_received.idSend = MainCtrl->received_CAN->idSend;
    CAN_received.byte0  = MainCtrl->received_CAN->byte0;
    CAN_received.byte1  = MainCtrl->received_CAN->byte1;
    CAN_received.byte2  = MainCtrl->received_CAN->byte2;
    CAN_received.byte3  = MainCtrl->received_CAN->byte3;
    CAN_received.byte4  = MainCtrl->received_CAN->byte4;
	CAN_received.byte5  = MainCtrl->received_CAN->byte5;
	CAN_received.byte6  = MainCtrl->received_CAN->byte6;
	CAN_received.byte7  = MainCtrl->received_CAN->byte7;

	if (MainCtrl->is_receivedCAN == true){
		MainCtrl->is_receivedCAN = false;
		MainCtrl->sts_LED_ROS_SEND = true;
		CAN_received_pub.publish(&CAN_received);
		saveTime_pub = millis();
	}
	/* -- -- -- -- CAN SEND -- -- -- -- */
	sts_spinOnce = nodeHandle.spinOnce();
	MainCtrl->led_loop();
}

void check_spinOnce(){
	if (sts_spinOnce == 0){
		MainCtrl->led_control(0, 1, 1, 1);
	}else if (sts_spinOnce == -1){
		MainCtrl->led_control(1, 0, 1, 1);
	}else if (sts_spinOnce == -2){
		MainCtrl->led_control(1, 1, 0, 1);
	}
}

void create_task() {   
    xTaskCreatePinnedToCore(
        CanHandle_Run, 	/* Task function. */
        "Task_Run_CAN", /* name of task. */
        10000,          /* Stack size of task */
        NULL,         	/* parameter of the task */
        1,            	/* priority of the task */
        &Task_Run_CAN,  /* Task handle to keep track of created task */
        1);           	/* pin task to core 0 */              
}