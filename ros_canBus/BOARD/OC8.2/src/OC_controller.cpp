#include "OC_controller.h"

OC_controller::OC_controller(CAN_manager* _CANctr)
{
    CANctr = _CANctr;
}

OC_controller::~OC_controller()
{
}

void OC_controller::liftUp()
{
   //Quay thuan
	if (step_LiftUp == 0){ // điều khiển chiều.
		// Serial.println ("BN quay nguoc");  
    setCommandStatus(IN_PROCESSING_UP);
		timeStart_LiftUp = millis();
		digitalWrite(EN_LIFTER, HIGH);
		digitalWrite(LIFTER_L, LOW);
		speed_LiftUp = 0;
		ledcWrite(channelPWM_LiftUp, speed_LiftUp);
		step_LiftUp = 1;

	}else if (step_LiftUp == 1){ // tăng dần tốc độ.		
		if (digitalRead(LIFTER_UP_SR) == HIGH){ // Bắt được cảm biến giới hạn.
			step_LiftUp = 3;
		}

		if (speed_LiftUp >= 1022){ // Đạt tốc độ giới hạn.
			step_LiftUp = 2;
			speed_LiftUp = 1022;
      ledcWrite(channelPWM_LiftUp, speed_LiftUp);
		}else{						// Tăng dần tốc độ.
			speed_LiftUp += 20;
			ledcWrite(channelPWM_LiftUp, speed_LiftUp);
			delay(1);
		}

	}else if (step_LiftUp == 2){ // Đợi bắt được cảm biến.
		if(millis() - timeStart_LiftUp > timeCheckError){
			speed_LiftUp = 0;
			ledcWrite(channelPWM_LiftUp, speed_LiftUp);
      setCommandStatus(LIFTING_ERROR);
		}

		if (digitalRead(LIFTER_UP_SR) == HIGH){ // Bắt được cảm biến giới hạn.
			speed_LiftUp = 0;
			step_LiftUp = 3;
		}

	}else if (step_LiftUp == 3){ // Giảm dần tốc độ về 0.
		speed_LiftUp -= 600;
    ledcWrite(channelPWM_LiftUp, speed_LiftUp);

		if(speed_LiftUp <= 0){
			speed_LiftUp = 0;
			step_LiftUp = 4;
			digitalWrite(EN_LIFTER, LOW);
		}
		
		
		 
	}else if (step_LiftUp == 4){ // Hoàn thành.	
    setCommandStatus(COMMAND_DONE_UP);
	}
}

void OC_controller::liftDown()
{
	if (step_LiftDown == 0){ // điều khiển chiều.
    setCommandStatus(IN_PROCESSING_DOWN);

		// Serial.println ("BN quay nguoc");  
		timeStart_LiftDown = millis();
		digitalWrite(EN_LIFTER, HIGH);
		digitalWrite(LIFTER_H, LOW);
		speed_LiftDown = 0;
		ledcWrite(channelPWM_LiftDown, speed_LiftDown);
		step_LiftDown = 1;

	}else if (step_LiftDown == 1){ // tăng dần tốc độ.
		if (digitalRead(LIFTER_DOWN_SR) == HIGH){ // Bắt được cảm biến giới hạn.
			step_LiftDown = 3;
		}

		if (speed_LiftDown >= 1022){ // Đạt tốc độ giới hạn.
			step_LiftDown = 2;
			speed_LiftDown = 1022;
      ledcWrite(channelPWM_LiftDown, speed_LiftDown);
		}else{						// Tăng dần tốc độ.
			speed_LiftDown += 30;
			ledcWrite(channelPWM_LiftDown, speed_LiftDown);
			delay(1);
		}

	}else if (step_LiftDown == 2){ // Đợi bắt được cảm biến.
		if(millis() - timeStart_LiftDown > timeCheckError){
			speed_LiftDown = 0;
			ledcWrite(channelPWM_LiftDown, speed_LiftDown);
      setCommandStatus(LIFTING_ERROR);
		}

		if (digitalRead(LIFTER_DOWN_SR) == HIGH){ // Bắt được cảm biến giới hạn.
			speed_LiftDown = 0;
			step_LiftDown = 3;
		}

	}else if (step_LiftDown == 3){ // Giảm dần tốc độ về 0.
    ledcWrite(channelPWM_LiftDown, speed_LiftDown);
		speed_LiftDown -= 600;
		if(speed_LiftDown <= 0){
			speed_LiftDown = 0;
			step_LiftDown = 4;
			digitalWrite(EN_LIFTER, LOW);
		}
		 
	}else if (step_LiftDown == 4){ // Hoàn thành.	
    setCommandStatus(COMMAND_DONE_DOWN);
	}
}

void OC_controller::stopAndReset(){
  digitalWrite(EN_LIFTER, LOW);
  digitalWrite(EN_CONVAYER, LOW);
  digitalWrite(CONVAYER_H, LOW);
  digitalWrite(CONVAYER_L, LOW);
  digitalWrite(LIFTER_H, LOW);
  digitalWrite(LIFTER_L, LOW);

  ledcWrite(channelPWM_LiftUp, 0);
  ledcWrite(channelPWM_LiftDown, 0);
  ledcWrite(8, 0);
  ledcWrite(12, 0);
  setCommandStatus(COMMAND_DONE);

	speed_LiftDown = 0;
	speed_LiftUp = 0;
	step_LiftUp = 0;
	step_LiftDown = 0;
}

void OC_controller::init_main() 
{
  WiFi.mode(WIFI_OFF);
  btStop();
  delay(20);
  Serial.begin(57600);
  delay(20);
  pinMode(SENSOR_1, INPUT_PULLUP);
  pinMode(SENSOR_2, INPUT_PULLUP);
  pinMode(SENSOR_3, INPUT_PULLUP);
  pinMode(SENSOR_4, INPUT_PULLUP);
  pinMode(SENSOR_5, INPUT_PULLUP);
  pinMode(SENSOR_6, INPUT_PULLUP);
  pinMode(SENSOR_7, INPUT_PULLUP);

  pinMode(E1,          OUTPUT);
  pinMode(EN_LIFTER,   OUTPUT);
  pinMode(EN_CONVAYER, OUTPUT);
  pinMode(LIFTER_H,    OUTPUT);
  pinMode(LIFTER_L,    OUTPUT);
  pinMode(CONVAYER_H,  OUTPUT);
  pinMode(CONVAYER_L,  OUTPUT);
  
  digitalWrite(E1,          LOW);
  digitalWrite(EN_LIFTER,   LOW);
  digitalWrite(EN_CONVAYER, LOW);
  digitalWrite(LIFTER_H,    LOW);
  digitalWrite(LIFTER_L,    LOW);
  digitalWrite(CONVAYER_H,  LOW);
  digitalWrite(CONVAYER_L,  LOW);

  //PWM Steup
  //ledcSetup(ledChannel, freq, resolution);
  ledcSetup(0,  1000, 10);
  ledcSetup(4,  1000, 10);
  ledcSetup(8,  1000, 10);
  ledcSetup(12, 1000, 10);   
  //ledcAttachPin(ledPin, ledChannel);
  ledcAttachPin(LIFTER_H, 0);
  ledcAttachPin(LIFTER_L, 4);
  ledcAttachPin(CONVAYER_H, 8);
  ledcAttachPin(CONVAYER_L, 12);
}

void OC_controller::CAN_Receive(){
  // - ID_received | IncommingCommand | 
  if (CANctr->CAN_ReceiveFrom(ID_Convert) )
  {
    if (CANctr->GetByteReceived(ID_received) == ID_OC)
    {
      cmd_request = CANctr->GetByteReceived(IncommingCommand);
      cmd_resetError = CANctr->GetByteReceived(IncommingCommandReset);
      Serial.printf("Received: command - %d | reset - %d\n", cmd_request, cmd_resetError);
    }
  }
}

void OC_controller::CAN_Transmit(){
  // | CommandReceived | commandStatus | bit sensor | Error | 

  CANctr->SetByteTransmit(cmd_request, CommandReceived);
  CANctr->SetByteTransmit(getCommandStatus(), commandStatus);

  uint8_t sensorBits = 0;
  sensorBits |=  (getSensor(SENSOR_7)<<6)|(getSensor(SENSOR_6)<<5)|(getSensor(SENSOR_5)<<4)|(getSensor(SENSOR_4)<<3)|(getSensor(SENSOR_3)<<2)|(getSensor(SENSOR_2)<<1)|(getSensor(SENSOR_1)<<0);
  CANctr->SetByteTransmit(sensorBits, sensorsData);
  CANctr->SetByteTransmit(getError(), error_check);

  // Serial.println((String) "sensorBits: " + sensorBits);

  if (CANctr->CAN_Send()){

  }else{
    Serial.println("OC CAN_Send - ERROR");
  }
}

void OC_controller::CAN_send(){
  if ((millis() - preTime_sendCAN) > (1000 / FREQUENCY_sendCAN))
  {   
    preTime_sendCAN = millis();
    CAN_Transmit();
  }
}

void OC_controller::OC_loop(){
  if (pre_cmd_request != cmd_request){
    pre_cmd_request = cmd_request;
    setError(ISOK);
    stopAndReset();
  }

  if (cmd_resetError == 1){
    setError(ISOK);
    stopAndReset();
    Serial.println("OC DO - stopAndReset");
  }

  if(getError() == ISOK){ // - Neu ko co Loi
    // Serial.printf("Doing: %d \n",cmd);
    if (cmd_request == MOTOR_OC_STOP){
      stopAndReset();
      Serial.println("OC DO - MOTOR_OC_STOP");

    }else if (cmd_request == LIFT_TABLE_DOWN){
      Serial.println("OC DO - LIFT_TABLE_DOWN");
      liftDown();

    }else if (cmd_request == LIFT_TABLE_UP){
      Serial.println("OC DO - LIFT_TABLE_UP");
      liftUp();
    }else{
      // Serial.println("OC DO - NOT JOB");
    }

  }else{
    stopAndReset();
    // Serial.println("OC DO - NOT");
  }

  /* -- */
  CAN_Receive();
  /* -- */
  CAN_send();
}

void OC_controller::debuge(){

}