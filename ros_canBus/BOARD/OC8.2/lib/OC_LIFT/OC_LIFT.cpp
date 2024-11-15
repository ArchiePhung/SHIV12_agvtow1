#include "OC_LIFT.h"

OC_LIFT::OC_LIFT(int EN_LIFTER, int LIFTER_H, int LIFTER_L, int SENSOR_UP, int SENSOR_DOWN, int SENSOR_LIFT, int timeCheckError)
{
    EN_LIFTER_ = EN_LIFTER;
	LIFTER_H_ = LIFTER_H;
	LIFTER_L_ = LIFTER_L;
	SENSOR_UP_ = SENSOR_UP;
	SENSOR_DOWN_ = SENSOR_DOWN;
	SENSOR_LIFT_ = SENSOR_LIFT;
	timeCheckError_ = timeCheckError;
}

OC_LIFT::~OC_LIFT()
{

}

void OC_LIFT::setAll() 
{
	pinMode(SENSOR_UP_, INPUT_PULLUP);
	pinMode(SENSOR_DOWN_, INPUT_PULLUP);
	pinMode(SENSOR_LIFT_, INPUT);

	pinMode(EN_LIFTER_, OUTPUT);
	pinMode(LIFTER_H_, OUTPUT);
	pinMode(LIFTER_L_, OUTPUT);

	digitalWrite(EN_LIFTER_, LOW);
	digitalWrite(LIFTER_H_, LOW);
	digitalWrite(LIFTER_L_, LOW);

	//PWM Steup
	//ledcSetup(ledChannel, freq, resolution);
	ledcSetup(channelPWM_LiftUp, 1000, 10);
	ledcSetup(channelPWM_LiftDown, 1000, 10);

	//ledcAttachPin(ledPin, ledChannel);
	ledcAttachPin(LIFTER_H_, channelPWM_LiftUp);
	ledcAttachPin(LIFTER_L_, channelPWM_LiftDown);
}

void OC_LIFT::test_liftUp()
{
	digitalWrite(EN_LIFTER_, HIGH);
	digitalWrite(LIFTER_L_, LOW);
	digitalWrite(LIFTER_H_, HIGH);
	ledcWrite(channelPWM_LiftUp, 1000);	
}

void OC_LIFT::test_liftDown()
{
	digitalWrite(EN_LIFTER_, HIGH);
	digitalWrite(LIFTER_L_, HIGH);
	digitalWrite(LIFTER_H_, LOW);
	ledcWrite(channelPWM_LiftDown, 1000);	
}

int OC_LIFT::liftUp()
{
   //Quay thuan
	if (step_LiftUp == 0){ // điều khiển chiều.
		// Serial.println ("BN quay nguoc");  
		timeStart_LiftUp = millis();
		digitalWrite(EN_LIFTER_, HIGH);
		digitalWrite(LIFTER_L_, LOW);
		speed_LiftUp = 0;
		ledcWrite(channelPWM_LiftUp, speed_LiftUp);
		step_LiftUp = 1;

	}else if (step_LiftUp == 1){ // tăng dần tốc độ.		
		if (digitalRead(SENSOR_UP_) == HIGH){ // Bắt được cảm biến giới hạn.
			step_LiftUp = 3;
		}

		if (speed_LiftUp >= 1022){ // Đạt tốc độ giới hạn.
			step_LiftUp = 2;
			speed_LiftUp = 1022;
		}else{						// Tăng dần tốc độ.
			speed_LiftUp += 2;
			ledcWrite(channelPWM_LiftUp, speed_LiftUp);
			delay(1);
		}

	}else if (step_LiftUp == 2){ // Đợi bắt được cảm biến.
		if(millis() - timeStart_LiftUp > timeCheckError_){
			speed_LiftUp = 0;
			ledcWrite(channelPWM_LiftUp, speed_LiftUp);
			return -1;
		}

		if (digitalRead(SENSOR_UP_) == HIGH){ // Bắt được cảm biến giới hạn.
			speed_LiftUp = 0;
			step_LiftUp = 3;
		}

	}else if (step_LiftUp == 3){ // Giảm dần tốc độ về 0.
		speed_LiftUp -= 600;
		if(speed_LiftUp <= 0){
			speed_LiftUp = 0;
			step_LiftUp = 4;
			digitalWrite(EN_LIFTER_, LOW);
		}
		
		ledcWrite(channelPWM_LiftUp, speed_LiftUp);
		 
	}else if (step_LiftUp == 4){ // Hoàn thành.	
		return 1;
	}

	return 0;
}

int OC_LIFT::liftDown()
{
	if (step_LiftDown == 0){ // điều khiển chiều.
		// Serial.println ("BN quay nguoc");  
		timeStart_LiftDown = millis();
		digitalWrite(EN_LIFTER_, HIGH);
		digitalWrite(LIFTER_H_, LOW);
		speed_LiftDown = 0;
		ledcWrite(channelPWM_LiftDown, speed_LiftDown);
		step_LiftDown = 1;

	}else if (step_LiftDown == 1){ // tăng dần tốc độ.		
		if (digitalRead(SENSOR_DOWN_) == HIGH){ // Bắt được cảm biến giới hạn.
			step_LiftDown = 3;
		}

		if (speed_LiftDown >= 1022){ // Đạt tốc độ giới hạn.
			step_LiftDown = 2;
			speed_LiftDown = 1022;
		}else{						// Tăng dần tốc độ.
			speed_LiftDown += 2;
			ledcWrite(channelPWM_LiftDown, speed_LiftDown);
			delay(1);
		}

	}else if (step_LiftDown == 2){ // Đợi bắt được cảm biến.
		if(millis() - timeStart_LiftDown > timeCheckError_){
			speed_LiftDown = 0;
			ledcWrite(channelPWM_LiftDown, speed_LiftDown);
			return -1;
		}

		if (digitalRead(SENSOR_DOWN_) == HIGH){ // Bắt được cảm biến giới hạn.
			speed_LiftDown = 0;
			step_LiftDown = 3;
		}

	}else if (step_LiftDown == 3){ // Giảm dần tốc độ về 0.
		speed_LiftDown -= 600;
		if(speed_LiftDown <= 0){
			speed_LiftDown = 0;
			step_LiftDown = 4;
			digitalWrite(EN_LIFTER_, LOW);
		}
		ledcWrite(channelPWM_LiftDown, speed_LiftDown);
		 
	}else if (step_LiftDown == 4){ // Hoàn thành.	
		return 1;
	}

	return 0;
}

void OC_LIFT::stopAndReset(){
	digitalWrite(EN_LIFTER_, LOW);
	digitalWrite(LIFTER_H_, LOW);
	digitalWrite(LIFTER_L_, LOW);
	ledcWrite(channelPWM_LiftUp, 0);
	ledcWrite(channelPWM_LiftDown, 0);
	speed_LiftDown = 0;
	speed_LiftUp = 0;
	step_LiftUp = 0;
	step_LiftDown = 0;
}

int OC_LIFT::readSensorLift(){
	if (digitalRead(SENSOR_LIFT_) == 0){
		return 0;
	}else{
		return 1;
	}
}

int OC_LIFT::readSensorUp(){
	if (digitalRead(SENSOR_UP_) == 0){
		return 0;
	}else{
		return 1;
	}
}

int OC_LIFT::readSensorDown(){
	if (digitalRead(SENSOR_DOWN_) == 0){
		return 0;
	}else{
		return 1;
	}
}