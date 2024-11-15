#ifndef OC_LIFT_H
#define OC_LIFT_H

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

class OC_LIFT
{
	private:
		// Pin 
		int EN_LIFTER_ = 0;
		int LIFTER_H_ = 0;
		int LIFTER_L_ = 0;
		int SENSOR_UP_ = 0;
		int SENSOR_DOWN_ = 0;
		int SENSOR_LIFT_ = 0;

		int timeCheckError_ = 0;

		float timeStart_LiftUp = 0;  	// Lưu thời gian bắt đầu nâng.
		float timeStart_LiftDown = 0;	// Lưu thời gian bắt đầu hạ.
		int channelPWM_LiftUp = 0;
		int channelPWM_LiftDown = 4;
		int step_LiftUp;	// Lưu bước nâng bàn nâng.
		int step_LiftDown;	// Lưu bước hạ bàn nâng.
		int speed_LiftUp;	// Lưu tốc độ nâng bàn nâng.	
		int speed_LiftDown;	// Lưu tốc độ nâng hạ nâng.

	public:
		OC_LIFT(int EN_LIFTER, int LIFTER_H, int LIFTER_L, int SENSOR_UP, int SENSOR_DOWN, int SENSOR_LIFT, int timeCheckError);
		~OC_LIFT();
		void setAll();
		
		int liftUp();
		int liftDown();
		int readSensorLift();
		int readSensorUp();
		int readSensorDown();
		void stopAndReset();
		void test_liftDown();
		void test_liftUp();
};


#endif