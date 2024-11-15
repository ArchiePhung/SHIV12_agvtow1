#ifndef STI_BASE_CONFIG_H
#define STI_BASE_CONFIG_H

//uncomment the base you're building
#define STI_BASE DIFFERENTIAL_DRIVE
#define USE_BNO055_IMU
#define DEBUG 0
/*==================================================================================================
*                                      DEFINES AND MACROS
==================================================================================================*/
/**
* @brief          : Thiet lap hang so cho bo dieu khien
* @details        : Hang so PID trong bo dieu khien hoi tiep
* @note           : Bo dieu khien duoc tich hop boi hai vong PID a.k.a dieu khien dong toc
*/
#define K_P_1 1 // 1
#define K_I_1 0.12 // 1.2
#define K_D_1 0.03 // 0.03

#define K_P_2 1  // 1
#define K_I_2 0.12 // 0.12
#define K_D_2 0.03 // 0.03

/*
* @brie     : Thiet lap hang so cho bo dieu khien
* @details  : Thong so banh, van toc dieu khien cua AGV
*/
#define MAX_RPM 5000                   // tin hieu dieu khien cao nhat
#define COUNTS_PER_REV_1 1600         // xung encoder tren 1 vong quay truc banh xe
#define COUNTS_PER_REV_2 1600         // xung encoder tren 1 vong quay truc banh xe - fixed
#define WHEEL_DIAMETER 0.15     	  // duong kinh banh xe 150 mm
#define PWM_BITS 8                    // do phan giai cua xung PWM (cua bo dieu khien)
#define LR_WHEELS_DISTANCE 0.541      // khoang cach giua hai banh xe - fixed
#define FR_WHEELS_DISTANCE 0   // distance between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN
// Thong so tin hieu dieu khien
// #define PWM_MAX (pow(2, PWM_BITS) - 1)
// #define PWM_MIN (-PWM_MAX)
#define PWM_MAX 250
#define PWM_MIN -250
/*
* @brief    : Thiet lap hang so cho bo dieu khien
* @details  : Define chan I/O cua IC (ESP32)
* @note     : Su dung driver BLH5100 dieu khien dong co [ 1: dong co trai || 2: dong co phai ]
*/
// ------------ENCODER -----------------------------//
// ENCODER PINS 2
#define MOTOR1_ENCODER_A 23
#define MOTOR1_ENCODER_B 18 
// ENCODER PINS 1
#define MOTOR2_ENCODER_A 33
#define MOTOR2_ENCODER_B 19

// -----------Driver Engine: BLHD100k -------------//
// theo chân của PCF
// 100W
	// #define ENABLE_PIN 4
	// #define BRAKE_PIN  5
	// #define RESET_ALARM_PIN 0  // Reset canh bao:
	// #define ROTATION_PIN1 7
	// #define ROTATION_PIN2 6
// 200W
#define FWD2  4
#define REV2  7
#define FWD1  5
#define REV1  27 // theo chan ESP

// I/O
// #define PCF_address 0x20
#define PCF_SDA 21
#define PCF_SCL 22
#define SPEED_OUT_PIN2 25
#define SPEED_OUT_PIN1 26
#define ALARM_IN_PIN2  34   // Canh bao
#define ALARM_IN_PIN1  35 

// -------------- Safety ------------------//
#define ONEBIT_PIN 39 
#define EMC_PIN 36 
// ---------------IMU: BNO005 -------------//
// #define IMU_address
#define IMU_SDA 21
#define IMU_SCL 22

//--------------- HZ check ---------------//
#define PUB_STATUS_FREQUENCY 5 // hz tần số pub dữ liệu trạng thái Main - Max: 20
#define CONTROL_FREQUENCY 35 	// hz tần số lấy mẫu encoder và pub vel , max 31hz
#define CHECK_FREQUENCY 2 		// hz tần số kiểm tra kết nói giữa MC 8.2 vs PC.
#define CHECK_DIS_PHYSICAL 2 		// hz tần số kiểm tra kết nói giữa MC 8.2 vs PC.

#define ENCODER_USE_INTERRUPTS // mega 2560

#define ENB_CONFIG 0 // Config parameter: Kalman filter and PID.

#endif /*AGV_CONTROLLER_CONFIG_H*/
