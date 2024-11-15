/**
 * File : hardwareconfig.h
 * Version : 8.2.0
 * Date    : 17/06/2020
 * Author  : AnDX
 * Update: Quang AI
 * Description :
 * 
 *******************************************************/

#ifndef __HARDWARECONFIG_H
#define __HARDWARECONFIG_H

#include <Arduino.h>
#include "HardwareSerial.h"
#include "CAN_config.h"

#define DEBUG_BY_SERIAL
#if defined(DEBUG_BY_SERIAL)
#define debugSerial Serial
#define LOG_BEGIN(BAUD) debugSerial.begin(BAUD);\
                        debugSerial.println("Log start!!  ε=ε=(づ￣ 3￣)づ  ε=ε=ε=┏(゜ロ゜;)┛ ")
#define LOG_MESS(...) debugSerial.println(__VA_ARGS__)
#define LOG_MESS_STRING(...) debugSerial.println((String) __VA_ARGS__)
#define LOG_NUM(NUM) debugSerial.println((String) #NUM + " = " + NUM)
#else
#define LOG_BEGIN(BAUD)
#define LOG_MESS(...)
#define LOG_MESS_STRING(...)
#define LOG_NUM(NUM)
#endif

#define ID_OC      0x04
#define ID_Convert 0x01
#define FREQUENCY_sendCAN 5

#define CAN_BAUD_SPEED  CAN_SPEED_125KBPS
#define CAN_TX          GPIO_NUM_5
#define CAN_RX          GPIO_NUM_4
#define CAN_FRAME       CAN_frame_std
#define CAN_ID          ID_OC
#define CAN_SEND_SIZE   8

#define CAN_TIMER       60

#define TIME_OUT    50000
#define SENSOR_1    13 //
#define SENSOR_2    12 //
#define SENSOR_3    15 //
#define SENSOR_4    18 //
#define SENSOR_5    21 //
#define SENSOR_6    22 //
#define SENSOR_7    23 //
#define E1          19 //
#define EN_LIFTER   14 //
#define EN_CONVAYER 25 //
#define LIFTER_H    26 //1 or 0
#define LIFTER_L    27 //0 or 1
#define CONVAYER_H  33 //1 or 0
#define CONVAYER_L  32 //0 or 1

#define CHECKING_RACK_SR SENSOR_1
#define LIFTER_UP_SR     SENSOR_4
#define LIFTER_DOWN_SR   SENSOR_3

#endif
