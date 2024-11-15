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

#define ID_HC       0x02
#define ID_Convert  0x01

#define ID_RFID_HEAD    1
#define ID_RFID_BEHIND  2

#define BADERSHOCK_ISR  34
#define FREQUENCY_sendCAN 12

#endif