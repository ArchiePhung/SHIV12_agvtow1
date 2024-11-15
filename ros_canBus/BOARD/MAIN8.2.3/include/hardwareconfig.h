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
#include "CAN_config.h"

#define CONTROL_PROCESS
#define CAN_DEBUG
#define SERVER_PROCESS
#define SERVER_DEBUG

#define DEBUG 1

#define BIT_SENSOR_Lift 5 // 0
#define BIT_SENSOR_UP   1 // 1
#define BIT_SENSOR_DOWN 3 // 2

#define ID_MAIN    0x03
#define ID_Convert 0x01
#define FREQUENCY_sendCAN 5

#define timerCAN  0
#define timerCTR  20
#define timerPOW  1000

#define CAN_BAUD_SPEED  CAN_SPEED_125KBPS
#define CAN_TX          GPIO_NUM_5
#define CAN_RX          GPIO_NUM_4
#define CAN_FRAME       CAN_frame_std
#define CAN_ID          ID_MAIN
#define CAN_SEND_SIZE   8

#define ID_RFID_HEAD    1
#define ID_RFID_BEHIND  2

//------------------------------ Pin define -----------------------------
#define PIN_ADC_VOLTAGE 34
#define PIN_ADC_CURRENT 35

#define PIN_EMC_READ    13  // en = 1 -> 0
#define PIN_EMC_WRITE   27 // en = 0 
#define PIN_EMC_RESET   26 // en = 1
#define BUTTON_EMC_RES  39 // B1 - pullup/ raise -> pulldown /raise
#define BUTTON_SHUTDOWN 36 // B2 - pulldown/ raise

#define SWITCH_MC_BOARD 21 //0: MC-CAN ; 1: MC-ROS
#define SWITCH_L        19

#define PIN_L1          19
#define PIN_L2          21

#define PIN_CHARGE      14 // en = 1
#define PIN_WRITE_POWER 25 // en = 0 (low)
#define PIN_READ_PC     22 // en = 0

#define ENABLE_5v           25  // allway low //  high both to shutdown
#define ENABLE_22v          12  // allway low //

#define PIN_SOUND_SP    18  // en = 0
#define PIN_SOUND_1     15  // en = 0
#define PIN_SOUND_2     0   // en = 0
#define PIN_SOUND_3     32  // en = 0
#define PIN_SOUND_4     33  // en = 0

#define SMALLEST_BATTERY_VALUE 210
#define LOW_BATTERY_VALUE 225
// #define hsa_v  0.0729
// #define hsb_v  18.932
#define hsa_v  0.0653595
#define hsb_v  37.18954

#define hsa_c  0.9976
#define hsb_c  146.0266

// #define c_adc_set 3020
#define c_adc_set 2950
//--------------------------------------------------------------------------------------------------------------------

// //Sever connection config
#define NETWORK_NAME "STI_Vietnam_No8"
#define NETWORK_PASSWORD "66668888"
#define UDP_ADDRESS "192.168.1.31" // PC 
#define UDP_RECEIVE_PORT    8888
#define UDP_SEND_PORT       8000
#define MAXIMUM_RECEIVED_PACKET 500
#define SIZE_TRANSMIT_PACKET 30
#define SIZE_ROS_PACKET 5
#define SIZE_REPLY_PACKET 6

// #define DEBUG_BY_UDP
#define DEBUG_BY_SERIAL
#if defined(DEBUG_BY_SERIAL)
#define debugSerial Serial
#define LOG_BEGIN(BAUD) debugSerial.begin(BAUD);\
                        debugSerial.println("Log start!!  ε=ε=(づ￣ 3￣)づ  ε=ε=ε=┏(゜ロ゜;)┛ ")
#define LOG_MESS(...) debugSerial.println(__VA_ARGS__)
#define LOG_MESS_STRING(...) debugSerial.println((String) __VA_ARGS__)
#define LOG_NUM(NUM) debugSerial.println((String) #NUM + " = " + NUM)
#elif defined(DEBUG_BY_UDP)

#define LOG_BEGIN(BAUD) WiFi.onEvent(WiFiEvent);\
                        WiFi.begin(NETWORK_NAME, NETWORK_PASSWORD);\
                        logSever->beginPacket(UDP_ADDRESS, UDP_SEND_PORT);\
                        logSever->println("Logging udp - begin  ε=ε=(づ￣ 3￣)づ  ε=ε=ε=┏(゜ロ゜;)┛ ");\
                        logSever->endPacket()
#define LOG_MESS(...)   logSever->beginPacket(UDP_ADDRESS, UDP_SEND_PORT);\
                        logSever->println(__VA_ARGS__);\
                        logSever->endPacket()
#define LOG_MESS_STRING(...)    logSever->beginPacket(UDP_ADDRESS, UDP_SEND_PORT);\
                                logSever->println((String) __VA_ARGS__);\
                                logSever->endPacket()
#define LOG_NUM(NUM)    logSever->beginPacket(UDP_ADDRESS, UDP_SEND_PORT);\
                        logSever->println((String) #NUM + " = " + NUM);\
                        logSever->endPacket()
#else
#define LOG_BEGIN(BAUD)
#define LOG_MESS(...)
#define LOG_MESS_STRING(...)
#define LOG_NUM(NUM)
#endif

#endif
