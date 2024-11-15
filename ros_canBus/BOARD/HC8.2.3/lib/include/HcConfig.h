/**
 * File : HcConfig.h
 * Version : 8.2.0
 * Date    : 10/06/2020
 * Author  : AnDX
 * Description :
 * Dinh nghia phan cung su dung trong mach HC 
 *******************************************************/

#ifndef __HCCONFIG__
#define __HCCONFIG__

#include <HardwareSerial.h>
#include "PCF8574.h"

// #define DEBUG_SERIAL_ENABLE

#ifdef DEBUG_SERIAL_ENABLE 
#define hcDebugSrial Serial
#endif

#define hcRFIDSerial Serial1

#define hcRS485Dir      15

#define hcSickTopPin1In 3
#define hcSickTopPin2In 2
#define hcSickTopPin3In 1
#define hcSickTopPin4In 0
#define hcSickTopSelOut 33

#define hcSickBotPin1In 6
#define hcSickBotPin2In 7
#define hcSickBotPin3In 5
#define hcSickBotPin4In 4
#define hcSickBotSelOut 32

#endif
