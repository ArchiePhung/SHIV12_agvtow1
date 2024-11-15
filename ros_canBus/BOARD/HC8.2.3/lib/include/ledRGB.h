/**
 * File : ledRGB.h
 * Version : 8.2.0
 * Date    : 10/06/2020
 * Author  : AnDX
 * Description :
 * Thu vien dieu khien led RGB 
 *******************************************************/

#ifndef __LED_RGB_
#define __LED_RGB_
#include <Arduino.h>

class ledRGB
{
    private:
        uint8_t rPin;
        uint8_t gPin;
        uint8_t bPin;
        uint8_t rChange;
        uint8_t gChange;
        uint8_t bChange;
        
    public:
        ledRGB(uint8_t _rPin,uint8_t _gPin,uint8_t _bPin,uint8_t _rChange,uint8_t _gChange,uint8_t _bChange);
        void ledRun(void);
        void ledRun2(void);
        void ledRun3(void);
        void ledRun4(void);
        void ledRun5(void);
        void ledByPWM(uint8_t,uint8_t,uint8_t);
        void ledError(void);
        void setColor(uint8_t rVal,uint8_t gVal,uint8_t bVal);

};

#endif