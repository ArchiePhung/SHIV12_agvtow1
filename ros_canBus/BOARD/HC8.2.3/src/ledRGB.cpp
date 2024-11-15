/**
 * File : ledRGB.cpp
 * Version : 8.2.0
 * Date    : 10/06/2020
 * Author  : AnDX
 * Description :
 * Thu vien dieu khien led RGB 
 *******************************************************/

#include "ledRGB.h"
/* Define chân led */
ledRGB::ledRGB(uint8_t _rPin,uint8_t _gPin,uint8_t _bPin,uint8_t _rChange,uint8_t _gChange,uint8_t _bChange): \
rPin(_rPin),gPin(_gPin),bPin(_bPin),rChange(_rChange),gChange(_gChange),bChange(_bChange)
{
    ledcSetup(rChange,5000,8);
    ledcAttachPin(rPin,rChange);
  
    ledcSetup(gChange,5000,8);
    ledcAttachPin(gPin,gChange);

    ledcSetup(bChange,5000,8);
    ledcAttachPin(bPin,bChange);
}

/* Màu led */
void ledRGB::setColor(uint8_t bVal,uint8_t gVal,uint8_t rVal)
{
    ledcWrite(rChange, rVal);
    ledcWrite(gChange, gVal);
    ledcWrite(bChange, bVal);
}

/* Led màu xanh */
void ledRGB::ledRun(void)
{
    static uint8_t dimmer = 0;
    static bool flager = true;

    if (dimmer >= 254){
        flager = false;
    }
    else if (dimmer <= 150){
        flager = true;
    }

    if(flager){
        dimmer ++;
    }
    else{
        dimmer --;
    }
    setColor(0,dimmer,0);
    vTaskDelay(10);
}

/* Led màu da trời */
void ledRGB::ledRun2(void)
{
    static uint8_t dimmer = 0;
    static bool flager = true;

    if (dimmer >= 254){
        flager = false;
    }
    else if (dimmer <= 150){
        flager = true;
    }

    if(flager){
        dimmer ++;
    }
    else{
        dimmer --;
    }
    setColor(0,0,dimmer);
    vTaskDelay(10);
}

/* Led màu trắng */
void ledRGB::ledRun3(void)
{
    static uint8_t dimmer = 0;
    static bool flager = true;

    if (dimmer >= 254){
        flager = false;
    }
    else if (dimmer <= 150){
        flager = true;
    }

    if(flager){
        dimmer ++;
    }
    else{
        dimmer --;
    }
    setColor(dimmer,dimmer,dimmer);
    vTaskDelay(10);
}

/* Led trắng nhạt */
void ledRGB::ledRun4(void)
{
    setColor(50,50,50);
    vTaskDelay(10);
}

/* Tắt led */
void ledRGB::ledRun5(void)
{
    setColor(0,0,0);
}

/* Led đỏ*/
void ledRGB::ledError()
{
    static uint8_t dimmer = 0;
    static bool flager = true;

    if (dimmer >= 254){
        flager = false;
    }
    else if (dimmer <= 150){
        flager = true;
    }

    if(flager){
        dimmer ++;
    }
    else{
        dimmer --;
    }
    setColor(dimmer,0,0);
    vTaskDelay(10);
}

/* Led tùy chỉnh */
void ledRGB::ledByPWM(uint8_t valuePWMR,uint8_t valuePWMG, uint8_t valuePWMB){
    setColor(valuePWMR,valuePWMG,valuePWMB);
}