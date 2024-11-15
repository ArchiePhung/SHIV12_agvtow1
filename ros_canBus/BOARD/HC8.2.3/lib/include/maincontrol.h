/*==================================================================================================
*   Project              :  MAIN AI CONTROL
*   Doccument            :  ESP32S 
*   FileName             :  modbuscontrol.h
*   File Description     :  Khai bao cac ham su dung chinh
*
==================================================================================================*/
/*==================================================================================================
Revision History:
Modification     
    Author                  	Date D/M/Y     Description of Changes
----------------------------	----------     ------------------------------------------
    Do Xuan An              	30/07/2020     Tao file
----------------------------	----------     ------------------------------------------
==================================================================================================*/
#ifndef __MAIN_CONTROL_H
#define __MAIN_CONTROL_H 
/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "cancontrol.h"
#include "hcRFIDLib.h"
#include "ledRGB.h"
#include "controll_config.h"
#include "hardwareconfig.h"
#include "WiFi.h"
/*==================================================================================================
*                                        FILE VERSION 
==================================================================================================*/

/*==================================================================================================
*                                          CONSTANTS
==================================================================================================*/
 
/*==================================================================================================
*                                      DEFINES AND MACROS
==================================================================================================*/

/*==================================================================================================
*                                             ENUMS
==================================================================================================*/

/*==================================================================================================
*                                STRUCTURES AND OTHER TYPEDEFS
==================================================================================================*/

/*==================================================================================================
*                                             CLASS
==================================================================================================*/
class MainControl
{
    private:
        uint8_t rfidSelectID=0;
        uint8_t rfidReader[6]={0,0,0,0,0,0};
        uint8_t sickRespon = 0;
        uint8_t statusRev_CAN = 0;
        uint8_t sickRespon_before = 0;
        uint8_t sickRespon_after = 0;
        uint8_t ledControl = 0;
        uint8_t cam_ledControl=0;
        uint8_t errorRespon = errorStatus::ISOK;
        uint8_t sickSelectID = ID_RFID_HEAD;
        
        CAN_manager *Can_Main = new CAN_manager(CAN_SPEED_125KBPS, GPIO_NUM_5, GPIO_NUM_4, CAN_frame_std, ID_HC, 8);
        ledRGB *ledFCControlBefor = new ledRGB(18,19,23,0,1,2);
        ledRGB *ledFCControlApter = new ledRGB(25,26,27,3,4,5);
        hcRFIDLib hcRFID;
        unsigned long preTime_sendCAN = 0;
        unsigned long saveTime_checkCAN = 0;
        unsigned long pre_led1_time = 0;
    public:
        MainControl();
        ~MainControl();
        void MainInit(void);
        void CanRecevieHandle(void);
        bool CanTransmitHandle(void); 
        void LedControler(void);
        void SetError(uint8_t);
        void SickRespon(void);
        void read_SICK(void);
        void ReadRFID(void);
        void BarderShockChecker(void);
        void Run();
        void LedTest();

};
#endif /*<!__MAIN_CONTROL_H>*/
//------------------------------------------END FILE----------------------------------------------//
