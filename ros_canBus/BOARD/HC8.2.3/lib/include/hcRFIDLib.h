/**
 * File : hcRFIDLib.h
 * Version : 8.2.0
 * Date    : 10/06/2020
 * Author  : AnDX
 * Description :
 * Thu vien su dung RFID 
 *******************************************************/

#ifndef __HCRFIDLIB_H
#define __HCRFIDLIB_H

#include <Arduino.h>
#include "HcConfig.h"
#include "controll_config.h"

#define DEFAULT_TIMEOUT            1000

// #define HC_RFID_IS_FAIL            0x02
// #define HC_RFID_PACKETRECIEVEERR   0x03
// #define HC_RFID_TIMEOUT            0x04
// #define HC_RFID_REV_FAIL           0x05
// #define HC_RFID_BADPACKET          0x06
// #define HC_RFID_CRCERROR           0x07

#define HC_RFID_FUNCTION_READ_DATA          0x03
#define HC_RFID_FUNCTION_WRITE_SIGLE_REG    0x06
#define HC_RFID_FUNCTION_REQUIRE_DATA       0xE

typedef enum
{
    ANTENISOFF_MASTERSLAVE_MODE = 0x00,
    ANTENISOFF_SLAVEAVTIVESEND_MODE = 0x01,
    ANTENISON_MASTERSLAVE_MODE = 0x02,
    ANTENISON_SLAVEAVTIVESEND_MODE = 0x03
} hcRFIDMode;

struct hcRFIDLib_Packet
{
    hcRFIDLib_Packet(uint8_t length,uint8_t *data)
    {
        this->length = length;
        if(length < 100)
        {
            memcpy(this->data, data, length);
        }
        else memcpy(this->data, data, 100);
    }
    uint8_t length;
    uint8_t data[100];
};
class hcRFIDLib
{
    public:
        hcRFIDLib();
        void hcRFIDInit(void);
        void setIdDevice(uint8_t _id);
        void writeStructuredPacket(const hcRFIDLib_Packet &packet);
        uint8_t getStructuredPacket(hcRFIDLib_Packet * packet, uint16_t timeout = DEFAULT_TIMEOUT);
        uint8_t setModeRFID(hcRFIDMode _mode);
        uint8_t setRFIDID(uint8_t _id);
        uint8_t getCardNumberData(uint8_t *CardNum);
    private:
        
        void hcRS485DirIsTx(void)
        {
             digitalWrite(hcRS485Dir,HIGH);
        };
        void hcRS485DirIsRx(void)
        {
            digitalWrite(hcRS485Dir,LOW);
        }
        uint16_t hcRFIDCheckSum(uint8_t *cBuffer,uint16_t iBufLen)
        {
            unsigned int i, j;  
	        unsigned int wPolynom = 0xa001;
	        unsigned int wCrc	  =	0xffff;
            for (i = 0; i < iBufLen; i++)
            {
                wCrc ^= cBuffer[i];
                for (j = 0; j < 8; j++)
                {
                    if (wCrc &0x0001)
                    {
                        wCrc = (wCrc >> 1) ^ wPolynom;
                    }
                    else
                    {
                         wCrc = wCrc >> 1;
                    }
                }
            }
            return wCrc;
        }
        private:
        uint8_t idDevice = 0;
};




#endif