/**
 * File : hcRFIDLib.cpp
 * Version : 8.2.0
 * Date    : 10/06/2020
 * Author  : AnDX
 * Description :
 * Dinh nghia thu vien su dung RFID 
 *******************************************************/

#include "../include/hcRFIDLib.h"

#define SERIAL_WRITE_BYTE(...) hcRFIDSerial.write(__VA_ARGS__)
#define SERIAL_WRITE_WORD(v) SERIAL_WRITE_BYTE((uint8_t)(v & 0xFF)) ; SERIAL_WRITE_BYTE((uint8_t)(v>>8));

#define GET_COMMAND_PACKET(...)                     \
    uint8_t data[] = {__VA_ARGS__};                 \
    hcRFIDLib_Packet packet(sizeof(data), data);    \
    hcRS485DirIsTx();                               \
    writeStructuredPacket(packet);                  \
    hcRFIDSerial.flush();                           \
    hcRS485DirIsRx();                               \
    uint8_t Respont = getStructuredPacket(&packet); \
    if (Respont != ISOK) return Respont;

#define SEND_COMMAND_PACKET(...)     \
    GET_COMMAND_PACKET(__VA_ARGS__); \
    return packet.data[5];

hcRFIDLib::hcRFIDLib()
{
    
}

/* Đặt ID giao tiếp (đầu trước hoặc đầu sau) */
void hcRFIDLib::setIdDevice(uint8_t _id)
{
    idDevice = _id;
}

/* Setup chân giao tiếp */
void hcRFIDLib::hcRFIDInit(void)
{
    #ifdef DEBUG_SERIAL_ENABLE
	    hcDebugSrial.begin(115200);
        hcDebugSrial.println("Preparing");
    #endif
    pinMode(hcRS485Dir,OUTPUT);
    hcRFIDSerial.begin(19200, SERIAL_8E1, 13,14);
}

/* Gửi dữ liệu */
void hcRFIDLib::writeStructuredPacket(const hcRFIDLib_Packet &packet)
{
    SERIAL_WRITE_BYTE(idDevice);
    // hcDebugSrial.print("-> 0x"); hcDebugSrial.println((uint8_t)(idDevice), HEX);
    for(uint8_t i = 0; i< packet.length; i++) {
        SERIAL_WRITE_BYTE(packet.data[i]);
    }
    uint8_t dataSend[100];
    dataSend[0] = idDevice;
    memcpy(&dataSend[1],packet.data,packet.length);
    uint16_t CheckSum = hcRFIDCheckSum(dataSend,packet.length+1);
    SERIAL_WRITE_WORD(CheckSum);
}

/* Gửi và nhận phản hồi của đầu đọc, lưu vào chuỗi */
uint8_t hcRFIDLib::getStructuredPacket(hcRFIDLib_Packet * packet, uint16_t timeout)
{
    uint8_t byte;
    uint16_t idx=0, timer=0;
    hcRS485DirIsRx();
    while(!hcRFIDSerial.available()) 
    {
        delay(1);
        // delay(3);
        timer++;
        if( timer >= timeout) 
        {
            #ifdef DEBUG_SERIAL_ENABLE
	            hcDebugSrial.println("Timed out");
            #endif
	        return HC_RFID_TIMEOUT;
        }
    }
    while(hcRFIDSerial.available())
    {
        byte = hcRFIDSerial.read();
        #ifdef DEBUG_SERIAL_ENABLE
            //hcDebugSrial.print("-> 0x"); hcDebugSrial.println(byte, HEX);
        #endif
        packet->data[idx] = byte;
        idx++;
    }
    if(packet->data[0] != idDevice) return HC_RFID_REV_FAIL;
    uint16_t CheckSum = hcRFIDCheckSum(packet->data,idx-2);
    if(CheckSum != (((uint16_t)packet->data[idx-1] << 8)|(packet->data[idx-2]))) 
    {
        #ifdef DEBUG_SERIAL_ENABLE
            hcDebugSrial.println("Loi Check Sum");
        #endif
        return HC_RFID_CRCERROR;
    }
    else return ISOK;  
}

/* Đặt mode cho đầu đọc thẻ */
uint8_t hcRFIDLib::setModeRFID(hcRFIDMode _mode)
{
    SEND_COMMAND_PACKET(HC_RFID_FUNCTION_WRITE_SIGLE_REG,0x00,0x00,0x00,(uint8_t)_mode);
}

/* Đặt ID cho đầu đọc thẻ */
uint8_t hcRFIDLib::setRFIDID(uint8_t _id)
{
    SEND_COMMAND_PACKET(HC_RFID_FUNCTION_WRITE_SIGLE_REG,0x00,0x01,0xA0,_id);
}

/* Gửi request và nhận phản hồi của đầu đọc thẻ, xử lí thông tin */
uint8_t hcRFIDLib::getCardNumberData(uint8_t *CardNum)
{
    GET_COMMAND_PACKET(HC_RFID_FUNCTION_READ_DATA,0x00,0x0E,0x00,0x07);
    if(packet.data[1] != HC_RFID_FUNCTION_READ_DATA || packet.data[2] != HC_RFID_FUNCTION_REQUIRE_DATA)
    {
        // Serial.println("Doc Loi Du Lieu");
        return HC_RFID_IS_FAIL;
    }
    memcpy(CardNum,&packet.data[5],5); /* Lưu dữ liệu thẻ */
    return ISOK;
}
