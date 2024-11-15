/*==================================================================================================
*   Project              :  MAIN AI CONTROL
*   Doccument            :  ESP32S 
*   FileName             :  maincontrol.cpp
*   File Description     :  Dinh nghia cac ham thu vien su dung trong maincontrol.h
*
==================================================================================================*/
/*==================================================================================================
Revision History:
Modification     
    Author                  Date D/M/Y     Description of Changes
------------------------    -----------    ---------------------------------------------------------
    Do Xuan An              30/07/2020     Tao file
------------------------    -----------    ---------------------------------------------------------
==================================================================================================*/
/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "maincontrol.h"
#include "hardwareconfig.h"
#include "controll_config.h"
/*==================================================================================================
*                                     FILE VERSION CHECKS
==================================================================================================*/

/*==================================================================================================
*                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/
/*==================================================================================================
*                                       LOCAL MACROS
==================================================================================================*/
/*==================================================================================================
*                                      LOCAL CONSTANTS
==================================================================================================*/
/*==================================================================================================
*                                      LOCAL VARIABLES
==================================================================================================*/

/*==================================================================================================
*                                      GLOBAL CONSTANTS
==================================================================================================*/

/*==================================================================================================
*                                      GLOBAL VARIABLES
==================================================================================================*/
PCF8574 pcf8574(0x20);
// PCF8574 pcf8574(0x38);
uint8_t status_blsock;
/*==================================================================================================
*                                   LOCAL FUNCTION PROTOTYPES
==================================================================================================*/
/*==================================================================================================
*                                        LOCAL FUNCTION 
==================================================================================================*/
/*==================================================================================================
*                                      GLOBAL FUNCTIONS
==================================================================================================*/
MainControl::MainControl()
{
    sickRespon = 0;
}

MainControl::~MainControl()
{

}

/* Báo lỗi */
void MainControl::SetError(uint8_t err){
    errorRespon = err;
}

/* Setup chân điều khiển */
void MainControl::MainInit() {
    WiFi.mode(WIFI_OFF);
    btStop();
    delay(10);
    pcf8574.begin();
    pinMode(BADERSHOCK_ISR, INPUT_PULLUP);
    delay(10);
    Can_Main->CAN_prepare();
    LOG_MESS("Begin");
}

/* Hàm nhận CAN */
void MainControl::CanRecevieHandle() {
    if(Can_Main->CAN_ReceiveFrom(ID_Convert)) {
        if(ID_HC == Can_Main->GetByteReceived(0)){
            saveTime_checkCAN = millis();
            if (MODE_RESET_ALARM == Can_Main->GetByteReceived(1)){
                errorRespon = errorStatus::ISOK;
            }
            else {
                ledControl = Can_Main->GetByteReceived(1);
            }
            // LOG_NUM(ledControl);
            // LOG_NUM(cam_ledControl);
        }
    }

    if ((millis() - saveTime_checkCAN) > 1000 )
    {   
        statusRev_CAN = 0;
    }else{
        statusRev_CAN = 1;
    }
}

/* Loop điều khiển LED */
void MainControl::LedControler()
{
    static uint8_t dimmer = 0;
    static bool flager = true;

    if (ledControl == 1){ // error : sang nhanh, nhay do 
        if ((millis() - pre_led1_time) > 1)
        {
            pre_led1_time = millis();
            if (dimmer >= 254){
                flager = false;
            }
            else if (dimmer <= 1){
                flager = true;
            }

            if(flager){
                dimmer ++;
            }
            else{
                dimmer --;
            }

            ledFCControlBefor->setColor(dimmer, 0, 0);
            ledFCControlApter->setColor(dimmer, 0, 0);
        }
    }
    if (ledControl == 2){ // di chuyen : sang cham ,xanh la 
        if ((millis() - pre_led1_time) > 10)
        {
            pre_led1_time = millis();
            if (dimmer >= 254){
                flager = false;
            }
            else if (dimmer <= 1){
                flager = true;
            }

            if(flager){
                dimmer ++;
            }
            else{
                dimmer --;
            }

            ledFCControlBefor->setColor(0, dimmer, 0);
            ledFCControlApter->setColor(0, dimmer, 0);
        }
    }
    if (ledControl == 3){ // tag : sang nhanh , xanh la 
        if ((millis() - pre_led1_time) < 200)
        {   
            ledFCControlBefor->setColor(0, 254, 0);
            ledFCControlApter->setColor(0, 254, 0);
        }
        if (((millis() - pre_led1_time) > 200) && (millis() - pre_led1_time) < 300)
        {   
            ledFCControlBefor->setColor(0, 0, 0);
            ledFCControlApter->setColor(0, 0, 0);

        }
        if ((millis() - pre_led1_time) > 300)
        {   
            pre_led1_time = millis();
        }


    }
    if (ledControl == 4){ // nang, ha : sang cham , xanh duong 
        if ((millis() - pre_led1_time) > 10)
        {
            pre_led1_time = millis();
            if (dimmer >= 254){
                flager = false;
            }
            else if (dimmer <= 1){
                flager = true;
            }

            if(flager){
                dimmer ++;
            }
            else{
                dimmer --;
            }

            ledFCControlBefor->setColor(0, dimmer, dimmer);
            ledFCControlApter->setColor(0, dimmer, dimmer);
        }
    }
    
    if (ledControl == 5){ // sac: sang cham , vang 
        if ((millis() - pre_led1_time) > 10)
        {
            pre_led1_time = millis();
            if (dimmer >= 254){
                flager = false;
            }
            else if (dimmer <= 1){
                flager = true;
            }

            if(flager){
                dimmer ++;
            }
            else{
                dimmer --;
            }

            ledFCControlBefor->setColor(dimmer, dimmer, 0);
            ledFCControlApter->setColor(dimmer, dimmer, 0);
        }
    }
    if (ledControl == 6){ //
        if ((millis() - pre_led1_time) < 200)
        {   
            ledFCControlBefor->setColor(254, 0, 254);
            ledFCControlApter->setColor(254, 0, 254);
        }
        if (((millis() - pre_led1_time) > 200) && (millis() - pre_led1_time) < 300)
        {   
            ledFCControlBefor->setColor(0, 0, 0);
            ledFCControlApter->setColor(0, 0, 0);
        }
        if ((millis() - pre_led1_time) > 300)
        {   
            pre_led1_time = millis();
        }
    }
    if (ledControl == 7){ //
        if ((millis() - pre_led1_time) < 200)
        {   
            ledFCControlBefor->setColor(254, 254, 0);
            ledFCControlApter->setColor(254, 254, 0);
        }
        if (((millis() - pre_led1_time) > 200) && (millis() - pre_led1_time) < 300)
        {   
            ledFCControlBefor->setColor(0, 0, 0);
            ledFCControlApter->setColor(0, 0, 0);
        }
        if ((millis() - pre_led1_time) > 300)
        {   
            pre_led1_time = millis();
        }
    }
    if (ledControl == 8){ //
        if ((millis() - pre_led1_time) > 10)
        {
            pre_led1_time = millis();
            if (dimmer >= 254){
                flager = false;
            }
            else if (dimmer <= 1){
                flager = true;
            }

            if(flager){
                dimmer ++;
            }
            else{
                dimmer --;
            }

            ledFCControlBefor->setColor(dimmer, 0, dimmer);
            ledFCControlApter->setColor(dimmer, 0, dimmer);
        }
    }
}

/* Hàm gửi CAN */
bool MainControl::CanTransmitHandle()
{    
    // int Error = ISOK ;
    // LOG_NUM(status_blsock);
    Can_Main->SetByteTransmit(statusRev_CAN, 3);
    Can_Main->SetByteTransmit(sickRespon_before, 4);
    Can_Main->SetByteTransmit(sickRespon_after, 5);
    Can_Main->SetByteTransmit(status_blsock, 6);
    Can_Main->SetByteTransmit(0, 7);

    if(Can_Main->CAN_Send())
    {
        // LOG_MESS_STRING("CAN_Send OK");
        // LOG_NUM(sickRespon_before);
        return true;
    }
    // LOG_MESS_STRING("CAN_Send NOT OK");
    return false;
}

/* Loop test led */
void MainControl::LedTest(){
    LOG_MESS_STRING("Start Test");
    ledFCControlApter->ledByPWM(255,0,0);
    delay(3000);
    ledFCControlApter->ledByPWM(0,255,0);
    delay(3000);
    ledFCControlApter->ledByPWM(0,0,255);
    delay(3000);
    ledFCControlApter->ledByPWM(255,255,255);
    delay(3000);
    ledFCControlApter->ledByPWM(0,0,0);
    delay(3000);
}

/* Đọc giá trị chân cảm biến vùng SICK */
void MainControl::SickRespon()
{
    uint8_t sickTemplate = signals::SickField_nothg;
    if(sickSelectID == ID_RFID_HEAD)
    {
        // sickRespon = (pcf8574.digitalRead(hcSickTopPin1In) << 3) | (pcf8574.digitalRead(hcSickTopPin2In) << 2) | (pcf8574.digitalRead(hcSickTopPin3In) << 1) | pcf8574.digitalRead(hcSickTopPin4In);

        // sickTemplate = (pcf8574.digitalRead(hcSickTopPin1In) << 3) | (pcf8574.digitalRead(hcSickTopPin2In) << 2) | (pcf8574.digitalRead(hcSickTopPin3In) << 1) | pcf8574.digitalRead(hcSickTopPin4In);
        // LOG_MESS("TOP");
        // LOG_NUM(sickTemplate);

        // sickTemplate = (pcf8574.digitalRead(hcSickBotPin1In) << 3) | (pcf8574.digitalRead(hcSickBotPin2In) << 2) | (pcf8574.digitalRead(hcSickBotPin3In) << 1) | pcf8574.digitalRead(hcSickBotPin4In);
        sickTemplate=signals::SickField_safe;
        // LOG_MESS("BOT");
        // LOG_NUM(sickTemplate);
    }
    else if(sickSelectID == ID_RFID_BEHIND)
    {
        // sickTemplate = (pcf8574.digitalRead(hcSickTopPin1In) << 3) | (pcf8574.digitalRead(hcSickTopPin2In) << 2) | (pcf8574.digitalRead(hcSickTopPin3In) << 1) | pcf8574.digitalRead(hcSickTopPin4In);
        // sickTemplate = (pcf8574.digitalRead(hcSickBotPin1In) << 3) | (pcf8574.digitalRead(hcSickBotPin2In) << 2) | (pcf8574.digitalRead(hcSickBotPin3In) << 1) | pcf8574.digitalRead(hcSickBotPin4In);
        sickTemplate=signals::SickField_safe;
        // LOG_NUM(sickTemplate);
    }
    else{
        sickTemplate = signals::SickField_nothg;
        // LOG_NUM(sickTemplate);
    }

    if(sickTemplate & 0b0001) { sickRespon = signals::SickField_stop; }
    else if(sickTemplate & 0b0010) { sickRespon = signals::SickField_warn; }
    else if(sickTemplate & 0b0100) { sickRespon = signals::SickField_safe; }
    else if(sickTemplate & 0b1000) { sickRespon = signals::SickField_nothg; }
    else { sickRespon = signals::SickField_nothg; }
}

void MainControl::read_SICK(){
    bool z1t = 0;
    bool z2t = 0;
    bool z3t = 0;
    bool z1s = 0;
    bool z2s = 0;
    bool z3s = 0;

    z2t = pcf8574.readButton(hcSickTopPin1In);
    z1t = pcf8574.readButton(hcSickTopPin2In);
    z3t = pcf8574.readButton(hcSickTopPin3In);

    if(z1t == 0){
        sickRespon_before = 1;
    }else if (z2t == 0){
        sickRespon_before = 2;
    }else if (z3t == 0){
        sickRespon_before = 3;
    }else{
        sickRespon_before = 0;
    }
    
    z1s = pcf8574.readButton(hcSickBotPin1In);
    z2s = pcf8574.readButton(hcSickBotPin2In);
    z3s = pcf8574.readButton(hcSickBotPin3In);

    if(z1s == 0){
        sickRespon_after = 1;
    }else if (z2s == 0){
        sickRespon_after = 2;
    }else if (z3s == 0){
        sickRespon_after = 3;
    }else{
        sickRespon_after = 0;
    }

    // Serial.print(z1t);
    // Serial.print(z2t);
    // Serial.print(z3t);
    // Serial.print(" | ");
    // Serial.print(z1s);
    // Serial.print(z2s);
    // Serial.println(z3s);
}

/* Kiểm tra lỗi badershock */
void MainControl::BarderShockChecker(){ 
    if(LOW == digitalRead(BADERSHOCK_ISR)){
        SetError(errorStatus::BREAK_BADERSHOCK);
        status_blsock = 1;
    }else{
        status_blsock = 0;
    }
}

/* Loop điều khiển HC */
void MainControl::Run()
{
    // -- SEND
    if ((millis() - preTime_sendCAN) > (1000 / FREQUENCY_sendCAN)){   
        preTime_sendCAN = millis();
        BarderShockChecker();
        read_SICK();
        CanTransmitHandle();
    }
    // -- 
    CanRecevieHandle();
    // -- 
    LedControler();
}
//------------------------------------------END FILE----------------------------------------------//
