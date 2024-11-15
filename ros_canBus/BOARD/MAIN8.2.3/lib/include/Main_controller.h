#ifndef MAIN_CONTROLLER_H
#define MAIN_CONTROLLER_H

#include "hardwareconfig.h"
#include "controll_config.h"
#include "CAN_manager.h"
#include <WiFi.h>  

struct Control_Main{ // Main
    uint8_t CAN_address;
    bool sound_enb = 0;
    uint8_t sound_type = 0;
    bool charge_write = 0;
    bool EMG_reset = 0;
    bool EMG_write = 0;
    
    Control_Main(uint8_t adr) {CAN_address = adr;}
    ~Control_Main(){}
};

struct Status_Main {
    uint8_t CAN_address;

    uint8_t byte0_analogVoltage = 0;
    uint8_t byte1_analogVoltage = 0;
    uint8_t byte0_analogCurrent = 0;
    uint8_t byte1_analogCurrent = 0;
    bool stsButton_reset = 0;
    bool stsButton_power = 0;
    uint8_t stsButton = 0;
    bool EMG_status = 0;
    bool stsRev_CAN = 0;
    Status_Main(uint8_t adr) {CAN_address = adr;}
    ~Status_Main(){}
};

class Main_controller
{
    private:
        
        CAN_manager* CAN_main = NULL;

        bool enable_debug = 0;
        // -
        unsigned long saveTime_checkCAN = 0;
        unsigned long saveTime_checkReset = 0;
        unsigned long saveTime_run1 = 0;

        // - for reset EMG
        int flag_reseting = 0;
        unsigned long saveTime_checkReseted = 0;
        unsigned long saveTime_sendCAN = 0;
        // -
        uint16_t volRead_time0;
        uint16_t volRead_time1;
        uint16_t volRead_time2;
        uint16_t volRead_time3;
        // - 
        float av_coefficient = 0;
        float bv_coefficient = 0;
        float ac_coefficient = 0;
        float bc_coefficient = 0;
        // -
        uint8_t statusRev_CAN = 0;
        unsigned long saveTime_checkCAN_Rev = 0;
        uint16_t voltageAnalog = 0;
        uint16_t currentAnalog = 0;
        uint16_t voltage_mini = 0; // - 0.1 V
        uint16_t current_mini = 0; // - 0.1 A
        // - 
        unsigned long saveTime_debug = 0;
    public:
        Main_controller(CAN_manager*);
        ~Main_controller();
        
        Status_Main* Main_status = new Status_Main(ID_MAIN);
        Control_Main* Main_comd = new Control_Main(ID_MAIN);

        unsigned long saveTime_checkShutdown = 0;
        unsigned long saveTime_checkLowVoltage = 0;
        unsigned long saveTime_ledControl = 0;

        bool stsSend_CAN = 0;
        bool statusBitSensor[8];

        void CANReceiveHandle();

        bool CANTransmitHandle();

        void setCommandFromController(uint8_t);

        void EMG_detect();

        void soundControl(uint8_t, uint8_t);

        void init_main();

        void loopMainCtr();

        void readButton_Shutdown();

        void resetEMG();
    
        void shutdown_voltageLow();

        void read_voltage();

        void read_currentCharge();

        void control_ledByVoltage();

        void SetEMG(bool tog){ // true = enable
            if (tog) {
                digitalWrite(PIN_EMC_WRITE, LOW);
            }
            else {
                digitalWrite(PIN_EMC_WRITE, HIGH);
            }
        }

        uint16_t filter_analogVoltages(uint16_t vol_in){
            uint16_t vol_out;
            volRead_time3 = volRead_time2;
            volRead_time2 = volRead_time1;
            volRead_time1 = volRead_time0;
            volRead_time0 = vol_in;
            vol_out = volRead_time0 + volRead_time1 + volRead_time2 + volRead_time3;

            return vol_out/4.;
        }

        bool getBit_fromInt(int, int);

        int bytes_to_int(uint8_t, uint8_t);

        uint8_t int_to_byte0(int);

        uint8_t int_to_byte1(int);

        void setCoefficient_voltage(float av, float bv){
            av_coefficient = av;
            bv_coefficient = bv;
        }

        void setCoefficient_current(float ac, float bc){
            ac_coefficient = ac;
            bc_coefficient = bc;
        }

        void Debug_run();
};
#endif