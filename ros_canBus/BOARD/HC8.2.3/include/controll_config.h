#ifndef CONTROLL_CONFIG
#define CONTROLL_CONFIG

enum class WorkMode{
    manual      = 0x01,
    automatic   = 0x02
};

enum processStatus
{
    IN_PROCESSING   = 0x0C,
    COMMAND_DONE    = 0x0D,
    CANCELED        = 0x0F
};

enum errorStatus
{
    ISOK                        = 0xE0, //224

    ERROR_LINE_OUT              = 0XE1, //225
    ERROR_SENSOR_LINE_TIME_OUT  = 0XE2, //226
    ERROR_SENSOR_LINE_CHECKSUM  = 0XE3, //227
    ERROR_SENSOR_LINE_ID        = 0xE4, //228
    ERROR_MOTOR_LEFT            = 0xE5, //229
    ERROR_MOTOR_RIGHT           = 0XE6, //230

    BREAK_BADERSHOCK            = 0xE7, //231
    ERROR_RFID_READER           = 0xE8, //232

    LOST_CAN_HC                 = 0xE9, //233
    LOST_CAN_OC                 = 0xEA, //234
    LOST_CAN_MC                 = 0xEB, //235
    CAN_DID_NOT_SEND            = 0xEC, //236

    HC_RFID_IS_FAIL             = 0x2E, //46
    HC_RFID_PACKETRECIEVEERR    = 0x3E, //62
    HC_RFID_TIMEOUT             = 0x4E, //78
    HC_RFID_REV_FAIL            = 0x5E, //94
    HC_RFID_BADPACKET           = 0x6E, //110
    HC_RFID_CRCERROR            = 0x7E, //126

    LIFTING_ERROR               = 0xEE, //238

    EMC_ALL                     = 0xEF, //239
};

enum commands
{
    //MAIN commands
    SOUND_1                     = 0x01,
    SOUND_2                     = 0x02,
    SOUND_3                     = 0x03,
    SOUND_EMC                   = 0x04,
    POWER_CHARGING              = 0x05,
    POWER_UNCHARGING            = 0x06,
    ENABLE_SOFT_EMC             = 0x07,
    DISABLE_SOFT_EMC            = 0x08,

    //MC commands
    MODE_STRAIGHT               = 0x21, //33
    TURN_LEFT                   = 0x22, //34
    TURN_RIGHT                  = 0x23, //35
    MODE_TURNING_BACKWARD       = 0x24, //36

    MODE_1_BIT_CATCH            = 0x25, //37
    MODE_HORIZON_LINE           = 0x26, //38
    MODE_RIGHT_DEVIATION        = 0x27, //39
    MODE_LEFT_DEVIATION         = 0x28, //40
    MODE_MOVE_BACKWARD          = 0x29, //41
    MODE_BREAK                  = 0x2A, //42
    MODE_BACKWARD_HORIZON_LINE  = 0x2B, //43
    MODE_BACKWARD_1_BIT_CATCH   = 0x2C, //44
    DO_NOTHING                  = 0x99, //153

    //HC commands
    LED_MODE_G                  = 0x31, //49
    LED_MODE_B                  = 0x32, //50
    LED_MODE_W                  = 0X33, //51
    LED_MODE_EMC                = 0x34, //52
    REQUEST_HEAD_FRONT          = 0x35, //53
    REQUEST_HEAD_BEHIND         = 0x36, //54
    LED_CAM_ON                  = 0x37,
    LED_CAM_OFF                 = 0x38,

    //OC commands
    LIFT_TABLE_UP               = 0x41,
    LIFT_TABLE_DOWN             = 0x42,
    MOTOR_OC_STOP               = 0x43,

    CONVEYER_IN                 = 0x44,
    CONVEYER_OUT                = 0x45,
    
    // common command (for all)
    MODE_RESET_ALARM            = 0x00
};

enum AgvStatus{
    WORKING         = 0x00,
    CHARGING        = 0x01,
    LOW_BATTERY     = 0x03,
    EMC             = 0x0E,
};

enum signals
{
    SickField_nothg = 0b1000, //8
    SickField_safe  = 0b1100, //12
    SickField_warn  = 0b1010, //10
    SickField_stop  = 0b1111  //15
};

#endif