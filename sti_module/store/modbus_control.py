#!/usr/bin/env python
# author : Anh Tuan - 14/5/2020

import serial
import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu
from sti_msgs.msg import Modbus_0x , Modbus_4x_read , Modbus_4x_write

import rospy
import time
import threading
import signal

PORT = '/dev/stibase_modbus'
BAUDRATE = 115200
ID_SLAVE = 10
data_0x = Modbus_0x()
data_4x_read = Modbus_4x_read()
data_4x_write = Modbus_4x_write()
id_modbus_recived = False

class address_zero_x :
    Err_line_follow = 0
    Low_Battery = 1
    EMC_All = 2
    Power_off = 3
    Motor_Err_1 = 4
    Motor_Err_2 = 5
    Motor_Marker_1 = 6
    Motor_Marker_2 = 7
    H1_Err_sensor_line = 8
    H1_Err_RFID = 9

    H1_Bader_shock = 10
    H1_Sick_ready = 11
    H1_Sick_Field_1 = 12 
    H1_Sick_Field_2 = 13
    H1_Sick_Field_3 = 14
    H2_Err_sensor_line = 15
    H2_Err_RFID = 16
    H2_Bader_shock = 17
    H2_Sick_ready = 18
    H2_Sick_Field_1 = 19

    H2_Sick_Field_2 = 20
    H2_Sick_Field_3 = 21
    BN_SS_H = 22
    BN_SS_L = 23
    BN_SS1 = 24
    BN_SS2  = 25
    Battery_charger = 26
    Err_all = 27
    Err_BN = 28
    Reset_Err_motor = 29

    Power_off_set = 31
    EMC_ON = 32
    H1_Field_select_1 = 33
    H1_Field_select_2 = 34
    H2_Field_seclect_1 = 35
    H2_Field_seclect_2 = 36
    Sick_Line_Follow = 37
    Sick_R_H1 = 38
    Sick_R_H2 = 39

    Battery_charger_on = 40
    khongvach = 41

class address_four_x :
    Status_move = 0
    Status_run = 1
    H1_Position = 2 
    H1_RFID_H = 3
    H1_RFID_L = 4
    H1_RFID_Time = 5
    H2_Position = 6
    H2_RFID_H = 7
    H2_RFID_L = 8
    H2_RFID_Time = 9

    M_speed_1 = 10
    M_speed_2 = 11
    Encoder_Encoder_1 = 12
    Encoder_Encoder_2 = 13
    Battery_value = 14
    BN_S = 15
    BN_R_S = 16
    BT_SS = 17
    BT_SIE = 18
    Sie_in = 19

    Cmd_move = 21
    Initial_speed = 22
    Line_select = 23
    BN_FC  = 24
    Led_FC = 25
    BT_s_1 = 26
    BT_s_2 = 27
    BT_s_3 = 28
    BT_DC_1 = 29

    BT_DC_2 = 30
    BT_SIE_OUT =31
    COI_FC = 32

class m_data_zero_x :
    Err_line_follow  = bool()

class m_data_four_x :
    Status_move = int()


class Connect_modbus(threading.Thread):
    # global data_0x ,data_4x_read ,data_4x_write , id_modbus_recived
    def __init__(self, threadID):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.shutdown_flag = threading.Event()
        self.rate = rospy.Rate(100)
        
    def run(self):
        global data_0x ,data_4x_read ,data_4x_write , id_modbus_recived
        # modbus
        # logger = modbus_tk.utils.create_logger("console")
        try:
            #Connect to the slave
            MODBUS = modbus_rtu.RtuMaster(
                serial.Serial(port=PORT, baudrate= BAUDRATE, bytesize=8, parity='E', stopbits=1, xonxoff=0)
            )
            MODBUS.open 
            MODBUS.set_timeout(5.0)
            MODBUS.set_verbose(True)
            rospy.loginfo("Modbus connected !")

        except modbus_tk.modbus.ModbusError as exc:
            # logger.error("%s- Code=%d", exc, exc.get_exception_code())
            rospy.loginfo("Modbus false !")

        while not self.shutdown_flag.is_set():
          # 0x 
            data_0x.Err_line_follow     = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.Err_line_follow  , 1 )[0]
            data_0x.Low_Battery         = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.Low_Battery , 1 )[0]
            data_0x.EMC_All             = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.EMC_All , 1 )[0]
            data_0x.Power_off           = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.Power_off , 1 )[0]
            data_0x.Motor_Err_1         = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.Motor_Err_1 , 1 )[0]
            data_0x.Motor_Err_2         = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.Motor_Err_2 , 1 )[0]
            data_0x.Motor_Marker_1      = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.Motor_Marker_1 , 1 )[0]
            data_0x.Motor_Marker_2      = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.Motor_Marker_2 , 1 )[0]
            data_0x.H1_Err_sensor_line  = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.H1_Err_sensor_line , 1 )[0]
            data_0x.H1_Err_RFID         = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.H1_Err_RFID , 1 )[0]
            data_0x.H1_Bader_shock      = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.H1_Bader_shock , 1 )[0]
            data_0x.H1_Sick_ready       = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.H1_Sick_ready , 1 )[0]
            data_0x.H1_Sick_Field_1     = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.H1_Sick_Field_1 , 1 )[0]
            data_0x.H1_Sick_Field_2     = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.H1_Sick_Field_2 , 1 )[0]
            data_0x.H1_Sick_Field_3     = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.H1_Sick_Field_3 , 1 )[0]
            data_0x.H2_Err_sensor_line  = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.H2_Err_sensor_line , 1 )[0]
            data_0x.H2_Err_RFID         = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.H2_Err_RFID , 1 )[0]
            data_0x.H2_Bader_shock      = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.H2_Bader_shock , 1 )[0]
            data_0x.H2_Sick_ready       = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.H2_Sick_ready , 1 )[0]
            data_0x.H2_Sick_Field_1     = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.H2_Sick_Field_1 , 1 )[0]
            data_0x.H2_Sick_Field_2     = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.H2_Sick_Field_2 , 1 )[0]
            data_0x.H2_Sick_Field_3     = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.H2_Sick_Field_3 , 1 )[0]
            data_0x.BN_SS_H             = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.BN_SS_H , 1 )[0]
            data_0x.BN_SS_L             = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.BN_SS_L , 1 )[0]
            data_0x.BN_SS1              = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.BN_SS1 , 1 )[0]
            data_0x.BN_SS2              = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.BN_SS2  , 1 )[0]
            data_0x.Battery_charger     = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.Battery_charger , 1 )[0]
            data_0x.Err_all             = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.Err_all , 1 )[0]
            data_0x.Err_BN              = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.Err_BN , 1 )[0]
            data_0x.Reset_Err_motor     = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.Reset_Err_motor , 1 )[0]
            data_0x.Power_off_set       = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.Power_off_set , 1 )[0]
            data_0x.EMC_ON              = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.EMC_ON , 1 )[0]
            data_0x.H1_Field_select_1   = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.H1_Field_select_1 , 1 )[0]
            data_0x.H1_Field_select_2   = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.H1_Field_select_2 , 1 )[0]
            data_0x.H2_Field_seclect_1  = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.H2_Field_seclect_1 , 1 )[0]
            data_0x.H2_Field_seclect_2  = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.H2_Field_seclect_2 , 1 )[0]
            data_0x.Sick_Line_Follow    = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.Sick_Line_Follow , 1 )[0]
            data_0x.Sick_R_H1           = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.Sick_R_H1 , 1 )[0]
            data_0x.Sick_R_H2           = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.Sick_R_H2 , 1 )[0]
            data_0x.Battery_charger_on  = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.Battery_charger_on , 1 )[0]
            data_0x.khongvach           = MODBUS.execute(ID_SLAVE, cst.READ_COILS, address_zero_x.khongvach , 1 )[0]

          # 4x read
            data_4x_read.Status_move         = MODBUS.execute(ID_SLAVE, cst.READ_HOLDING_REGISTERS, address_four_x.Status_move  , 1 )[0]
            data_4x_read.Status_run          = MODBUS.execute(ID_SLAVE, cst.READ_HOLDING_REGISTERS, address_four_x.Status_run   , 1 )[0]
            data_4x_read.H1_Position         = MODBUS.execute(ID_SLAVE, cst.READ_HOLDING_REGISTERS, address_four_x.H1_Position  , 1 )[0]
            data_4x_read.H1_RFID_H           = MODBUS.execute(ID_SLAVE, cst.READ_HOLDING_REGISTERS, address_four_x.H1_RFID_H  , 1 )[0]
            data_4x_read.H1_RFID_L           = MODBUS.execute(ID_SLAVE, cst.READ_HOLDING_REGISTERS, address_four_x.H1_RFID_L  , 1 )[0]
            data_4x_read.H1_RFID_Time        = MODBUS.execute(ID_SLAVE, cst.READ_HOLDING_REGISTERS, address_four_x.H1_RFID_Time  , 1 )[0]
            data_4x_read.H2_Position         = MODBUS.execute(ID_SLAVE, cst.READ_HOLDING_REGISTERS, address_four_x.H2_Position  , 1 )[0]
            data_4x_read.H2_RFID_H           = MODBUS.execute(ID_SLAVE, cst.READ_HOLDING_REGISTERS, address_four_x.H2_RFID_H  , 1 )[0]
            data_4x_read.H2_RFID_L           = MODBUS.execute(ID_SLAVE, cst.READ_HOLDING_REGISTERS, address_four_x.H2_RFID_L  , 1 )[0]
            data_4x_read.H2_RFID_Time        = MODBUS.execute(ID_SLAVE, cst.READ_HOLDING_REGISTERS, address_four_x.H2_RFID_Time  , 1 )[0]
            data_4x_read.M_speed_1           = MODBUS.execute(ID_SLAVE, cst.READ_HOLDING_REGISTERS, address_four_x.M_speed_1  , 1 )[0]
            data_4x_read.M_speed_2           = MODBUS.execute(ID_SLAVE, cst.READ_HOLDING_REGISTERS, address_four_x.M_speed_2  , 1 )[0]
            data_4x_read.Encoder_Encoder_1   = MODBUS.execute(ID_SLAVE, cst.READ_HOLDING_REGISTERS, address_four_x.Encoder_Encoder_1  , 1 )[0]
            data_4x_read.Encoder_Encoder_2   = MODBUS.execute(ID_SLAVE, cst.READ_HOLDING_REGISTERS, address_four_x.Encoder_Encoder_2  , 1 )[0]
            data_4x_read.Battery_value       = MODBUS.execute(ID_SLAVE, cst.READ_HOLDING_REGISTERS, address_four_x.Battery_value  , 1 )[0]
            data_4x_read.BN_S                = MODBUS.execute(ID_SLAVE, cst.READ_HOLDING_REGISTERS, address_four_x.BN_S  , 1 )[0]
            data_4x_read.BN_R_S              = MODBUS.execute(ID_SLAVE, cst.READ_HOLDING_REGISTERS, address_four_x.BN_R_S   , 1 )[0]
            data_4x_read.BT_SS               = MODBUS.execute(ID_SLAVE, cst.READ_HOLDING_REGISTERS, address_four_x.BT_SS  , 1 )[0]
            data_4x_read.BT_SIE              = MODBUS.execute(ID_SLAVE, cst.READ_HOLDING_REGISTERS, address_four_x.BT_SIE  , 1 )[0]
            data_4x_read.Sie_in              = MODBUS.execute(ID_SLAVE, cst.READ_HOLDING_REGISTERS, address_four_x.Sie_in  , 1 )[0]
          
          # s4x write
            if id_modbus_recived == True :
                id_modbus_recived = False
                MODBUS.execute(ID_SLAVE, cst.WRITE_SINGLE_REGISTER, address_four_x.Cmd_move      , output_value = data_4x_write.Cmd_move )
                MODBUS.execute(ID_SLAVE, cst.WRITE_SINGLE_REGISTER, address_four_x.Initial_speed , output_value = data_4x_write.Initial_speed )
                MODBUS.execute(ID_SLAVE, cst.WRITE_SINGLE_REGISTER, address_four_x.Line_select   , output_value = data_4x_write.Line_select )
                MODBUS.execute(ID_SLAVE, cst.WRITE_SINGLE_REGISTER, address_four_x.BN_FC         , output_value = data_4x_write.BN_FC  )
                MODBUS.execute(ID_SLAVE, cst.WRITE_SINGLE_REGISTER, address_four_x.Led_FC        , output_value = data_4x_write.Led_FC )
                MODBUS.execute(ID_SLAVE, cst.WRITE_SINGLE_REGISTER, address_four_x.BT_s_1        , output_value = data_4x_write.BT_s_1 )
                MODBUS.execute(ID_SLAVE, cst.WRITE_SINGLE_REGISTER, address_four_x.BT_s_2        , output_value = data_4x_write.BT_s_2 )
                MODBUS.execute(ID_SLAVE, cst.WRITE_SINGLE_REGISTER, address_four_x.BT_s_3        , output_value = data_4x_write.BT_s_3 )
                MODBUS.execute(ID_SLAVE, cst.WRITE_SINGLE_REGISTER, address_four_x.BT_DC_1       , output_value = data_4x_write.BT_DC_1 )
                MODBUS.execute(ID_SLAVE, cst.WRITE_SINGLE_REGISTER, address_four_x.BT_DC_2       , output_value = data_4x_write.BT_DC_2 )
                MODBUS.execute(ID_SLAVE, cst.WRITE_SINGLE_REGISTER, address_four_x.BT_SIE_OUT    , output_value = data_4x_write.BT_SIE_OUT )
                MODBUS.execute(ID_SLAVE, cst.WRITE_SINGLE_REGISTER, address_four_x.COI_FC        , output_value = data_4x_write.COI_FC )
            
            # print "battery= ", data_4x_read.Battery_value

            self.rate.sleep()
       
        print('Thread #%s stopped' % self.threadID)

class Connect_ros(threading.Thread):
    global data_0x ,data_4x_read ,data_4x_write 
    def __init__(self, threadID):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.shutdown_flag = threading.Event()
        self.rate = rospy.Rate(100)

        self.modbus_pub_0x = rospy.Publisher('modbus_0x',Modbus_0x , queue_size=50)
        self.modbus_pub_4x_read = rospy.Publisher('modbus_4x_read',Modbus_4x_read , queue_size=50)
        rospy.Subscriber("modbus_4x_write", Modbus_4x_write, self.call_modbus)

    def call_modbus(self,data):
        global data_4x_write , id_modbus_recived
        data_4x_write = data
        if id_modbus_recived == False :
            id_modbus_recived = True

    def run(self):
        while not self.shutdown_flag.is_set():
            self.modbus_pub_0x.publish(data_0x)
            self.modbus_pub_4x_read.publish(data_4x_read)

            self.rate.sleep()
        print('Thread #%s stopped' % self.threadID)

class ServiceExit(Exception):
    """
    Custom exception which is used to trigger the clean exit
    of all running threads and the main program.
    """
    pass
 
def service_shutdown(signum, frame):
    print('Caught signal %d' % signum)
    raise ServiceExit

def main():
    rospy.init_node('comunication_modbus')
    # Register the signal handlers
    signal.signal(signal.SIGTERM, service_shutdown)
    signal.signal(signal.SIGINT, service_shutdown)
 
    print('Starting main program')
 
    # Start the job threads
    try:
        thread1 = Connect_modbus(1)
        thread1.start()
        thread2 = Connect_ros(2)
        thread2.start()
 
        # Keep the main thread running, otherwise signals are ignored.
        while True:
            time.sleep(0.01)
 
    except ServiceExit:
        thread1.shutdown_flag.set()
        thread1.join()
        thread2.shutdown_flag.set()
        thread2.join()
    print('Exiting main program')
 
if __name__ == '__main__':
    main()