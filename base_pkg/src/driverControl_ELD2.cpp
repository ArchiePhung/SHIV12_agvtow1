// lib ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <message_pkg/Driver_query.h>
#include <message_pkg/Driver_respond.h>
#include <std_msgs/Bool.h>

// lib sys + socket
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <string>
#include <modbuspp.h>
#include <ctime>
// lib 
#include <vector>
#include <math.h>

using namespace std;
using namespace Modbus;

int main(int argc, char** argv)
{
    // param
    string PORT = "/dev/ttyUSB0";
    string BAUDRATE = "57600";

    int ID_driver1 = 1;
    int ID_driver2 = 2;

    int maxRPM = 3800;
    int minRPM = maxRPM*(-1);
    // str config
    string strConfig = BAUDRATE + "E1";
    // khoi tao modbus
    Master MB (Rtu, PORT, "57600E1");
    MB.rtu().setSerialMode(Rs485);
    // add slave
    Slave &drive1 = MB.addSlave(ID_driver1);
    Slave &drive2 = MB.addSlave(ID_driver2);

    return 0;
}


