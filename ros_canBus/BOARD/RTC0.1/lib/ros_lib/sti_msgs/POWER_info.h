#ifndef _ROS_sti_msgs_POWER_info_h
#define _ROS_sti_msgs_POWER_info_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"

namespace sti_msgs
{
  class POWER_info : public ros::Msg
  {
    public:
        typedef std_msgs::Float32 _voltages_type ;        _voltages_type         voltages;
        typedef std_msgs::Float32 _voltages_analog_type ; _voltages_analog_type  voltages_analog;
        typedef std_msgs::Float32 _charge_current_type ;  _charge_current_type   charge_current;
        typedef std_msgs::Float32 _charge_analog_type ;   _charge_analog_type    charge_analog;
        typedef std_msgs::Bool    _button1_type ;         _button1_type          stsButton_reset;
        typedef std_msgs::Bool    _button2_type ;         _button2_type          stsButton_power;
        typedef std_msgs::Bool    _EMC_status_type ;      _EMC_status_type       EMC_status;
        typedef std_msgs::Int16   _CAN_status_type ;      _CAN_status_type       CAN_status;
     
    POWER_info():
      voltages(),
      voltages_analog(),
      charge_current(),
      charge_analog(),
      stsButton_reset(),
      stsButton_power(),
      EMC_status(),
      CAN_status()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->voltages.serialize(outbuffer + offset);
      offset += this->voltages_analog.serialize(outbuffer + offset);
      offset += this->charge_current.serialize(outbuffer + offset);
      offset += this->charge_analog.serialize(outbuffer + offset);
      offset += this->stsButton_reset.serialize(outbuffer + offset);
      offset += this->stsButton_power.serialize(outbuffer + offset);
      offset += this->EMC_status.serialize(outbuffer + offset);
      offset += this->CAN_status.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->voltages.deserialize(inbuffer + offset);
      offset += this->voltages_analog.deserialize(inbuffer + offset);
      offset += this->charge_current.deserialize(inbuffer + offset);
      offset += this->charge_analog.deserialize(inbuffer + offset);

      offset += this->stsButton_reset.deserialize(inbuffer + offset);
      offset += this->stsButton_power.deserialize(inbuffer + offset);
      offset += this->EMC_status.deserialize(inbuffer + offset);
      offset += this->CAN_status.deserialize(inbuffer + offset);
      return offset;
    }

    const char * getType(){ return "sti_msgs/POWER_info"; };
    const char * getMD5(){ return "81f1ed0d33c4056e6df07717109342c6"; }; 

  };

}
#endif