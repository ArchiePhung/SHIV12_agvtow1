#ifndef _ROS_sti_msgs_POWER_request_h
#define _ROS_sti_msgs_POWER_request_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"

namespace sti_msgs
{
  class POWER_request : public ros::Msg
  {
    public:
        typedef std_msgs::Bool  _charge_type ;      _charge_type      charge;
        typedef std_msgs::Bool  _sound_on_type ;    _sound_on_type    sound_on;
        typedef std_msgs::Int16 _sound_type_type ;  _sound_type_type  sound_type;
        typedef std_msgs::Bool  _EMC_write_type ;   _EMC_write_type   EMC_write;
        typedef std_msgs::Bool  _EMC_reset_type ;   _EMC_reset_type   EMC_reset;

    POWER_request():
      charge(),
      sound_on(),
      sound_type(),
      EMC_write(),
      EMC_reset()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->charge.serialize(outbuffer + offset);
      offset += this->sound_on.serialize(outbuffer + offset);
      offset += this->sound_type.serialize(outbuffer + offset);
      offset += this->EMC_write.serialize(outbuffer + offset);
      offset += this->EMC_reset.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->charge.deserialize(inbuffer + offset);
      offset += this->sound_on.deserialize(inbuffer + offset);
      offset += this->sound_type.deserialize(inbuffer + offset);
      offset += this->EMC_write.deserialize(inbuffer + offset);
      offset += this->EMC_reset.deserialize(inbuffer + offset);
      return offset;
    }

    const char * getType(){ return "sti_msgs/POWER_request"; };
    const char * getMD5(){ return "a4b5b78eb154a5117b65e7916649effd"; }; 

  };

}
#endif