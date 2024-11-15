#ifndef _ROS_sti_msgs_HC_info_h
#define _ROS_sti_msgs_HC_info_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Int8.h"

namespace sti_msgs
{
  class HC_info : public ros::Msg
  {
    public:
      typedef std_msgs::Int8 _status_type ;     _status_type       status;
      typedef std_msgs::Int8 _zone_sick_ahead_type ;  _zone_sick_ahead_type    zone_sick_ahead;
      typedef std_msgs::Int8 _zone_sick_behind_type ;  _zone_sick_behind_type    zone_sick_behind;
      typedef std_msgs::Int8 _vacham_type ;     _vacham_type       vacham;
     

    HC_info():
      status(),
      zone_sick_ahead(),
      zone_sick_behind(),
      vacham()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->status.serialize(outbuffer + offset);
      offset += this->zone_sick_ahead.serialize(outbuffer + offset);
      offset += this->zone_sick_behind.serialize(outbuffer + offset);
      offset += this->vacham.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->zone_sick_ahead.deserialize(inbuffer + offset);
      offset += this->zone_sick_behind.deserialize(inbuffer + offset);
      offset += this->vacham.deserialize(inbuffer + offset);
      return offset;
    }

    const char * getType(){ return "sti_msgs/HC_info"; };
    const char * getMD5(){ return "40363c78da4d794cb59efd1a501364aa"; }; // 49023105f5d69f2ff5d720955c38bf41

  };

}
#endif