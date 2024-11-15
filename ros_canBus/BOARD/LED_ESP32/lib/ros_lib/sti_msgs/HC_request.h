#ifndef _ROS_sti_msgs_HC_request_h
#define _ROS_sti_msgs_HC_request_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Int16.h"

namespace sti_msgs
{
  class HC_request : public ros::Msg
  {
    public:
      typedef std_msgs::Int16 _RBG1_type ;  _RBG1_type   RBG1;
      typedef std_msgs::Int16 _RBG2_type ;  _RBG2_type   RBG2;
     

    HC_request():
      RBG1(),
      RBG2()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->RBG1.serialize(outbuffer + offset);
      offset += this->RBG2.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->RBG1.deserialize(inbuffer + offset);
      offset += this->RBG2.deserialize(inbuffer + offset);
      return offset;
    }

    const char * getType(){ return "sti_msgs/HC_request"; };
    const char * getMD5(){ return "f2be05955d3c9687dd91317a1b2a35da"; }; 

  };

}
#endif