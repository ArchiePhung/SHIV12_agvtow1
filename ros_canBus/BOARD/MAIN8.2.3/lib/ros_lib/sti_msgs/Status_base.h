#ifndef _ROS_sti_msgs_Status_base_h
#define _ROS_sti_msgs_Status_base_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
// #include "geometry_msgs/Vector3.h"
#include "std_msgs/Int16.h"

namespace sti_msgs
{
  class Status_base : public ros::Msg
  {
    public:
      typedef std_msgs::Int16 _status_type ;   _status_type    status;
      typedef std_msgs::Int16 _connect_error_type ;  _connect_error_type   connect_error;
      typedef std_msgs::Int16 _run_error_type ;      _run_error_type       run_error;

    Status_base():
      status(),
      connect_error(),
      run_error()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->status.serialize(outbuffer + offset);
      offset += this->connect_error.serialize(outbuffer + offset);
      offset += this->run_error.serialize(outbuffer + offset);

      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->connect_error.deserialize(inbuffer + offset);
      offset += this->run_error.deserialize(inbuffer + offset);

     return offset;
    }

    const char * getType(){ return "sti_msgs/Status_base"; };
    const char * getMD5(){ return "7449f44365d7d04ff96ff05648f2ef2d"; }; //7449f44365d7d04ff96ff05648f2ef2d

  };

}
#endif