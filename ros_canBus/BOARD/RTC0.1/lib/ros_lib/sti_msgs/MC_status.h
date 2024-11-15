#ifndef _ROS_sti_msgs_MC_status_h
#define _ROS_sti_msgs_MC_status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
// #include "geometry_msgs/Vector3.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"
namespace sti_msgs
{
  class MC_status : public ros::Msg
  {
    public:
      typedef std_msgs::Bool _sts_emg_type ;    _sts_emg_type     sts_emg;
      typedef std_msgs::Bool _sts_alam1_type ;  _sts_alam1_type   sts_alam1;
      typedef std_msgs::Bool _sts_alam2_type ;  _sts_alam2_type   sts_alam2;

    MC_status():
      sts_emg(),
      sts_alam1(),
      sts_alam2()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->sts_emg.serialize(outbuffer + offset);
      offset += this->sts_alam1.serialize(outbuffer + offset);
      offset += this->sts_alam2.serialize(outbuffer + offset);

      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->sts_emg.deserialize(inbuffer + offset);
      offset += this->sts_alam1.deserialize(inbuffer + offset);
      offset += this->sts_alam2.deserialize(inbuffer + offset);

     return offset;
    }

    const char * getType(){ return "sti_msgs/MC_status"; };
    const char * getMD5(){ return "6e416a973f45776fb1ccbae41f2a8090"; };

  };

}
#endif