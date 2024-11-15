#ifndef _ROS_sti_msgs_Lift_status_h
#define _ROS_sti_msgs_Lift_status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
// #include "geometry_msgs/Vector3.h"
#include "std_msgs/Int16.h"

namespace sti_msgs
{
  class Lift_status : public ros::Msg
  {
    public:
      typedef std_msgs::Int16 _status_type; _status_type status;
      typedef std_msgs::Int16 _sensorLift_type; _sensorLift_type sensorLift;
	  typedef std_msgs::Int16 _sensorUp_type; _sensorUp_type sensorUp;
	  typedef std_msgs::Int16 _sensorDown_type; _sensorDown_type sensorDown;

    Lift_status():
      status(),
      sensorLift(),
	  sensorUp(),
	  sensorDown()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->status.serialize(outbuffer + offset);
      offset += this->sensorLift.serialize(outbuffer + offset);
	  offset += this->sensorUp.serialize(outbuffer + offset);
	  offset += this->sensorDown.serialize(outbuffer + offset);

      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->sensorLift.deserialize(inbuffer + offset);
	  offset += this->sensorUp.deserialize(inbuffer + offset);
	  offset += this->sensorDown.deserialize(inbuffer + offset);

     return offset;
    }

    const char * getType(){ return "sti_msgs/Lift_status"; };
    const char * getMD5(){ return "95e256d0dfced53a33461cd2d2411c23"; }; // 95e256d0dfced53a33461cd2d2411c23

  };

}
#endif