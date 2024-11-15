#ifndef _ROS_sti_msgs_Lift_control_h
#define _ROS_sti_msgs_Lift_control_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Int16.h"

namespace sti_msgs
{
  class Lift_control : public ros::Msg
  {
    public:
		typedef std_msgs::Int16 _control_type; _control_type control;
		typedef std_msgs::Int16 _reset_type; _reset_type reset;

    Lift_control():
      control(),
	  reset()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      	int offset = 0;
      	offset += this->control.serialize(outbuffer + offset);
		offset += this->reset.serialize(outbuffer + offset);
      	return offset;
    }

	virtual int deserialize(unsigned char *inbuffer)
	{
		int offset = 0;
		offset += this->control.deserialize(inbuffer + offset);
		offset += this->reset.deserialize(inbuffer + offset);
		return offset;
	}

    const char * getType(){ return "sti_msgs/Lift_control"; };
    const char * getMD5(){ return "2ff4ce95a4a88dfd4b8fa61600e42f41"; }; //

  };

}
#endif