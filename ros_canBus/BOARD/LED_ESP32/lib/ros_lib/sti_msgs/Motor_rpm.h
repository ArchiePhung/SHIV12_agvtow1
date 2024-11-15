#ifndef _ROS_sti_msgs_Motor_rpm_h
#define _ROS_sti_msgs_Motor_rpm_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sti_msgs
{

  class Motor_rpm : public ros::Msg
  {
		public:
			typedef float _motor1_requir_type;
				_motor1_requir_type motor1_requir;	
			typedef float _motor1_respond_type;
				_motor1_respond_type motor1_respond;
			typedef float _motor2_requir_type;
				_motor2_requir_type motor2_requir;	
			typedef float _motor2_respond_type;
				_motor2_respond_type motor2_respond;

			Motor_rpm():
				motor1_requir(0),
				motor1_respond(0),
				motor2_requir(0),
				motor2_respond(0)
			{
			}

    virtual int serialize(unsigned char *outbuffer) const
    {
      	int offset = 0;

     	union {
			float real;
			uint32_t base;
		} u_motor1_requir;
		u_motor1_requir.real = this->motor1_requir;
		*(outbuffer + offset + 0) = (u_motor1_requir.base >> (8 * 0)) & 0xFF;
		*(outbuffer + offset + 1) = (u_motor1_requir.base >> (8 * 1)) & 0xFF;
		*(outbuffer + offset + 2) = (u_motor1_requir.base >> (8 * 2)) & 0xFF;
		*(outbuffer + offset + 3) = (u_motor1_requir.base >> (8 * 3)) & 0xFF;
		offset += sizeof(this->motor1_requir);

    	union {
			float real;
			uint32_t base;
		} u_motor1_respond;
		u_motor1_respond.real = this->motor1_respond;
		*(outbuffer + offset + 0) = (u_motor1_respond.base >> (8 * 0)) & 0xFF;
		*(outbuffer + offset + 1) = (u_motor1_respond.base >> (8 * 1)) & 0xFF;
		*(outbuffer + offset + 2) = (u_motor1_respond.base >> (8 * 2)) & 0xFF;
		*(outbuffer + offset + 3) = (u_motor1_respond.base >> (8 * 3)) & 0xFF;
		offset += sizeof(this->motor1_respond);
	  
		union {
			float real;
			uint32_t base;
		} u_motor2_requir;
		u_motor2_requir.real = this->motor2_requir;
		*(outbuffer + offset + 0) = (u_motor2_requir.base >> (8 * 0)) & 0xFF;
		*(outbuffer + offset + 1) = (u_motor2_requir.base >> (8 * 1)) & 0xFF;
		*(outbuffer + offset + 2) = (u_motor2_requir.base >> (8 * 2)) & 0xFF;
		*(outbuffer + offset + 3) = (u_motor2_requir.base >> (8 * 3)) & 0xFF;
		offset += sizeof(this->motor2_requir);

		union {
			float real;
			uint32_t base;
		} u_motor2_respond;
		u_motor2_respond.real = this->motor2_respond;
		*(outbuffer + offset + 0) = (u_motor2_respond.base >> (8 * 0)) & 0xFF;
		*(outbuffer + offset + 1) = (u_motor2_respond.base >> (8 * 1)) & 0xFF;
		*(outbuffer + offset + 2) = (u_motor2_respond.base >> (8 * 2)) & 0xFF;
		*(outbuffer + offset + 3) = (u_motor2_respond.base >> (8 * 3)) & 0xFF;
		offset += sizeof(this->motor2_respond);  

		return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
		int offset = 0;

		union {
			float real;
			uint32_t base;
		} u_motor1_requir;
		u_motor1_requir.base = 0;
		u_motor1_requir.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
		u_motor1_requir.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
		u_motor1_requir.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
		u_motor1_requir.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
		this->motor1_requir = u_motor1_requir.real;
		offset += sizeof(this->motor1_requir);

		union {
			float real;
			uint32_t base;
		} u_motor1_respond;
		u_motor1_respond.base = 0;
		u_motor1_respond.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
		u_motor1_respond.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
		u_motor1_respond.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
		u_motor1_respond.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
		this->motor1_respond = u_motor1_respond.real;
		offset += sizeof(this->motor1_respond);

		union {
			float real;
			uint32_t base;
		} u_motor2_requir;
		u_motor2_requir.base = 0;
		u_motor2_requir.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
		u_motor2_requir.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
		u_motor2_requir.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
		u_motor2_requir.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
		this->motor2_requir = u_motor2_requir.real;
		offset += sizeof(this->motor2_requir);

		union {
			float real;
			uint32_t base;
		} u_motor2_respond;
		u_motor2_respond.base = 0;
		u_motor2_respond.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
		u_motor2_respond.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
		u_motor2_respond.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
		u_motor2_respond.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
		this->motor2_respond = u_motor2_respond.real;
		offset += sizeof(this->motor2_respond);     

		return offset;
    }

    const char * getType(){ return "sti_msgs/Motor_rpm"; };
    const char * getMD5(){ return "32af8bb0821104478f5af6bdc31e5373"; };

  };

}
#endif