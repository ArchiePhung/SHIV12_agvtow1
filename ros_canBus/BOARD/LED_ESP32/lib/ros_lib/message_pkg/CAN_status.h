#ifndef _ROS_message_pkg_CAN_status_h
#define _ROS_message_pkg_CAN_status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace message_pkg
{
	class CAN_status : public ros::Msg
	{
		public:
			typedef unsigned char _statusSend_type;
			_statusSend_type statusSend;	
			typedef unsigned char _statusReceived_type;
			_statusReceived_type statusReceived;
			typedef unsigned char _isBusy_type;
			_isBusy_type isBusy;

		CAN_status():
			statusSend(0),
			statusReceived(0),
			isBusy(0)
		{
		}

		virtual int serialize(unsigned char *outbuffer) const
		{
			int offset = 0;

			union {
				unsigned char real;
				uint8_t base;
			} u_statusSend;
			u_statusSend.real = this->statusSend;
			*(outbuffer + offset + 0) = (u_statusSend.base >> (8 * 0)) & 0xFF;
			offset += sizeof(this->statusSend);
			
			union {
				unsigned char real;
				uint8_t base;
			} u_statusReceived;
			u_statusReceived.real = this->statusReceived;
			*(outbuffer + offset + 0) = (u_statusReceived.base >> (8 * 0)) & 0xFF;
			offset += sizeof(this->statusReceived);

			union {
				unsigned char real;
				uint8_t base;
			} u_isBusy;
			u_isBusy.real = this->isBusy;
			*(outbuffer + offset + 0) = (u_isBusy.base >> (8 * 0)) & 0xFF;
			offset += sizeof(this->isBusy);

			return offset;
		}

		virtual int deserialize(unsigned char *inbuffer)
		{
			int offset = 0;

			union {
				unsigned char real;
				uint8_t base;
			} u_statusSend;
			u_statusSend.base = 0;
			u_statusSend.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
			this->statusSend = u_statusSend.real;
			offset += sizeof(this->statusSend);

			union {
				unsigned char real;
				uint8_t base;
			} u_statusReceived;
			u_statusReceived.base = 0;
			u_statusReceived.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
			this->statusReceived = u_statusReceived.real;
			offset += sizeof(this->statusReceived);

			union {
				unsigned char real;
				uint8_t base;
			} u_isBusy;
			u_isBusy.base = 0;
			u_isBusy.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
			this->isBusy = u_isBusy.real;
			offset += sizeof(this->isBusy);

			return offset;
		}

		const char * getType(){ return "message_pkg/CAN_status"; };
		const char * getMD5(){ return "f08d25fc46eeb770198e7bf982cab95a"; };

	};

}
#endif