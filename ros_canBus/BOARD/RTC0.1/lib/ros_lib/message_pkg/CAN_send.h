#ifndef _ROS_message_pkg_CAN_send_h
#define _ROS_message_pkg_CAN_send_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace message_pkg
{
	class CAN_send : public ros::Msg
	{
		public:
			typedef uint32_t _id_type;
			_id_type id;
			typedef uint8_t _byte0_type;
			_byte0_type byte0;	
			typedef uint8_t _byte1_type;
			_byte1_type byte1;
			typedef uint8_t _byte2_type;
			_byte2_type byte2;
			typedef uint8_t _byte3_type;
			_byte3_type byte3;
			typedef uint8_t _byte4_type;
			_byte4_type byte4;
			typedef uint8_t _byte5_type;
			_byte5_type byte5;
			typedef uint8_t _byte6_type;
			_byte6_type byte6;
			typedef uint8_t _byte7_type;
			_byte7_type byte7;

		CAN_send():
			id(0),
			byte0(0),
			byte1(0),
			byte2(0),
			byte3(0),
			byte4(0),
			byte5(0),
			byte6(0),
			byte7(0)
		{
		}

		virtual int serialize(unsigned char *outbuffer) const
		{
			int offset = 0;

			// -- ID
			union {
				uint32_t real;
				uint32_t base;
			} u_id;
			u_id.real = this->id;
			*(outbuffer + offset + 0) = (u_id.base >> (8 * 0)) & 0xFF;
			*(outbuffer + offset + 1) = (u_id.base >> (8 * 1)) & 0xFF;
			*(outbuffer + offset + 2) = (u_id.base >> (8 * 2)) & 0xFF;
			*(outbuffer + offset + 3) = (u_id.base >> (8 * 3)) & 0xFF;
			offset += sizeof(this->id);
			//
			union {
				uint8_t real;
				uint8_t base;
			} u_byte0;
			u_byte0.real = this->byte0;
			*(outbuffer + offset + 0) = (u_byte0.base >> (8 * 0)) & 0xFF;
			offset += sizeof(this->byte0);
			
			union {
				uint8_t real;
				uint8_t base;
			} u_byte1;
			u_byte1.real = this->byte1;
			*(outbuffer + offset + 0) = (u_byte1.base >> (8 * 0)) & 0xFF;
			offset += sizeof(this->byte1);

			union {
				uint8_t real;
				uint8_t base;
			} u_byte2;
			u_byte2.real = this->byte2;
			*(outbuffer + offset + 0) = (u_byte2.base >> (8 * 0)) & 0xFF;
			offset += sizeof(this->byte2);

			union {
				uint8_t real;
				uint8_t base;
			} u_byte3;
			u_byte3.real = this->byte3;
			*(outbuffer + offset + 0) = (u_byte3.base >> (8 * 0)) & 0xFF;
			offset += sizeof(this->byte3);
			
			union {
				uint8_t real;
				uint8_t base;
			} u_byte4;
			u_byte4.real = this->byte4;
			*(outbuffer + offset + 0) = (u_byte4.base >> (8 * 0)) & 0xFF;
			offset += sizeof(this->byte4);

			union {
				uint8_t real;
				uint8_t base;
			} u_byte5;
			u_byte5.real = this->byte5;
			*(outbuffer + offset + 0) = (u_byte5.base >> (8 * 0)) & 0xFF;
			offset += sizeof(this->byte5);

			union {
				uint8_t real;
				uint8_t base;
			} u_byte6;
			u_byte6.real = this->byte6;
			*(outbuffer + offset + 0) = (u_byte6.base >> (8 * 0)) & 0xFF;
			offset += sizeof(this->byte6);

			union {
				uint8_t real;
				uint8_t base;
			} u_byte7;
			u_byte7.real = this->byte7;
			*(outbuffer + offset + 0) = (u_byte7.base >> (8 * 0)) & 0xFF;
			offset += sizeof(this->byte7);

			return offset;
		}

		virtual int deserialize(unsigned char *inbuffer)
		{
			int offset = 0;

			union {
				uint32_t real;
				uint32_t base;
			} u_id;
			u_id.base = 0;
			u_id.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
			u_id.base |= ((uint8_t) (*(inbuffer + offset + 1))) << (8 * 1);
			u_id.base |= ((uint8_t) (*(inbuffer + offset + 2))) << (8 * 2);
			u_id.base |= ((uint8_t) (*(inbuffer + offset + 3))) << (8 * 3);
			this->id = u_id.real;
			offset += sizeof(this->id);

			union {
				uint8_t real;
				uint8_t base;
			} u_byte0;
			u_byte0.base = 0;
			u_byte0.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
			this->byte0 = u_byte0.real;
			offset += sizeof(this->byte0);

			union {
				uint8_t real;
				uint8_t base;
			} u_byte1;
			u_byte1.base = 0;
			u_byte1.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
			this->byte1 = u_byte1.real;
			offset += sizeof(this->byte1);

			union {
				uint8_t real;
				uint8_t base;
			} u_byte2;
			u_byte2.base = 0;
			u_byte2.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
			this->byte2 = u_byte2.real;
			offset += sizeof(this->byte2);

			union {
				uint8_t real;
				uint8_t base;
			} u_byte3;
			u_byte3.base = 0;
			u_byte3.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
			this->byte3 = u_byte3.real;
			offset += sizeof(this->byte3);

			union {
				uint8_t real;
				uint8_t base;
			} u_byte4;
			u_byte4.base = 0;
			u_byte4.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
			this->byte4 = u_byte4.real;
			offset += sizeof(this->byte4);

			union {
				uint8_t real;
				uint8_t base;
			} u_byte5;
			u_byte5.base = 0;
			u_byte5.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
			this->byte5 = u_byte5.real;
			offset += sizeof(this->byte5);

			union {
				uint8_t real;
				uint8_t base;
			} u_byte6;
			u_byte6.base = 0;
			u_byte6.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
			this->byte6 = u_byte6.real;
			offset += sizeof(this->byte6);

			union {
				uint8_t real;
				uint8_t base;
			} u_byte7;
			u_byte7.base = 0;
			u_byte7.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
			this->byte7 = u_byte7.real;
			offset += sizeof(this->byte7);

			return offset;
		}

		const char * getType(){ return "message_pkg/CAN_send"; };
		const char * getMD5(){ return "331e20b7386371452e71b0aa43203faa"; };

	};

}
#endif