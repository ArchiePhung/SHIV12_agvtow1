#ifndef _ROS_sti_msgs_Motor_parameter_h
#define _ROS_sti_msgs_Motor_parameter_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sti_msgs
{

	class Motor_parameter : public ros::Msg
	{
		public:
			typedef float _speed_type;
			_speed_type speed;	
			typedef float _errMea_type;
			_errMea_type errMea;
			typedef float _errEst_type;
			_errEst_type errEst;
			typedef float _q_type;
			_q_type q;

			typedef float _kp_type;
			_kp_type kp;
			typedef float _ki_type;
			_ki_type ki;
			typedef float _kd_type;
			_kd_type kd;

		Motor_parameter():
			speed(0),
			errMea(0),
			errEst(0),
			q(0),
			kp(0),
			ki(0),
			kd(0)
		{
		}

		virtual int serialize(unsigned char *outbuffer) const
		{
			int offset = 0;

			union {
				float real;
				uint32_t base;
			} u_speed;
			u_speed.real = this->speed;
			*(outbuffer + offset + 0) = (u_speed.base >> (8 * 0)) & 0xFF;
			*(outbuffer + offset + 1) = (u_speed.base >> (8 * 1)) & 0xFF;
			*(outbuffer + offset + 2) = (u_speed.base >> (8 * 2)) & 0xFF;
			*(outbuffer + offset + 3) = (u_speed.base >> (8 * 3)) & 0xFF;
			offset += sizeof(this->speed);

			union {
				float real;
				uint32_t base;
			} u_errMea;
			u_errMea.real = this->errMea;
			*(outbuffer + offset + 0) = (u_errMea.base >> (8 * 0)) & 0xFF;
			*(outbuffer + offset + 1) = (u_errMea.base >> (8 * 1)) & 0xFF;
			*(outbuffer + offset + 2) = (u_errMea.base >> (8 * 2)) & 0xFF;
			*(outbuffer + offset + 3) = (u_errMea.base >> (8 * 3)) & 0xFF;
			offset += sizeof(this->errMea);
			
			union {
				float real;
				uint32_t base;
			} u_errEst;
			u_errEst.real = this->errEst;
			*(outbuffer + offset + 0) = (u_errEst.base >> (8 * 0)) & 0xFF;
			*(outbuffer + offset + 1) = (u_errEst.base >> (8 * 1)) & 0xFF;
			*(outbuffer + offset + 2) = (u_errEst.base >> (8 * 2)) & 0xFF;
			*(outbuffer + offset + 3) = (u_errEst.base >> (8 * 3)) & 0xFF;
			offset += sizeof(this->errEst);

			union {
				float real;
				uint32_t base;
			} u_q;
			u_q.real = this->q;
			*(outbuffer + offset + 0) = (u_q.base >> (8 * 0)) & 0xFF;
			*(outbuffer + offset + 1) = (u_q.base >> (8 * 1)) & 0xFF;
			*(outbuffer + offset + 2) = (u_q.base >> (8 * 2)) & 0xFF;
			*(outbuffer + offset + 3) = (u_q.base >> (8 * 3)) & 0xFF;
			offset += sizeof(this->q);

			union {
				float real;
				uint32_t base;
			} u_kp;
			u_kp.real = this->kp;
			*(outbuffer + offset + 0) = (u_kp.base >> (8 * 0)) & 0xFF;
			*(outbuffer + offset + 1) = (u_kp.base >> (8 * 1)) & 0xFF;
			*(outbuffer + offset + 2) = (u_kp.base >> (8 * 2)) & 0xFF;
			*(outbuffer + offset + 3) = (u_kp.base >> (8 * 3)) & 0xFF;
			offset += sizeof(this->kp);
			
			union {
				float real;
				uint32_t base;
			} u_ki;
			u_ki.real = this->ki;
			*(outbuffer + offset + 0) = (u_ki.base >> (8 * 0)) & 0xFF;
			*(outbuffer + offset + 1) = (u_ki.base >> (8 * 1)) & 0xFF;
			*(outbuffer + offset + 2) = (u_ki.base >> (8 * 2)) & 0xFF;
			*(outbuffer + offset + 3) = (u_ki.base >> (8 * 3)) & 0xFF;
			offset += sizeof(this->ki);

			union {
				float real;
				uint32_t base;
			} u_kd;
			u_kd.real = this->kd;
			*(outbuffer + offset + 0) = (u_kd.base >> (8 * 0)) & 0xFF;
			*(outbuffer + offset + 1) = (u_kd.base >> (8 * 1)) & 0xFF;
			*(outbuffer + offset + 2) = (u_kd.base >> (8 * 2)) & 0xFF;
			*(outbuffer + offset + 3) = (u_kd.base >> (8 * 3)) & 0xFF;
			offset += sizeof(this->kd);

			return offset;
		}

		virtual int deserialize(unsigned char *inbuffer)
		{
			int offset = 0;

			union {
				float real;
				uint32_t base;
			} u_speed;
			u_speed.base = 0;
			u_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
			u_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
			u_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
			u_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
			this->speed = u_speed.real;
			offset += sizeof(this->speed);

			union {
				float real;
				uint32_t base;
			} u_errMea;
			u_errMea.base = 0;
			u_errMea.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
			u_errMea.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
			u_errMea.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
			u_errMea.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
			this->errMea = u_errMea.real;
			offset += sizeof(this->errMea);

			union {
				float real;
				uint32_t base;
			} u_errEst;
			u_errEst.base = 0;
			u_errEst.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
			u_errEst.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
			u_errEst.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
			u_errEst.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
			this->errEst = u_errEst.real;
			offset += sizeof(this->errEst);

			union {
				float real;
				uint32_t base;
			} u_q;
			u_q.base = 0;
			u_q.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
			u_q.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
			u_q.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
			u_q.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
			this->q = u_q.real;
			offset += sizeof(this->q);

			union {
				float real;
				uint32_t base;
			} u_kp;
			u_kp.base = 0;
			u_kp.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
			u_kp.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
			u_kp.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
			u_kp.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
			this->kp = u_kp.real;
			offset += sizeof(this->kp);

			union {
				float real;
				uint32_t base;
			} u_ki;
			u_ki.base = 0;
			u_ki.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
			u_ki.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
			u_ki.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
			u_ki.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
			this->ki = u_ki.real;
			offset += sizeof(this->ki);

			union {
				float real;
				uint32_t base;
			} u_kd;
			u_kd.base = 0;
			u_kd.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
			u_kd.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
			u_kd.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
			u_kd.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
			this->kd = u_kd.real;
			offset += sizeof(this->kd);

			return offset;
		}

		const char * getType(){ return "sti_msgs/Motor_parameter"; };
		const char * getMD5(){ return "a1c21638afd332b6030e82ecfad4e484"; };

		};

}
#endif