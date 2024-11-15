#ifndef _ROS_sti_msgs_Ps2_msgs_h
#define _ROS_sti_msgs_Ps2_msgs_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
// #include "geometry_msgs/Vector3.h"
#include "std_msgs/Bool.h"

namespace sti_msgs
{

  class Ps2_msgs : public ros::Msg
  {
    public:
      typedef std_msgs::Bool _start_type ;  _start_type   start ;
      typedef std_msgs::Bool _select_type ;  _select_type   select ;
      typedef std_msgs::Bool _up_type ;      _up_type       up ;
      typedef std_msgs::Bool _right_type ;   _right_type    right;
      typedef std_msgs::Bool _left_type ;    _left_type     left ;
      typedef std_msgs::Bool _down_type ;    _down_type     down ;
      typedef std_msgs::Bool _triangle_type ;_triangle_type triangle;
      typedef std_msgs::Bool _circle_type ;  _circle_type   circle ;
      typedef std_msgs::Bool _cross_type ;   _cross_type    cross ;
      typedef std_msgs::Bool _square_type ;  _square_type   square ;
      typedef std_msgs::Bool _l1_type ;      _l1_type       l1 ;
      typedef std_msgs::Bool _l2_type ;      _l2_type       l2 ;
      typedef std_msgs::Bool _l3_type ;      _l3_type       l3 ;
      typedef std_msgs::Bool _r1_type ;      _r1_type       r1 ;
      typedef std_msgs::Bool _r2_type ;      _r2_type       r2 ;
      typedef std_msgs::Bool _r3_type ;      _r3_type       r3 ;

    Ps2_msgs():
      start(),
      select(),  
      up(), 
      right(),
      left(), 
      down(), 
      triangle(),
      circle(), 
      cross(), 
      square(), 
      l1(), 
      l2(),
      l3(), 
      r1(), 
      r2(), 
      r3()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->start.serialize(outbuffer + offset);
      offset += this->select.serialize(outbuffer + offset);
      offset += this->up.serialize(outbuffer + offset);
      offset += this->right.serialize(outbuffer + offset);
      offset += this->left.serialize(outbuffer + offset);
      offset += this->down.serialize(outbuffer + offset);
      offset += this->triangle.serialize(outbuffer + offset);
      offset += this->circle.serialize(outbuffer + offset);
      offset += this->cross.serialize(outbuffer + offset);
      offset += this->square.serialize(outbuffer + offset);
      offset += this->l1.serialize(outbuffer + offset);
      offset += this->l2.serialize(outbuffer + offset);
      offset += this->l3.serialize(outbuffer + offset);
      offset += this->r1.serialize(outbuffer + offset);
      offset += this->r2.serialize(outbuffer + offset);
      offset += this->r3.serialize(outbuffer + offset);

      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->start.deserialize(inbuffer + offset);
      offset += this->select.deserialize(inbuffer + offset);
      offset += this->up.deserialize(inbuffer + offset);
      offset += this->right.deserialize(inbuffer + offset);
      offset += this->left.deserialize(inbuffer + offset);
      offset += this->down.deserialize(inbuffer + offset);
      offset += this->triangle.deserialize(inbuffer + offset);
      offset += this->circle.deserialize(inbuffer + offset);
      offset += this->cross.deserialize(inbuffer + offset);
      offset += this->square.deserialize(inbuffer + offset);
      offset += this->l1.deserialize(inbuffer + offset);
      offset += this->l2.deserialize(inbuffer + offset);
      offset += this->l3.deserialize(inbuffer + offset);
      offset += this->r1.deserialize(inbuffer + offset);
      offset += this->r2.deserialize(inbuffer + offset);
      offset += this->r3.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "sti_msgs/Ps2_msgs"; };
    const char * getMD5(){ return "5ed7322334186fba585d3aa075cb4b59"; };
  };

}
#endif