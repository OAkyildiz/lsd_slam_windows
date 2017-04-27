#ifndef _ROS_lsd_slam_viewer_KeyframeMsg_h
#define _ROS_lsd_slam_viewer_KeyframeMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace lsd_slam_viewer
{

  class KeyframeMsg : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef bool _isKeyframe_type;
      _isKeyframe_type isKeyframe;
      float camToWorld[7];
      typedef float _fx_type;
      _fx_type fx;
      typedef float _fy_type;
      _fy_type fy;
      typedef float _cx_type;
      _cx_type cx;
      typedef float _cy_type;
      _cy_type cy;
      typedef uint32_t _height_type;
      _height_type height;
      typedef uint32_t _width_type;
      _width_type width;
      uint32_t pointcloud_length;
      typedef uint8_t _pointcloud_type;
      _pointcloud_type st_pointcloud;
      _pointcloud_type * pointcloud;

    KeyframeMsg():
      header(),
      isKeyframe(0),
      camToWorld(),
      fx(0),
      fy(0),
      cx(0),
      cy(0),
      height(0),
      width(0),
      pointcloud_length(0), pointcloud(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_isKeyframe;
      u_isKeyframe.real = this->isKeyframe;
      *(outbuffer + offset + 0) = (u_isKeyframe.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->isKeyframe);
      for( uint32_t i = 0; i < 7; i++){
      union {
        float real;
        uint32_t base;
      } u_camToWorldi;
      u_camToWorldi.real = this->camToWorld[i];
      *(outbuffer + offset + 0) = (u_camToWorldi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_camToWorldi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_camToWorldi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_camToWorldi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->camToWorld[i]);
      }
      union {
        float real;
        uint32_t base;
      } u_fx;
      u_fx.real = this->fx;
      *(outbuffer + offset + 0) = (u_fx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fx);
      union {
        float real;
        uint32_t base;
      } u_fy;
      u_fy.real = this->fy;
      *(outbuffer + offset + 0) = (u_fy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fy.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fy);
      union {
        float real;
        uint32_t base;
      } u_cx;
      u_cx.real = this->cx;
      *(outbuffer + offset + 0) = (u_cx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cx);
      union {
        float real;
        uint32_t base;
      } u_cy;
      u_cy.real = this->cy;
      *(outbuffer + offset + 0) = (u_cy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cy.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cy);
      *(outbuffer + offset + 0) = (this->height >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->height >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->height >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->height >> (8 * 3)) & 0xFF;
      offset += sizeof(this->height);
      *(outbuffer + offset + 0) = (this->width >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->width >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->width >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->width >> (8 * 3)) & 0xFF;
      offset += sizeof(this->width);
      *(outbuffer + offset + 0) = (this->pointcloud_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pointcloud_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pointcloud_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pointcloud_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pointcloud_length);
      for( uint32_t i = 0; i < pointcloud_length; i++){
      *(outbuffer + offset + 0) = (this->pointcloud[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pointcloud[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_isKeyframe;
      u_isKeyframe.base = 0;
      u_isKeyframe.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->isKeyframe = u_isKeyframe.real;
      offset += sizeof(this->isKeyframe);
      for( uint32_t i = 0; i < 7; i++){
      union {
        float real;
        uint32_t base;
      } u_camToWorldi;
      u_camToWorldi.base = 0;
      u_camToWorldi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_camToWorldi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_camToWorldi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_camToWorldi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->camToWorld[i] = u_camToWorldi.real;
      offset += sizeof(this->camToWorld[i]);
      }
      union {
        float real;
        uint32_t base;
      } u_fx;
      u_fx.base = 0;
      u_fx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_fx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_fx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->fx = u_fx.real;
      offset += sizeof(this->fx);
      union {
        float real;
        uint32_t base;
      } u_fy;
      u_fy.base = 0;
      u_fy.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fy.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_fy.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_fy.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->fy = u_fy.real;
      offset += sizeof(this->fy);
      union {
        float real;
        uint32_t base;
      } u_cx;
      u_cx.base = 0;
      u_cx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cx = u_cx.real;
      offset += sizeof(this->cx);
      union {
        float real;
        uint32_t base;
      } u_cy;
      u_cy.base = 0;
      u_cy.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cy.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cy.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cy.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cy = u_cy.real;
      offset += sizeof(this->cy);
      this->height =  ((uint32_t) (*(inbuffer + offset)));
      this->height |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->height |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->height |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->height);
      this->width =  ((uint32_t) (*(inbuffer + offset)));
      this->width |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->width |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->width |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->width);
      uint32_t pointcloud_lengthT = ((uint32_t) (*(inbuffer + offset)));
      pointcloud_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      pointcloud_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      pointcloud_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->pointcloud_length);
      if(pointcloud_lengthT > pointcloud_length)
        this->pointcloud = (uint8_t*)realloc(this->pointcloud, pointcloud_lengthT * sizeof(uint8_t));
      pointcloud_length = pointcloud_lengthT;
      for( uint32_t i = 0; i < pointcloud_length; i++){
      this->st_pointcloud =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_pointcloud);
        memcpy( &(this->pointcloud[i]), &(this->st_pointcloud), sizeof(uint8_t));
      }
     return offset;
    }

    const char * getType(){ return "lsd_slam_viewer/KeyframeMsg"; };
    const char * getMD5(){ return "db0a795065d8e05013b43054a0c4cb01"; };

  };

}
#endif
