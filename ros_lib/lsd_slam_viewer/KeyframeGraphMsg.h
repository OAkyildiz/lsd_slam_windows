#ifndef _ROS_lsd_slam_viewer_KeyframeGraphMsg_h
#define _ROS_lsd_slam_viewer_KeyframeGraphMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace lsd_slam_viewer
{

  class KeyframeGraphMsg : public ros::Msg
  {
    public:
      typedef uint32_t _numFrames_type;
      _numFrames_type numFrames;
      uint32_t frameData_length;
      typedef uint8_t _frameData_type;
      _frameData_type st_frameData;
      _frameData_type * frameData;
      typedef uint32_t _numConstraints_type;
      _numConstraints_type numConstraints;
      uint32_t constraintsData_length;
      typedef uint8_t _constraintsData_type;
      _constraintsData_type st_constraintsData;
      _constraintsData_type * constraintsData;

    KeyframeGraphMsg():
      numFrames(0),
      frameData_length(0), frameData(NULL),
      numConstraints(0),
      constraintsData_length(0), constraintsData(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->numFrames >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->numFrames >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->numFrames >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->numFrames >> (8 * 3)) & 0xFF;
      offset += sizeof(this->numFrames);
      *(outbuffer + offset + 0) = (this->frameData_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->frameData_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->frameData_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->frameData_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->frameData_length);
      for( uint32_t i = 0; i < frameData_length; i++){
      *(outbuffer + offset + 0) = (this->frameData[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->frameData[i]);
      }
      *(outbuffer + offset + 0) = (this->numConstraints >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->numConstraints >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->numConstraints >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->numConstraints >> (8 * 3)) & 0xFF;
      offset += sizeof(this->numConstraints);
      *(outbuffer + offset + 0) = (this->constraintsData_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->constraintsData_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->constraintsData_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->constraintsData_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->constraintsData_length);
      for( uint32_t i = 0; i < constraintsData_length; i++){
      *(outbuffer + offset + 0) = (this->constraintsData[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->constraintsData[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->numFrames =  ((uint32_t) (*(inbuffer + offset)));
      this->numFrames |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->numFrames |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->numFrames |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->numFrames);
      uint32_t frameData_lengthT = ((uint32_t) (*(inbuffer + offset)));
      frameData_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      frameData_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      frameData_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->frameData_length);
      if(frameData_lengthT > frameData_length)
        this->frameData = (uint8_t*)realloc(this->frameData, frameData_lengthT * sizeof(uint8_t));
      frameData_length = frameData_lengthT;
      for( uint32_t i = 0; i < frameData_length; i++){
      this->st_frameData =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_frameData);
        memcpy( &(this->frameData[i]), &(this->st_frameData), sizeof(uint8_t));
      }
      this->numConstraints =  ((uint32_t) (*(inbuffer + offset)));
      this->numConstraints |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->numConstraints |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->numConstraints |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->numConstraints);
      uint32_t constraintsData_lengthT = ((uint32_t) (*(inbuffer + offset)));
      constraintsData_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      constraintsData_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      constraintsData_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->constraintsData_length);
      if(constraintsData_lengthT > constraintsData_length)
        this->constraintsData = (uint8_t*)realloc(this->constraintsData, constraintsData_lengthT * sizeof(uint8_t));
      constraintsData_length = constraintsData_lengthT;
      for( uint32_t i = 0; i < constraintsData_length; i++){
      this->st_constraintsData =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_constraintsData);
        memcpy( &(this->constraintsData[i]), &(this->st_constraintsData), sizeof(uint8_t));
      }
     return offset;
    }

    const char * getType(){ return "lsd_slam_viewer/KeyframeGraphMsg"; };
    const char * getMD5(){ return "d23a8a86773b54db7399debf884d0c9e"; };

  };

}
#endif
