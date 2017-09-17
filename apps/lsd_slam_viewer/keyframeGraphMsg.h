#pragma once

#include <vector>
#include <memory>

#include "sophus/sim3.hpp"
//#include "../util/SophusUtil.h"
//#include "../IOWrapper/OpenCV/WrapperTypes.h"

namespace lsd_slam
{

	class keyframeMsg
	{
	public:
		typedef std::shared_ptr<keyframeMsg> Ptr;
		static Ptr Create()
		{
			return Ptr(new keyframeMsg());
		}

		double
			time;
		int
			id;
		unsigned int
			width,
			height;
		bool
			isKeyframe;
		float
			fx, fy,
			cx, cy,
			scale;
		//std::vector<InputPointDense>pointcloud;
		//SE3 camToWorld;

	protected:
		keyframeMsg() :
			time(0),
			id(0),
			width(0),
			height(0),
			isKeyframe(false),
			fx(0),
			fy(0),
			cx(0),
			cy(0),
			scale(1)
		{};
	};

	typedef const keyframeMsg* keyframeMsgConstPtr;
}