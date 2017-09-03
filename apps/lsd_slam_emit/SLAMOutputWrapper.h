      /**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam>
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _LSD_SLAMOutputWrapper_h
#define _LSD_SLAMOutputWrapper_h


#include "lsd_slam\io_wrapper\output_3d_wrapper.h"
#include <vector>
#include <string>
#include "opencv2/core/core.hpp"
#include "boost\asio\io_service.hpp"  // For UDPSocket and SocketException

#include "UDPClient.h"
//
//#include "ros_lib/lsd_slam_viewer/keyframeGraphMsg.h"
//#include "ros_lib/lsd_slam_viewer/keyframeMsg.h"
//#include "ros_lib/geometry_msgs/PoseStamped.h"
//#include "ros_lib/ros.h"


namespace lsd_slam
{


class Frame;
class KeyFrameGraph;

struct InputPointDense
{
	float idepth;
	float idepth_var;
	unsigned char color[4];

	std::vector<float> serialize();
	//void deserialize(char *data);
};

struct Point3DDense{
	double point[3];
	unsigned char color[4];

	void serialize(char *data);
	void deserialize(char *data);
};
struct KeyFrameMessage
{
	int id;
	float time;
	bool isKeyframe;

	// camToWorld as serialization of sophus sim(3).
	// may change with keyframeGraph - updates.
	Sophus::Sim3f camToWorld;


	// camera parameter(fx fy cx cy), width, height
	// will never change, but required for display.
	float fx;
	float fy;
	float cx;
	float cy;
	unsigned int height;
	unsigned int width;
	// data as InputPointDense(float idepth, float idepth_var, uchar color[4]), width x height
	// may be empty, in that case no associated pointcloud is ever shown.
	std::vector<InputPointDense> pointcloud;


};

struct GraphConstraint
{
	int from;
	int to;
	float err;
};

struct GraphFramePose
{
	int id;
	float camToWorld[7];
};

struct KeyframeGraphMsg {

	int numFrames;

	std::vector<GraphFramePose>  frameData;

	int numConstraints;


	std::vector<unsigned char> constraintsData;


};


/** Addition to LiveSLAMWrapper for ROS interoperability. */
class SLAMOutputWrapper : public Output3DWrapper
{
public:

	// initializes cam-calib independent stuff
	//SLAMOutputWrapper(int width, int height, std::ostream &);
	SLAMOutputWrapper(int width, int height, int _pt_quota);
	~SLAMOutputWrapper();
	bool first; //or id=1


	virtual void publishKeyframeGraph(KeyFrameGraph* graph);

	// publishes a keyframe. if that frame already existis, it is overwritten, otherwise it is added.
	virtual void publishKeyframe(Frame* f);

	// published a tracked frame that did not become a keyframe (i.e. has no depth data)
	virtual void publishTrackedFrame(Frame* f);

	Point3DDense* pointFromKF(KeyFrameMessage kFm);

	// publishes graph and all constraints, as well as updated KF poses.
	virtual void publishTrajectory(std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> trajectory, std::string identifier);

	virtual void publishTrajectoryIncrement(const Eigen::Matrix<float, 3, 1>& pt, std::string identifier);

	virtual void publishDebugInfo(const Eigen::Matrix<float, 20, 1>& data);

	// these should be in Frame, but there is no point in modifying or extending 
	// that class and make organization complex
	//std::vector<double> serializeHeader(Frame *f);
	std::vector<float> serializeCameraParams(Frame *f);

	std::vector<double> serializeCameraPose(Frame *kf);
	std::vector<float> serializePoint(const float idepth, const float idepthvar, const float  color);
	std::vector<float> serializeCloud(Frame *f);


	//std::ofstream & _log;
	//no need to reaccess there
	int pixsize;
	int _pt_quota;

	int publishLvl;

private:
	boost::asio::io_service io_service;

	UDPClient _camera_params_client;
	UDPClient _camera_pose_client;
	UDPClient _keyframe_client;


	int width, height;

	std::string liveframe_channel;
	//ros::Publisher liveframe_publisher;

	std::string keyframe_channel;
	//ros::Publisher keyframe_publisher;

	std::string graph_channel;
	//ros::Publisher graph_publisher;

	std::string debugInfo_channel;
	//ros::Publisher debugInfo_publisher;


	std::string pose_channel;
	//ros::Publisher pose_publisher;

	cv::Mat tracker_display;

	//ros::NodeHandle nh_;
};
}

#endif
