#pragma once

#include <vector>
#include "geometry_msgs\PoseStamped.h"
#include "lsd_slam_viewer\KeyframeMsg.h"
#include "pcl\point_cloud.h"
#include "pcl\point_types.h"

#define PCL_SIZE 926103

namespace viewer_client{

	union argb_float{
		float hex;
		unsigned char vals[4];
	};

	struct InputPointDense
	{
		float idepth;
		float idepth_var;
		argb_float color; //size=4

		//void serialize(char *data);

	};

	struct InputCloud{ //or keyframe-similar object
		std_msgs::Header header;

		std::vector<InputPointDense>  input_points;
		//maybe an associated camera pose?

	};

	struct Camera{
		void updateCamTraj(geometry_msgs::PoseStamped pose);
		geometry_msgs::PoseStamped get_lastCamPose();

		float fx, fy;
		float cx, cy;
		int height; int width;

		double scale;
		std::vector<geometry_msgs::PoseStamped> cameraTraj;

	};

	class SLAMData
	{
	public:
		SLAMData();
		virtual ~SLAMData();
		Camera getCamera(){ return this->camera; }
		//void setCameraParams(double Fx, double Fy, double Cx, double Cy, int h, int w);

		void updateCamTraj(geometry_msgs::PoseStamped pose);
		geometry_msgs::PoseStamped get_lastCamPose();
		void addUntrackedPoints(geometry_msgs::PoseStamped newPcl);
		 

		//handlers
		void camPoseHandle(std::vector<double> data);
		void camParamsHandle(std::vector<float> data);
		void keyFramHandle(std::vector<float> data);

		geometry_msgs::PoseStamped readCameraPose(std::vector<double> data);
		//std_msgs::Header readHeader(std::vector<double> data);
		//pcl::PointXYZRGBA readPoint(std::vector<double> data);
		InputCloud readKeyrame(std::vector<float> data);

		//void camPoseHandle(std::vector<double> data);

		//slam parameters
		const bool keepInMemory = true;
		const int minNearSupport = 9;
		const//bool paramsStillGood =  == 
		const double scaledDepthVarTH = -5.2; //scaledDepthVarTH
		const double absDepthVarTH = -2; //absDepthVarTH &&
		//	my_scale*1.2 > camToWorld.scale() &&
		//	my_scale < camToWorld.scale()*1.2 &&
		//	my_minNearSupport == minNearSupport &&
		int sparsifyFactor = 42;
		int pixsize =1;

	private:

		Camera camera;
		pcl::PointCloud<pcl::PointXYZRGBA> pointcloud;
		std::vector<InputCloud> keyframes;
	};

	

}