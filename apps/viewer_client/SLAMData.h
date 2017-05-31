#pragma once

#include <vector>
#include "geometry_msgs\PoseStamped.h"
#include "pcl\point_cloud.h"
#include "pcl\point_types.h"

namespace viewer_client{
	struct Camera{
		void updateCamTraj(geometry_msgs::PoseStamped pose);
		geometry_msgs::PoseStamped get_lastCamPose();

		double fx, fy;
		double cx, cy;
		int height; int width;

		double scale;
		std::vector<geometry_msgs::PoseStamped> cameraTraj;

	};

	class SLAMData
	{
	public:
		SLAMData();
		virtual ~SLAMData();

		void setCameraParams(double Fx, double Fy, double Cx, double Cy, int h, int w);

		void updateCamTraj(geometry_msgs::PoseStamped pose);
		geometry_msgs::PoseStamped get_lastCamPose();
		void addUntrackedPoints(geometry_msgs::PoseStamped newPcl);

		//handlers

		void camPoseHandle(std::vector<double> data);
		void camParamsHandle(std::vector<double> data);
		void keyFramHandle(std::vector<double> data);
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
		
	private:

		Camera camera;
		pcl::PointCloud<pcl::PointXYZRGBA> pointcloud;

	};

	namespace helpers{
		//Camera readCameraParams(std::vector<double> data);
		geometry_msgs::PoseStamped readCameraPose(std::vector<double> data);
		std_msgs::Header readHeader(std::vector<double> data);
		pcl::PointXYZRGBA readPoint(std::vector<double> data);

	}
}