#pragma once

#include <vector>
#include "geometry_msgs\PoseStamped.h"
#include "pcl\point_cloud.h"
#include "pcl\point_types.h"

struct Camera{
		void setCameraParams(double Fx, double Fy, double Cx, double Cy);
		void updateCamTraj(geometry_msgs::PoseStamped pose);
		geometry_msgs::PoseStamped get_lastCamPose();

		double fx, fy;
		double cx, cy;
		int height; int weight;

		std::vector<geometry_msgs::PoseStamped> cameraTraj;

};

class SLAMData
{
public:
	SLAMData();
	virtual ~SLAMData();

	void updateCamTraj(geometry_msgs::PoseStamped pose);
	geometry_msgs::PoseStamped get_lastCamPose();
	void addUntrackedPoints(geometry_msgs::PoseStamped newPcl);

	;
private:
	
	Camera camera;
	pcl::PointCloud<pcl::PointXYZRGBA> pointcloud;

};

