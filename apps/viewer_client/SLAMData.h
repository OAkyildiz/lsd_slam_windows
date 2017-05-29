#pragma once

#include <vector>
#include "geometry_msgs\PoseStamped.h"
#include "pcl\point_cloud.h"
#include "pcl\point_types.h"

struct Camera{
	public:
		void setCameraParams(double fx, double fy, double cx, double cy);
		void updateCamTraj(geometry_msgs::PoseStamped pose);
		geometry_msgs::PoseStamped get_lastCamPose();

	private:
		double fx, fy;
		double cx, cy;

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

