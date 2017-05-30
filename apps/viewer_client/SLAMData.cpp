#include "SLAMData.h"


void Camera::setCameraParams(double Fx, double Fy, double Cx, double Cy){
	this->fx = Fx;	this->fy = Fy;
	this->cx = Cx;	this->cy = Cy;
	
}



void Camera::updateCamTraj(geometry_msgs::PoseStamped pose){
	cameraTraj.push_back(pose);
}
geometry_msgs::PoseStamped Camera::get_lastCamPose(){
	if (cameraTraj.empty()){
		return geometry_msgs::PoseStamped();
	}
	else
		return cameraTraj.back();
}

SLAMData::SLAMData()
{
}


SLAMData::~SLAMData()
{
}



geometry_msgs::PoseStamped SLAMData::get_lastCamPose(){
	return camera.get_lastCamPose();
}
void SLAMData::updateCamTraj(geometry_msgs::PoseStamped pose){
	camera.updateCamTraj(pose);
}