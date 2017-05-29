#include "SLAMData.h"


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