#include "SLAMData.h"

using namespace viewer_client;

void SLAMData::setCameraParams(double Fx, double Fy, double Cx, double Cy, int h, int w){
	this->camera=Camera{ Fx, Fy, Cx, Cy, h, w, 1 }; //dunno what do do for scale yet
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
	return this->camera.get_lastCamPose();
}
void SLAMData::updateCamTraj(geometry_msgs::PoseStamped pose){
	this->camera.updateCamTraj(pose);
}

void SLAMData::camParamsHandle(std::vector<double> data){
	int q = 0;
	this->camera.fx = data[q]; q++;
	this->camera.fy = data[q]; q++;
	this->camera.cx = data[q]; q++;
	this->camera.cy = data[q]; q++;
	this->camera.height = data[q]; q++;
	this->camera.width = data[q]; q++;
	this->camera.scae = 1;


}

void SLAMData::camPoseHandle(std::vector<double> data){
	this->updateCamTraj(helpers::readCameraPose(data));
}

void SLAMData::keyFramHandle(std::vector<double> data){

}

using namespace helpers;
		//Camera readCameraParams(std::vector<double> data){  }

		geometry_msgs::PoseStamped readCameraPose(std::vector<double> data){
		//extra is for scale in this case.
		//geometry_msgs::PoseStamped  readCameraPose(char *data, double* extra){
		geometry_msgs::PoseStamped cameraPose = geometry_msgs::PoseStamped();
		//maybe use SE3? depends on pcl	

		int q = 0;
		cameraPose.header = helpers::readHeader(data);
		q += 2;

		cameraPose.pose.position.x = data[q]; q++;
		cameraPose.pose.position.y = data[q]; q++;
		cameraPose.pose.position.z = data[q]; q++;

		cameraPose.pose.orientation.x = data[q]; q++;
		cameraPose.pose.orientation.y = data[q]; q++;
		cameraPose.pose.orientation.z = data[q]; q++;
		cameraPose.pose.orientation.w = data[q]; q++;

		//*extra = *q;
		return cameraPose;
	}

	std_msgs::Header readHeader(std::vector<double> data){
		int q = 0;

		std_msgs::Header header = std_msgs::Header();

		header.seq = data[0];
		header.stamp.fromSec(data[1]);

		return header;
	}

	pcl::PointXYZRGBA readPoint(std::vector<double> data){
		int q = 0;
		pcl::PointXYZRGBA point;
		double idepth = data[q]; q++;
		double idepthvar = data[q]; q++;
		point.a = data[q]; q++;
		point.r = data[q]; q++;
		point.g = data[q]; q++;
		point.b = data[q]; q++;

		return point;
	}


	void pointFromDepth(){

	}
