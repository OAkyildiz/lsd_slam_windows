#include "SLAMData.h"

using namespace viewer_client;

/*void SLAMData::setCameraParams(double Fx, double Fy, double Cx, double Cy, int h, int w){
	this->camera=Camera{ Fx, Fy, Cx, Cy, h, w, 1 }; //dunno what do do for scale yet
}*/



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

void SLAMData::camParamsHandle(std::vector<float> data){
	int q = 0;
	this->camera.fx = data[q]; q++;
	this->camera.fy = data[q]; q++;
	this->camera.cx = data[q]; q++;
	this->camera.cy = data[q]; q++;
	this->camera.height =(int) data[q]; q++;
	this->camera.width =(int) data[q];
	this->camera.scale = 1;
	this->pixsize = this->camera.height * this->camera.width;
	std::cout << pixsize << std::endl;

}

void SLAMData::camPoseHandle(std::vector<double> data){
	this->updateCamTraj(SLAMData::readCameraPose(data));
}

void SLAMData::keyFramHandle(std::vector<float> data){
	this->keyframes.push_back(readKeyrame(data));
	
}


		//Camera readCameraParams(std::vector<double> data){  }

geometry_msgs::PoseStamped  SLAMData::readCameraPose(std::vector<double> data){
		//extra is for scale in this case.
		//geometry_msgs::PoseStamped  readCameraPose(char *data, double* extra){
		geometry_msgs::PoseStamped cameraPose;
		//maybe use SE3? depends on pcl	

		int q = 0;
		cameraPose.header.seq = data[q]; q++;
		cameraPose.header.stamp.fromSec(data[q]); q++;

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



InputCloud  SLAMData::readKeyrame(std::vector<float> data){
	InputCloud kfm;
	const float tstmp_f[2] = { data[1], data[2] }; // combine 4byte-wrods into double

	int q = 0;
	kfm.header.seq = data[0]; q++;
	kfm.header.stamp.fromSec(*reinterpret_cast<const double*>(tstmp_f)); q++;
	std::cout << "point_100:" << data[100 + 3] << ", " << data[pixsize+100 + 3] << std::endl;

	for (int i = 3; i < pixsize + 3; i++){
		InputPointDense ipt;
		ipt.idepth = data[i];
		ipt.idepth_var = data[pixsize + i];
		ipt.color.hex = data[2 * pixsize + i];
	
	

		int b = sizeof(ipt.color);
		int c = sizeof(unsigned char);
		std::cout << "color size test(4?): "<<  b/c  << std::endl;
		kfm.input_points.push_back(ipt);
	}
	std::cout << "point_c_100:" << kfm.input_points[100].idepth << ", " << kfm.input_points[100].idepth_var<< std::endl;

	return kfm;
}

/*pcl::PointXYZRGBA  SLAMData::readPoint(std::vector<double> data){
		int q = 0;
		pcl::PointXYZRGBA point;
		double idepth = data[q]; q++;
		double idepthvar = data[q]; q++;
		point.a = data[q]; q++;
		point.r = data[q]; q++;
		point.g = data[q]; q++;
		point.b = data[q]; q++;

		return point;
	}*/


//void  SLAMData::pointFromDepth()}
