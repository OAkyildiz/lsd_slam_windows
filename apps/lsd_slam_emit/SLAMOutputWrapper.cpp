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

#include "SLAMOutputWrapper.h"
#include "depth2Points.h"
#include "lsd_slam\util\sophus_util.h"
#include "lsd_slam\util\settings.h"

//#include "ros_lib/lsd_slam_viewer/keyframeGraphMsg.h"
//#include "../ros_lib/lsd_slam_viewer/keyframeMsg.h"
#include "..\ros_lib\geometry_msgs\PoseStamped.h"
//#include "ros_lib/ros.h"
//#include "ros_lib/ros/node_handle.h"

#include "lsd_slam/model/frame.h"
#include "lsd_slam/global_mapping/key_frame_graph.h"
#include "sophus/sim3.hpp"
#include "lsd_slam/global_mapping/g2o_type_sim3_sophus.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "boost/thread.hpp"
#include <fstream>
#include <iostream>

#define PACKETSIZE 512
#define POINT_QUOTA 5000
namespace lsd_slam
{
	std::string localhost = "127.0.0.1";
	
	
	//SLAMOutputWrapper::SLAMOutputWrapper(int width, int height, std::ostream &):
	SLAMOutputWrapper::SLAMOutputWrapper(int width, int height) :
	io_service(),
	//_log(),
	_camera_params_client(io_service, LOCAL_PORT_CAM_PARAMS, localhost, std::to_string(PORT_CAM_PARAMS)),
	_camera_pose_client(io_service, LOCAL_PORT_CAM_POSE, localhost, std::to_string(PORT_CAM_POSE)),
	_keyframe_client(io_service, LOCAL_PORT_KEYFRAME, localhost, std::to_string(PORT_KEYFRAME))
	{
	boost::thread _udp_thread((boost::bind(&boost::asio::io_service::run, &io_service)));
	//_log = std::ofstream();
	//_log.open("slam_log_1.txt");
	//make the window part of the class
	//actually replace it with cv::viz

	//cvNamedWindow("Tracking_output", 1); //Create window
	this->width = width;
	this->height = height;
	this->first = true;

	/* list of the original publishers*/
	// "lsd_slam/liveframes"; liveframe_publisher, keyframeMsg
	// "lsd_slam/keyframes"; keyframe_publisher, keyframeMsg
	// "lsd_slam/graph"; graph_publisher, keyframeGraphMsg
	// "lsd_slam/debug" debugInfo_publisher, Float32MultiArray
	// "lsd_slam/pose";  pose_publisher, PoseStamped
	

	tracker_display = cv::Mat::ones(640, 480, CV_8UC1);
	cv::circle(tracker_display, cv::Point(100,100), 20, cv::Scalar(0, 255, 0));
	cv::imshow("Tracking_output", tracker_display);
	cvWaitKey(20);
	publishLvl=0;
}

SLAMOutputWrapper::~SLAMOutputWrapper()
{
	//_log.close();
	io_service.stop();

}


void SLAMOutputWrapper::publishKeyframe(Frame* f)
{
	//	KeyFrameMessage fMsg;
	boost::shared_lock<boost::shared_mutex> lock = f->getActiveLock();

	// with ros mesasages
	//fMsg.header.id = f->id(); //fMsg.header.time = f->timestamp();\

	bool isKeyframe = true;
	if (this->first){
		std::vector<float> paramsData = serializeCameraParams(f);
		std::cout << "Sent camera params"<< std::endl;
		_camera_params_client.send(paramsData);
		pixsize = f->width(publishLvl)* f->height(publishLvl);
		this->first = false;
	}

	//std::cout << "Cam_Parametyeres@PKeyF: " << fMsg.fx << ", " << fMsg.fy << ", " << fMsg.cx << ", " << fMsg.cy << std::endl;
	//keyframe_publisher.publish(fMsg);
	std::vector<float> input_cloud = serializeCloud(f);
	//_log << "Writing pcl...\n";
	//_log << input_cloud;
	std::cout << "["<< f->timestamp()<<"] "<< "KF #" <<f->id() << " was sent: "<< input_cloud.size()<< " px"<< std::endl;

	_keyframe_client.send(input_cloud);//try throttling by factors with a button
									//find package numbers needed, but do visualizng with less points first
}

void SLAMOutputWrapper::publishTrackedFrame(Frame* tf)
{
	boost::shared_lock<boost::shared_mutex> lock = tf->getActiveLock();

	//KeyFrameMessage fMsg; //lsd_slam_viewer::KeyframeMsg fMsg;
	bool isKeyframe = false;
	//std::cout << "ID"<< fMsg.id << std::endl;
	//std::cout << "stamp: " << kf->timestamp() << std::endl;

	//	memcpy(fMsg.camToWorld.data(), kf->getScaledCamToWorld().cast<float>().data(), sizeof(float) * 7);

	if (this->first){
		std::vector<float> paramsData = serializeCameraParams(tf);
		std::cout << "Sent camera params" << std::endl;
		pixsize = tf->width(publishLvl)* tf->height(publishLvl);
		_camera_params_client.send(paramsData);
		this->first = false;
	}
	if (tf->hasIDepthBeenSet()){
		std::cout << "IDepth was set" << std::endl;

		std::vector<float> input_cloud = serializeCloud(tf);
		std::cout << "[" << tf->timestamp() << "] " << "TF #" << tf->id() << " was sent: " << input_cloud.size() << " px" << std::endl;
		_keyframe_client.send(input_cloud);
	}
	//keyframe_publisher.publish(fMsg);

	//std::cout << "#_TfPoints: " << fMsg.pointcloud.size() << std::endl;
	//fMsg.pointcloud.clear();
	// liveframe_publisher.publish(fMsg);



	//std::cout << "Sent camera pose" << std::endl;

	/*std::cout << "Camera Pose: " << pmsg.pose.position.x << ", " << pmsg.pose.position.y << ", " << pmsg.pose.position.z << ";( " <<
	pmsg.pose.orientation.x << ", " << pmsg.pose.orientation.y << ", " << pmsg.pose.orientation.z << ", " << pmsg.pose.orientation.w << " )" << std::endl;*/

	// pmsg.header.stamp = ros::Time::now(); // pmsg.header.stamp = 0; // pmsg.header.frame_id = "world"; // pose_publisher.publish(pMsg);
	SE3 camToWorld = se3FromSim3(tf->getScaledCamToWorld()); //here for now
	cv::circle(tracker_display, cv::Point(320+camToWorld.translation()[0]*640, -240 + camToWorld.translation()[1]*480), 2, cv::Scalar(255, 0, 0),4);
	cv::imshow("Tracking_output", tracker_display);
	//cv::waitKey(20);
	std::cout << "Published camera pose: " << camToWorld.translation()[0] << " " << camToWorld.translation()[1] << "  " << camToWorld.translation()[2] << std::endl;
	std::vector<double> poseData = serializeCameraPose(tf);
	_camera_pose_client.send(poseData);
	//std::cout << "# tf Points: " << fMsg.pointcloud.size() << std::endl;
	//_camera_pose_client.send("Sent Trackedframe \n");

}
// TODO: add htons() to these



Point3DDense* SLAMOutputWrapper::pointFromKF(KeyFrameMessage kFm)
{
	bool keepInMemory = true;
	int	minNearSupport = 9;
	//bool paramsStillGood =  == 
	double scaledDepthVarTH = -5.2; //scaledDepthVarTH
	double absDepthVarTH = -2; //absDepthVarTH &&
	//	my_scale*1.2 > camToWorld.scale() &&
	//	my_scale < camToWorld.scale()*1.2 &&
	//	my_minNearSupport == minNearSupport &&
	int sparsifyFactor = 42;
	float fyi = kFm.fy;
	float fxi = kFm.fy;
	float cxi = kFm.fy;
	float cyi = kFm.fy;

	int width = kFm.width;
	int height = kFm.height;

	kFm.pointcloud.resize(width*height*sizeof(InputPointDense));
	InputPointDense* originalInput = (InputPointDense*)kFm.pointcloud.data();

	//if (glBuffersValid && (paramsStillGood || numRefreshedAlready > 10)) return;
	//numRefreshedAlready++;

	//glBuffersValid = true;

	//// delete old vertex buffer
	//if (vertexBufferIdValid)
	//{
	//	glDeleteBuffers(1, &vertexBufferId);
	//	vertexBufferIdValid = false;
	//}

	//// if there are no vertices, done!
	if (originalInput == 0)
		return 0;


	//// make data
	Point3DDense* tmpBuffer = new Point3DDense[width*height];

	double my_scaledTH = scaledDepthVarTH;
	double my_absTH = absDepthVarTH;
	double my_scale = kFm.camToWorld.scale();
	int my_minNearSupport = minNearSupport;
	int my_sparsifyFactor = sparsifyFactor;
	//// data is directly in ros message, in correct format.
	int vertexBufferNumPoints = 0;

	int total = 0, displayed = 0;
	for (int y = 1; y<height - 1; y++){
		for (int x = 1; x < width - 1; x++)
		{
			if (originalInput[x + y*width].idepth <= 0) continue;
			total++;


			if (my_sparsifyFactor > 1 && rand() % my_sparsifyFactor != 0) continue;

			float depth = 1 / originalInput[x + y*width].idepth;
			float depth4 = depth*depth; depth4 *= depth4;


			if (originalInput[x + y*width].idepth_var * depth4 > my_scaledTH)
				continue;

			if (originalInput[x + y*width].idepth_var * depth4 * my_scale*my_scale > my_absTH)
				continue;

			if (my_minNearSupport > 1)
			{
				int nearSupport = 0;
				for (int dx = -1; dx < 2; dx++){
					for (int dy = -1; dy < 2; dy++)
					{
						int idx = x + dx + (y + dy)*width;
						if (originalInput[idx].idepth > 0)
						{
							float diff = originalInput[idx].idepth - 1.0f / depth;
							if (diff*diff < 2 * originalInput[x + y*width].idepth_var)
								nearSupport++;
						}
					}
				}

				if (nearSupport < my_minNearSupport)
					continue;
			}


			tmpBuffer[vertexBufferNumPoints].point[0] = (x*fxi + cxi) * depth;
			tmpBuffer[vertexBufferNumPoints].point[1] = (y*fyi + cyi) * depth;
			tmpBuffer[vertexBufferNumPoints].point[2] = depth;

			tmpBuffer[vertexBufferNumPoints].color[3] = 100;
			tmpBuffer[vertexBufferNumPoints].color[2] = originalInput[x + y*width].color[0];
			tmpBuffer[vertexBufferNumPoints].color[1] = originalInput[x + y*width].color[1];
			tmpBuffer[vertexBufferNumPoints].color[0] = originalInput[x + y*width].color[2];

			vertexBufferNumPoints++;
			displayed++;

		}
	}

	//totalPoints = total;
	//displayedPoints = displayed;

	//// create new ones, static
	//vertexBufferId = 0;
	//glGenBuffers(1, &vertexBufferId);
	//glBindBuffer(GL_ARRAY_BUFFER, vertexBufferId);         // for vertex coordinates
	//glBufferData(GL_ARRAY_BUFFER, sizeof(MyVertex) * vertexBufferNumPoints, tmpBuffer, GL_STATIC_DRAW);
	//vertexBufferIdValid = true;

	if (!keepInMemory)
	{
		delete[] originalInput;
		originalInput = 0;
	}

	return tmpBuffer;
}

void SLAMOutputWrapper::publishKeyframeGraph(KeyFrameGraph* graph)
{
	KeyframeGraphMsg gMsg;

	graph->edgesListsMutex.lock();
	gMsg.numConstraints = graph->edgesAll.size();
	gMsg.constraintsData.resize(gMsg.numConstraints * sizeof(GraphConstraint));
	GraphConstraint* constraintData = (GraphConstraint*)gMsg.constraintsData.data();
	for(unsigned int i=0;i<graph->edgesAll.size();i++)
	{
		constraintData[i].from = graph->edgesAll[i]->firstFrame->id();
		constraintData[i].to = graph->edgesAll[i]->secondFrame->id();
		Sophus::Vector7d err = graph->edgesAll[i]->edge->error();
		constraintData[i].err = sqrt(err.dot(err));
	}
	graph->edgesListsMutex.unlock();

	graph->keyframesAllMutex.lock_shared();
	gMsg.numFrames = graph->keyframesAll.size();
	gMsg.frameData.resize(gMsg.numFrames * sizeof(GraphFramePose));
	GraphFramePose* framePoseData = (GraphFramePose*)gMsg.frameData.data();
	for(unsigned int i=0;i<graph->keyframesAll.size();i++)
	{
		framePoseData[i].id = graph->keyframesAll[i]->id();
		memcpy(framePoseData[i].camToWorld, graph->keyframesAll[i]->getScaledCamToWorld().cast<float>().data(),sizeof(float)*7);
	}
	graph->keyframesAllMutex.unlock_shared();
	std::cout << "Sent KeyframeGraph: " << gMsg.numFrames <<" frames"<<std::endl;
	//graph_publisher.publish(gMsg);
}

void SLAMOutputWrapper::publishTrajectory(std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> trajectory, std::string identifier)
{
	// unimplemented ... do i need it?
}

void SLAMOutputWrapper::publishTrajectoryIncrement(const Eigen::Matrix<float, 3, 1>& pt, std::string identifier)
{
	// unimplemented ... do i need it?
}

void SLAMOutputWrapper::publishDebugInfo(const Eigen::Matrix<float, 20, 1>& data)
{
	//std_msgs::Float32MultiArray msg;
	for(int i=0;i<20;i++)
		std::cout << (float)(data[i]) << std::endl;

	//debugInfo_publisher.publish(msg);
}

//void draw_target(cv::Mat& rgb_img, look3d::PanoramicTracker& tracker) {
//	const Eigen::Vector4d point_x(0.1, 0, 1, 1);
//	const Eigen::Vector4d point_y(0, 0.1, 1, 1);
//	const Eigen::Vector4d point_z(0, 0, 1.1, 1);
//	const Eigen::Vector4d point_target(0, 0, 1.0, 1);
//
//	Eigen::Matrix<double, 3, 4> proj = get_projection(tracker);
//
//	Eigen::Vector3d point_cam = proj * point_target;
//	Eigen::Vector3d pointx_cam = proj * point_x;
//	Eigen::Vector3d pointy_cam = proj * point_y;
//	Eigen::Vector3d pointz_cam = proj * point_z;
//
//	cv::line(rgb_img, cv::Point(point_cam[0], point_cam[1]), cv::Point(pointx_cam[0], pointx_cam[1]), cv::Scalar(255, 0, 0), 3);
//	cv::line(rgb_img, cv::Point(point_cam[0], point_cam[1]), cv::Point(pointy_cam[0], pointy_cam[1]), cv::Scalar(0, 255, 0), 3);
//	cv::line(rgb_img, cv::Point(point_cam[0], point_cam[1]), cv::Point(pointz_cam[0], pointz_cam[1]), cv::Scalar(0, 0, 255), 3);
//}


/*std::vector<double> SLAMOutputWrapper::serializeHeader(Frame *f){
	std::vector<double> data(2, 0);


	data[0] = f->id();
	data[1] = f->timestamp();

	return data;
}*/

std::vector<float> SLAMOutputWrapper::serializeCameraParams(Frame *f){
 	std::vector<float> data(6, 0);

	int q = 0;

	//serializeCamera;
	data[q] = f->fx(publishLvl); q++;
	data[q] = f->fy(publishLvl); q++;
	data[q] = f->cx(publishLvl); q++;
	data[q] = f->cy(publishLvl); q++;
	data[q] = f->height(publishLvl); q++;
	data[q] = f->width(publishLvl);
	std::cout << " Params Size:" << data.size() << std::endl;

	return data;


}
// we sould hsare serializeHeader() fcn but phose always accompanies a header, so lets keep it simple
std::vector<double> SLAMOutputWrapper::serializeCameraPose(Frame *kf){
	std::vector<double> data(9,0);
	SE3 camToWorld = se3FromSim3(kf->getScaledCamToWorld());
	int q = 0;


	data[q] = kf->id(); q++;
	data[q] = kf->timestamp(); q++;
	//obv. this can also be done at once with a pointer to ctw.translation()
	data[q] = camToWorld.translation()[0]; q++;
	data[q] = camToWorld.translation()[1]; q++;
	data[q] = camToWorld.translation()[2]; q++;
	int c = 1;
	if (camToWorld.so3().unit_quaternion().w() < 0)
		c = -1;
	data[q] = c * camToWorld.so3().unit_quaternion().x(); q++;
	data[q] = c * camToWorld.so3().unit_quaternion().y(); q++;
	data[q] = c * camToWorld.so3().unit_quaternion().z(); q++;
	data[q] = c * camToWorld.so3().unit_quaternion().w(); q++;
	//data[q] = camToWorld.scale(); already scaled?
	//std::cout << " Pose Size:" << data.size() << std::endl;

	return data;

}

std::vector<float> SLAMOutputWrapper::serializePoint(const float idepth, const float idepth_var, const float color){
	std::vector<float> data(2, 0);
	int q = 0;

	data[q] = idepth; q++;
	data[q] = idepth_var; q++;
	data[q] = color; // this is conveniently packed as floats
					// which is 4 u_chars, will be deserialized at the endpoint


	return data;


}



std::vector<float> SLAMOutputWrapper::serializeCloud(Frame *f){
	
	//int pixsize = this->pixsize;
	int pixsize = POINT_QUOTA;
	double tstmp_d = f->timestamp();
	const float* tstmp_f = reinterpret_cast<const float*>(&tstmp_d); // split double into 4byte-words
	

	const float* idepth = f->idepth(publishLvl);  //check size of that
	const float* idepthvar = f->idepthVar(publishLvl);
	const float* color = f->image(publishLvl);

	//t_size size = pixsize * 3 + 3
	std::vector<float> data; // header+ pointcloud

	data.push_back(f->id());
	data.push_back(tstmp_f[0]);
	data.push_back(tstmp_f[1]);

	std::ofstream log;
	log.open("slam_log.txt");
	data.insert(data.end(), idepth, idepth + POINT_QUOTA);
	data.insert(data.end(), idepthvar, idepthvar + POINT_QUOTA);
	data.insert(data.end(), color, color + POINT_QUOTA);
	/*for (int idx = 0; idx < pixsize; idx++) {
		std::vector<float> pointData = serializePoint(idepth[idx], idepthvar[idx], color[idx]);
	}*/
	std::cout << "point_100:" << data[100 + 3] << ", " << data[pixsize + 100 + 3] << std::endl;
	std::cout << "Size: " << data.size() << std::endl;

	return data;
}

std::vector<float>  InputPointDense::serialize(){
	std::vector<float> data(3, 0);
	int q = 0;
	
	data[q] = idepth; q++;
	data[q] = idepth_var; q++;
	data[q] = color[0];
	data[q] = color[1];
	data[q] = color[2];
	data[q] = color[3];

	return data;

}

/*void InputPointDense::deserialize(char *data){
	int *q = (int*)data;
	this->idepth = *q;       q++;
	this->idepth_var = *q;   q++;
	for (int i = 0; i++; i < 4){
		this->color[i] = *q;     q++;
	}

	char *p = (char*)q;
	
}*/


void  Point3DDense::serialize(char* data){
	double *q = (double*)data;
	
	for (int j = 0; j++; j < 3){
		*q = this->point[j];     q++;
	}

	for (int i = 0; i++; i < 4){
		*q = this->color[i];     q++;
	}

}


void Point3DDense::deserialize(char *data)
{
	double *q = (double*)data;

	for (int j = 0; j++; j < 3){
		this->point[j] = *q;     q++;
	}

	for (int i = 0; i++; i < 4){
		this->color[i] = *q;     q++;
	}
}

}