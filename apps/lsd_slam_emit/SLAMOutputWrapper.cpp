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

#include "boost\asio\io_service.hpp"  // For UDPSocket and SocketException

#include "UDPClient.h"

namespace lsd_slam
{
	boost::asio::io_service io_service;
	UDPClient client(io_service, "127.0.0.1", "14");

	SLAMOutputWrapper::SLAMOutputWrapper(int width, int height){
	//make the vindow part of the class
	//actually replace it with cv::viz

	//cvNamedWindow("Tracking_output", 1); //Create window
	this->width = width;
	this->height = height;

	//!!
	// Although the code will compile, we don't want the actual subscribers and publishers, or the nodehandle.
	// Ros Master will not work on Windows (at least with current config
	////////

	/*liveframe_channel = nh_.resolveName("lsd_slam/liveframes");
	liveframe_publisher = nh_.advertise<lsd_slam_viewer::keyframeMsg>(liveframe_channel,1);

	keyframe_channel = nh_.resolveName("lsd_slam/keyframes");
	keyframe_publisher = nh_.advertise<lsd_slam_viewer::keyframeMsg>(keyframe_channel,1);

	graph_channel = nh_.resolveName("lsd_slam/graph");
	graph_publisher = nh_.advertise<lsd_slam_viewer::keyframeGraphMsg>(graph_channel,1);

	debugInfo_channel = nh_.resolveName("lsd_slam/debug");
	debugInfo_publisher = nh_.advertise<std_msgs::Float32MultiArray>(debugInfo_channel,1);


	pose_channel = nh_.resolveName("lsd_slam/pose");*/
	//pose_publisher = nh_.advertise<geometry_msgs::PoseStamped>(pose_channel,1);

	//boost::asio::io_service io_service;
	//UDPClient client(io_service, "0.0.0.0", "1027");

	tracker_display = cv::Mat::ones(640, 480, CV_8UC1);
	cv::circle(tracker_display, cv::Point(100,100), 20, cv::Scalar(0, 255, 0));
	cv::imshow("Tracking_output", tracker_display);
	cvWaitKey(10);
	publishLvl=0;

	

	//client.send("Hello, World!");
}

SLAMOutputWrapper::~SLAMOutputWrapper()
{
}


void SLAMOutputWrapper::publishKeyframe(Frame* f)
{
	KeyFrameMessage fMsg;


	boost::shared_lock<boost::shared_mutex> lock = f->getActiveLock();

	fMsg.id = f->id();
	fMsg.time = f->timestamp();

	//fMsg.header.id = f->id();
	//fMsg.header.time = f->timestamp();\

	fMsg.isKeyframe = true;

	int w = f->width(publishLvl);
	int h = f->height(publishLvl);

	memcpy(fMsg.camToWorld.data(), f->getScaledCamToWorld().cast<float>().data(), sizeof(float)*7);
	fMsg.fx = f->fx(publishLvl);
	fMsg.fy = f->fy(publishLvl);
	fMsg.cx = f->cx(publishLvl);
	fMsg.cy = f->cy(publishLvl);
	fMsg.width = w;
	fMsg.height = h;



	fMsg.pointcloud.resize(w*h*sizeof(InputPointDense));

	InputPointDense* pc = (InputPointDense*)fMsg.pointcloud.data();

	const float* idepth = f->idepth(publishLvl);
	const float* idepthvar = f->idepthVar(publishLvl);
	const float* color = f->image(publishLvl);
	int idx;
	for(idx =0;idx < w*h; idx++)
	{
		pc[idx].idepth = idepth[idx];
		pc[idx].idepth_var = idepthvar[idx];
		pc[idx].color[0] = color[idx];
		pc[idx].color[1] = color[idx];
		pc[idx].color[2] = color[idx];
		pc[idx].color[3] = color[idx];
	}
	std::cout << "Cam_Parametyeres@PKeyF: " << fMsg.fx << ", " << fMsg.fy << ", " << fMsg.cx << ", " << fMsg.cy << std::endl;

	//keyframe_publisher.publish(fMsg);
	std::cout << "#_KfPoints: " << fMsg.pointcloud.size() << std::endl;

	client.send("Sent Keyframe \n");
}

void SLAMOutputWrapper::publishTrackedFrame(Frame* kf)
{
	KeyFrameMessage fMsg;
	//lsd_slam_viewer::KeyframeMsg fMsg;


	fMsg.id = kf->id();
	fMsg.time = kf->timestamp();

	//fMsg.header.id = kf->id();
	//fMsg.stamp = kf->timestamp();
	fMsg.isKeyframe = false;


	memcpy(fMsg.camToWorld.data(),kf->getScaledCamToWorld().cast<float>().data(),sizeof(float)*7);
	fMsg.fx = kf->fx(publishLvl);
	fMsg.fy = kf->fy(publishLvl);
	fMsg.cx = kf->cx(publishLvl);
	fMsg.cy = kf->cy(publishLvl);
	fMsg.width = kf->width(publishLvl);
	fMsg.height = kf->height(publishLvl);

	std::cout << "Cam_Parametyeres@PTrackedF: " << fMsg.fx << ", " << fMsg.fy << ", " << fMsg.cx << ", " << fMsg.cy << std::endl;
	std::cout << "# tf0 Points: " << fMsg.pointcloud.size() << std::endl;
	fMsg.pointcloud.clear();

	////liveframe_publisher.publish(fMsg);


	SE3 camToWorld = se3FromSim3(kf->getScaledCamToWorld());

	geometry_msgs::PoseStamped pmsg;

	pmsg.pose.position.x = camToWorld.translation()[0];
	pmsg.pose.position.y = camToWorld.translation()[1];
	pmsg.pose.position.z = camToWorld.translation()[2];
	pmsg.pose.orientation.x = camToWorld.so3().unit_quaternion().x();
	pmsg.pose.orientation.y = camToWorld.so3().unit_quaternion().y();
	pmsg.pose.orientation.z = camToWorld.so3().unit_quaternion().z();
	pmsg.pose.orientation.w = camToWorld.so3().unit_quaternion().w();

	std::cout << "Camera Pose: " << pmsg.pose.position.x << ", " <<
		pmsg.pose.position.y << ", " <<
		pmsg.pose.position.z << ";( " <<
		pmsg.pose.orientation.x << ", " <<
		pmsg.pose.orientation.y << ", " <<
		pmsg.pose.orientation.z << ", " <<
		pmsg.pose.orientation.w << " )" << std::endl;

	if (pmsg.pose.orientation.w < 0)
	{
		pmsg.pose.orientation.x *= -1;
		pmsg.pose.orientation.y *= -1;
		pmsg.pose.orientation.z *= -1;
		pmsg.pose.orientation.w *= -1;
	}

	//pmsg.header.stamp = ros::Time::now();
//	pmsg.header.stamp = 0;
	pmsg.header.frame_id = "world";
	//pose_publisher.publish(pMsg);

	pointFromKF(fMsg);
	cv::circle(tracker_display, cv::Point(320+camToWorld.translation()[0]*640, -240 + camToWorld.translation()[1]*480), 2, cv::Scalar(255, 0, 0),4);
	cv::imshow("Tracking_output", tracker_display);
	std::cout << "PublishTrackedKeyframe: " << camToWorld.translation()[0] << " " << camToWorld.translation()[1] << "  " << camToWorld.translation()[2] << std::endl;

	std::cout << "# tf Points: " << fMsg.pointcloud.size() << std::endl;
	client.send("Sent TrackedKeyframe \n");

}

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


void  InputPointDense::serialize(char* data){
	int *q = (int*)data;
	*q = idepth;       q++;
	*q = idepth_var;   q++;
	
	for (int i = 0; i++; i < 3){
		*q = color[i];     q++;
	}
	int i = 0;

}
void InputPointDense::deserialize(char *data){
	int *q = (int*)data;
	idepth = *q;       q++;
	idepth_var = *q;   q++;
	for (int i = 0; i++; i < 4){
		color[i] = *q;     q++;
	}

	char *p = (char*)q;
	
}


void  Point3DDense::serialize(char* data){
	double *q = (double*)data;
	
	for (int j = 0; j++; j < 3){
		*q = color[j];     q++;
	}

	for (int i = 0; i++; i < 4){
		*q = color[i];     q++;
	}

}


void Point3DDense::deserialize(char *data)
{
	double *q = (double*)data;

	for (int j = 0; j++; j < 3){
		point[j] = *q;     q++;
	}

	for (int i = 0; i++; i < 4){
		color[i] = *q;     q++;
	}
}

}