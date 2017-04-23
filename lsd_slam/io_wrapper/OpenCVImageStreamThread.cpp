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

#include "OpenCVImageStreamThread.h"
//#include <ros/callback_queue.h>

#include <boost/thread.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include "cv_bridge/cv_bridge.h"
#include "util/settings.h"

#include <iostream>
#include <fstream>
#include <ctime>

namespace lsd_slam
{


using namespace cv;

OpenCVImageStreamThread::OpenCVImageStreamThread()
{
	// subscribe
	//vid_channel = nh_.resolveName("image");
	//vid_sub = nh_.subscribe(vid_channel, 1, &OpenCVImageStreamThread::vidCb, this);


	// wait for cam calib
	width_ = height_ = 0;

	// imagebuffer
	imageBuffer = new NotifyBuffer<TimestampedMat>(8);
	undistorter = 0;
	lastSEQ = 0;

	haveCalib = false;
	undistorter = 0;

}

OpenCVImageStreamThread::~OpenCVImageStreamThread()
{
	delete imageBuffer;
}

void OpenCVImageStreamThread::setCameraCapture(cv::VideoCapture* cap)
{
	capture = cap;
}

void OpenCVImageStreamThread::setCalibration(std::string file)
{
	if(file == "")
	{
		printf("NO camera calibration file!\n");
	}
	else
	{
		undistorter = Undistorter::getUndistorterForFile(file.c_str());

		if(!undistorter)
		{
			printf("Failed to read camera calibration from file... wrong syntax?\n");
			assert("Failed to read camera calibration from file... wrong syntax?");
		}

		fx_ = undistorter->getK().at<double>(0, 0);
		fy_ = undistorter->getK().at<double>(1, 1);
		cx_ = undistorter->getK().at<double>(2, 0);
		cy_ = undistorter->getK().at<double>(2, 1);

		width_ = undistorter->getOutputWidth();
		height_ = undistorter->getOutputHeight();
	}

	haveCalib = true;
}

void OpenCVImageStreamThread::run()
{
	boost::thread thread(boost::ref(*this));
}

void OpenCVImageStreamThread::operator()()
{
	//ros::spin();
	if (!haveCalib)
	{
		assert("no calibration");
		return;
	}
	if (!capture->isOpened())
	{
		assert("NO valid camera capture pointer");
		return;
	}
	while (1)
	{
		TimestampedMat bufferItem;
		bufferItem.timestamp = Timestamp::now();
		cv::Mat mt;
		*capture >> mt; //get image
		if (undistorter != 0)
		{
			assert(undistorter->isValid());
			undistorter->undistort(mt, bufferItem.data);
		}
		else
		{
			bufferItem.data = mt;
		}
		//bufferItem.data = cv::Mat(frame, true);
		imageBuffer->pushBack(bufferItem);
	}
	exit(0);
}


//void OpenCVImageStreamThread::vidCb(const sensor_msgs::ImageConstPtr img)
//{
//	if(!haveCalib) return;
//
//	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
//
//	if(img->header.seq < (unsigned int)lastSEQ)
//	{
//		printf("Backward-Jump in SEQ detected, but ignoring for now.\n");
//		lastSEQ = 0;
//		return;
//	}
//	lastSEQ = img->header.seq;
//
//	TimestampedMat bufferItem;
//	if(img->header.stamp.toSec() != 0)
//		bufferItem.timestamp =  Timestamp(img->header.stamp.toSec());
//	else
//		bufferItem.timestamp =  Timestamp(ros::Time::now().toSec());
//
//	if(undistorter != 0)
//	{
//		assert(undistorter->isValid());
//		undistorter->undistort(cv_ptr->image,bufferItem.data);
//	}
//	else
//	{
//		bufferItem.data = cv_ptr->image;
//	}
//
//	imageBuffer->pushBack(bufferItem);
//}
//
//void OpenCVImageStreamThread::infoCb(const sensor_msgs::CameraInfoConstPtr info)
//{
//	if(!haveCalib)
//	{
//		fx_ = info->P[0];
//		fy_ = info->P[5];
//		cx_ = info->P[2];
//		cy_ = info->P[6];
//
//		if(fx_ == 0 || fy_==0)
//		{
//			printf("camera calib from P seems wrong, trying calib from K\n");
//			fx_ = info->K[0];
//			fy_ = info->K[4];
//			cx_ = info->K[2];
//			cy_ = info->K[5];
//		}
//
//		width_ = info->width;
//		height_ = info->height;
//
//		printf("Received ROS Camera Calibration: fx: %f, fy: %f, cx: %f, cy: %f @ %dx%d\n",fx_,fy_,cx_,cy_,width_,height_);
//	}
//}

}
