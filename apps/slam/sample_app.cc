#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "live_slam_wrapper.h"

#include "util/settings.h"
#include "util/global_funcs.h"

#include "util/Undistorter.h"
#include "io_wrapper/OpenCVImageStreamThread.h"
#include "slam_system.h"
#include "DebugOutput3DWrapper.h"

using namespace std;
using namespace lsd_slam;
char key;
//PATH=C:\projects\uni\dissertation\Libraries\g2o\install\bin;C:\projects\uni\dissertation\Libraries\opencv\x86\vc12\bin;%PATH%
int main() {
	cvNamedWindow("Camera_Output_Undist", 1); //Create window

    //TODO: dynamic path, instead of constant
    std::string calib_fn = "E:/workspaces/lsd_slam_windows/data/out_camera_data.xml";
	//CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY); //Capture using any camera connected to your system
	cv::VideoCapture* capture= new cv::VideoCapture(0); // open the default camera
	if (!capture->isOpened())  // check if we succeeded
	  return -1;
	OpenCVImageStreamThread* inputStream = new OpenCVImageStreamThread();
    inputStream->setCalibration(calib_fn);

	inputStream->setCameraCapture(capture);
	inputStream->run();

	Output3DWrapper* outputWrapper = new DebugOutput3DWrapper(inputStream->width(), inputStream->height());
	LiveSLAMWrapper slamNode(inputStream, outputWrapper);
	
	cv::Mat mymat;
	*capture >> mymat;//Create image frames from capture
	cv::Mat tracker_display = cv::Mat::ones(640, 480, CV_8UC3);
	cv::circle(mymat, cv::Point(100, 100), 20, cv::Scalar(255, 1, 0),5);
	cv::imshow("Camera_Output_Undist", mymat);

	slamNode.Loop();

	//Undistorter* undistorter = Undistorter::getUndistorterForFile("out_camera_data.xml");

	//while (1){ //Create infinte loop for live streaming
	//	IplImage* frame = cvQueryFrame(capture); //Create image frames from capture
	//	TimestampedMat bufferItem;
	//	bufferItem.timestamp = Timestamp::now();
	//	
	//	cv::Mat mymat = cv::Mat(frame, true);

	//	
	//	undistorter->undistort(frame, mymat);
	//    
	//	cvShowImage("Camera_Output", frame); //Show image frames on created window
	//	cv::imshow("Camera_Output_Undist", mymat);
	//	key = cvWaitKey(0); //Capture Keyboard stroke
	//	if (char(key) == 27){
	//		break; //If you hit ESC key loop will break.
	//	}
	//}

	if (inputStream != nullptr)
		delete inputStream;
	if (outputWrapper != nullptr)
		delete outputWrapper;

	//cvReleaseCapture(&capture);  the camera will be deinitialized automatically in VideoCapture destructor
	cvDestroyWindow("Camera_Output"); //Destroy Window
	return 0;
}