
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
#include "SLAMOutputWrapper.h"

#include <ctime>
#include <iostream>
#include <string>

#include <errno.h>   // for errno
#include <limits.h>  // for INT_MAX
#include <stdlib.h>  // for strtol

#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>


#define _WIN32_WINNT		0x0A00  
#define _WIN32_WINNT_WIN10	0x0A00 // Windows 10  

//#include "ros_lib\geometry_msgs\Point.h"

using boost::asio::ip::udp;
using namespace std;
using namespace lsd_slam;

char key;

//PATH=C:\projects\uni\dissertation\Libraries\g2o\install\bin;C:\projects\uni\dissertation\Libraries\opencv\x86\vc12\bin;%PATH%
int main(int argc, char* argv[]){

	cvNamedWindow("Camera_Output_Undist", 1); //Create window
	//int s = socket(AF_UNIX, type, protocol);
	//if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
	//	error("cannot create socket");
	//}
	//TODO: dynamic path, instead of constant
	std::string calib_fn = "E:/workspaces/lsd_slam_windows/data/out_camera_data.xml";
	//CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY); //Capture using any camera connected to your system
	int pt_qt = 0;
	char *p;

	cv::VideoCapture* capture;
	if (argc > 1){
		errno = 0;
		long conv = strtol(argv[1], &p, 10);

		// Check for errors: e.g., the string does not represent an integer
		// or the integer is larger than int
		if (errno != 0 || *p != '\0' || conv > INT_MAX) {
			std::cout << "Couldn't read the arg " << argv[1] << std::endl;

		}
		else {
			// No error
			pt_qt = conv;
		}
	}
	if (argc > 2){
		//if (strcmp(argv[1] ,'0') || argv[1] == '1')
		//capture = new cv::VideoCapture(argv[1]);
		std::cout << "Source: " << argv[2] << std::endl;
	}
	else
		capture = new cv::VideoCapture(1); // open the default camera

	std::cout << "Point Quota: " << pt_qt << std::endl;
	
	if (!capture->isOpened())  // check if we succeeded
	  return -1;

	

	OpenCVImageStreamThread* inputStream = new OpenCVImageStreamThread();
    inputStream->setCalibration(calib_fn);

	inputStream->setCameraCapture(capture);
	inputStream->run();

	Output3DWrapper* outputWrapper = new SLAMOutputWrapper(inputStream->width(), inputStream->height(), pt_qt);
	LiveSLAMWrapper slamNode(inputStream, outputWrapper);

	cv::Mat mymat;
	*capture >> mymat;//Create image frames from capture
	cv::Mat tracker_display = cv::Mat::ones(640, 480, CV_8UC3);
	cv::circle(mymat, cv::Point(100, 100), 20, cv::Scalar(255, 1, 0),5);
	cv::imshow("Camera_Output_Undist", mymat);

	slamNode.Loop();

	if (inputStream != nullptr)
		delete inputStream;
	if (outputWrapper != nullptr)
		delete outputWrapper;

	//cvReleaseCapture(&capture);  the camera will be deinitialized automatically in VideoCapture destructor
	cvDestroyAllWindows(); //Destroy Window

	
	return 0;
}
