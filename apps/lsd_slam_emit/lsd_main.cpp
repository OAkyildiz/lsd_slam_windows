#include <iostream>
//#include <sys/socket.h>


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
	cv::VideoCapture* capture= new cv::VideoCapture(0); // open the default camera
	if (!capture->isOpened())  // check if we succeeded
	  return -1;
	OpenCVImageStreamThread* inputStream = new OpenCVImageStreamThread();
    inputStream->setCalibration(calib_fn);

	inputStream->setCameraCapture(capture);
	inputStream->run();

	Output3DWrapper* outputWrapper = new SLAMOutputWrapper(inputStream->width(), inputStream->height());
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
