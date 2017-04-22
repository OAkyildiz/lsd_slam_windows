// TestVideoInput.cpp: определяет точку входа для консольного приложения.
//

#include "stdafx.h"
#include "../videoInput/videoInput.h"
#include "include\opencv2\highgui\highgui_c.h"
#include "include\opencv2\imgproc\imgproc_c.h"


#pragma comment(lib, "../Debug/videoInput.lib")
#pragma comment(lib, "lib/opencv_highgui248d.lib")
#pragma comment(lib, "lib/opencv_core248d.lib")





int _tmain(int argc, _TCHAR* argv[])
{
	using namespace std;		
	
	vector<Device> listOfDevices;

	ResultCode::Result result = videoInput::getInstance().getListOfDevices(listOfDevices);


	
	DeviceSettings deviceSettings;

	deviceSettings.symbolicLink = listOfDevices[0].symbolicName;

	deviceSettings.indexStream = 0;

	deviceSettings.indexMediaType = 0;


			

	CaptureSettings captureSettings;
	
	captureSettings.pIStopCallback = 0;

	captureSettings.readMode = ReadMode::SYNC;

	captureSettings.videoFormat = CaptureVideoFormat::RGB32;


	MediaType MT = listOfDevices[0].listStream[0].listMediaType[0];

    cvNamedWindow ("VideoTest", CV_WINDOW_AUTOSIZE);
                
	CvSize size = cvSize(MT.width, MT.height);
 

    IplImage* frame;
 
    frame = cvCreateImage(size, 8,4);

	
	
	ReadSetting readSetting;

	readSetting.symbolicLink = deviceSettings.symbolicLink;

	readSetting.pPixels = (unsigned char *)frame->imageData;

	result = videoInput::getInstance().setupDevice(deviceSettings, captureSettings);

	while(1)
	{
		ResultCode::Result readState = videoInput::getInstance().readPixels(readSetting);

		if(readState == ResultCode::READINGPIXELS_DONE)
		{			
			cvShowImage("VideoTest", frame);
		}
		else
			break;
					
        char c = cvWaitKey(33);
 
        if(c == 27) 
            break;
	}

	result = videoInput::getInstance().closeDevice(deviceSettings);
	

	return 0;
}

