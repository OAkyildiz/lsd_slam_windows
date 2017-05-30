/*
 *   C++ sockets on Unix and Windows
 *   Copyright (C) 2002
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <iostream>           // For cout and cerr
#include <cstdlib>            // For atoi()
//#include "pcl/visualization/cloud_viewer.h"

#ifdef WIN32
#include <windows.h>          // For ::Sleep()
void sleep(unsigned int seconds) {::Sleep(seconds * 1000);}

#define WINVER				0x0A00  
#define _WIN32_WINNT		0x0A00  
#define _WIN32_WINNT_WIN10	0x0A00 // Windows 10  

#else
#include <unistd.h>           // For sleep()
#endif

#include "UDPServer.h"

using namespace std;

int main(int argc, char* argv[])
{
	try
	{
		/*if (argc != 2){
			std::cerr << "Usage: client <host>" << std::endl;
			return 1;
		}
		else*/
		boost::asio::io_service io_service;
		UDPServer server(io_service);
		io_service.run();
		//pcl::visualization::CloudViewer viewer("Cloud Viewer");
		std::cout << "IO Service is runninng" << std::endl;
	}
	catch (std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}

	return 0;
}
