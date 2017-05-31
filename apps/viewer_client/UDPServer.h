#pragma once

#include <ctime>
#include <iostream>
#include <string>
#include <stdio.h>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include "boost\asio\io_service.hpp"  // For UDPSocket and SocketException

#include "SLAMData.h"

//TODO: Put common constants into one parent include file

#define PORT_KEYFRAME 12001
#define PORT_CAM_PARAMS 12002
#define PORT_CAM_POSE 12003

using boost::asio::ip::udp;

struct addr{
	std::string ip;
	unsigned int port;

	void printAddrInfo(std::string msg);

};

std::string make_daytime_string();
class UDPServer
{
public:
	UDPServer(boost::asio::io_service& io_service, unsigned short port, unsigned short size, void(*fn)(std::vector<double>));
	virtual ~UDPServer(){}
private:
	void start_receive();
	void handle_receive(const boost::system::error_code& error,
		std::size_t /*bytes_transferred*/);

	void assign_callback(void(*fn)(std::vector<double>));
	unsigned short size;
	udp::socket socket_;
	udp::endpoint remote_endpoint_;

	void (*callback)(std::vector<double>);
	viewer_client::SLAMData* slam;
	boost::array<char, 1024> recv_buffer_;
	std::vector<double> data_buffer_;
};

