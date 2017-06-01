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
	UDPServer(boost::asio::io_service& io_service, viewer_client::SLAMData* slam, unsigned short port, unsigned short size);
	virtual ~UDPServer(){}

protected:
	
	unsigned short size;
	udp::socket socket_;
	udp::endpoint remote_endpoint_;
	
	std::vector<double> data_buffer_;
	
	viewer_client::SLAMData* slam;

	void start_receive();
	virtual void handle_receive(const boost::system::error_code& error,
		std::size_t bytes_transferred){}
private:

	//void assign_callback(void(*fn)(std::vector<double>));
	//void assign_callback();

	//void (*callback)(std::vector<double>);
	//void(UDPServer::*callback)(const boost::system::error_code&,std::size_t);
	//boost::array<char, 1024> recv_buffer_;
	


};
// For reasons of precticality, instead of using reference by func. poitners for handlers we are using polymorphism
class camParamsServer : public UDPServer{

public:
	camParamsServer(boost::asio::io_service& io_service, viewer_client::SLAMData* slam);
	virtual ~camParamsServer(){}

	//@override
protected:
	void handle_receive(const boost::system::error_code& error,
		std::size_t bytes_transferred);
	//void start_receive();

};
class camPoseServer : public UDPServer{

public:
	camPoseServer(boost::asio::io_service& io_service, viewer_client::SLAMData* slam);
	virtual ~camPoseServer(){}
	
	//@override
	void handle_receive(const boost::system::error_code& error,
		std::size_t bytes_transferred);
//	void start_receive();
};
class keyFrameServer : public UDPServer{

public:
	keyFrameServer(boost::asio::io_service& io_service, viewer_client::SLAMData* slam);
	virtual ~keyFrameServer(){}

	//@override
	void handle_receive(const boost::system::error_code& error,
		std::size_t bytes_transferred);
};