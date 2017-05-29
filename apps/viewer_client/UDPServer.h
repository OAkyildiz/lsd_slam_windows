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
	UDPServer(boost::asio::io_service& io_service);
	virtual ~UDPServer(){}
private:
	void start_receive();
	void handle_receive(const boost::system::error_code& error,
		std::size_t /*bytes_transferred*/);

	void handle_send(boost::shared_ptr<std::string> /*message*/,
		const boost::system::error_code& /*error*/,
		std::size_t /*bytes_transferred*/);

	udp::socket socket_;
	udp::endpoint remote_endpoint_;
	boost::array<char, 1024> recv_buffer_;
};

