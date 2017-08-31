#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>

#include "UDPClient.h"

using boost::asio::ip::udp;

void addr::printAddrInfo(std::string msg){
	std::cout << msg << ip << ":" << port << " ..." << std::endl;
}

UDPClient::UDPClient(boost::asio::io_service& io_service, unsigned short localport, const std::string& host, const std::string& port) :
io_service_(io_service), socket_(io_service, udp::endpoint(udp::v4(), localport)){
		
	addr local{ socket_.local_endpoint().address().to_string(), socket_.local_endpoint().port() };
	local.printAddrInfo("UDP Client is starting at: ");
	

	udp::resolver resolver(io_service_);
	udp::resolver::query query(udp::v4(), host, port);
	udp::resolver::iterator iter = resolver.resolve(query);
	endpoint_ = *iter;

	addr remote{ endpoint_.address().to_string(), endpoint_.port() };

	remote.printAddrInfo("With unbound endpoint: ");

}

UDPClient::~UDPClient()
{
	socket_.close();
	//WSACleanup;
}

void UDPClient::send(const std::string& msg) {

	int bt = socket_.send_to(boost::asio::buffer(msg, msg.size()), endpoint_);
	std::cout << "Bytes sent:" << bt << std::endl;

}
void UDPClient::send(std::vector<double> msg) {
	int bt = socket_.send_to(boost::asio::buffer((char*)&msg.front(), msg.size()*sizeof(double)), endpoint_);
	std::cout << "Bytes sent:" << bt << std::endl;

}

void UDPClient::send(std::vector<float> msg) {


	int bt = socket_.send_to(boost::asio::buffer((char*)&msg.front(), msg.size()*sizeof(float)), endpoint_);
	std::cout << "Bytes sent:"<< bt << std::endl;

}