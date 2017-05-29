#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::udp;
struct addr{
	std::string ip;
	unsigned int port;

	void printAddrInfo(std::string msg);
};
class UDPClient
{
public:

	UDPClient(boost::asio::io_service& io_service, const std::string& host, const std::string& port);

	~UDPClient();
	
	void send(const std::string& msg);

private:
	boost::asio::io_service& io_service_;
	udp::socket socket_;
	udp::endpoint endpoint_;
};
