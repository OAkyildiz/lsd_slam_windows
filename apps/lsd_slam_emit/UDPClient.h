#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>

#define PORT_KEYFRAME 12001
#define PORT_CAM_PARAMS 12002
#define PORT_CAM_POSE 12003

#define LOCAL_PORT_KEYFRAME 11997
#define LOCAL_PORT_CAM_PARAMS 11998
#define LOCAL_PORT_CAM_POSE 11999

using boost::asio::ip::udp;
struct addr{
	std::string ip;
	unsigned int port;

	void printAddrInfo(std::string msg);
};
class UDPClient
{
public:

	UDPClient(boost::asio::io_service& io_service, unsigned short localport, const std::string& host, const std::string& port);

	~UDPClient();

	void UDPClient::send(const std::string& msg);
	void UDPClient::send(std::vector<double> msg);

private:
	boost::asio::io_service& io_service_;
	udp::socket socket_;
	udp::endpoint endpoint_;
};
