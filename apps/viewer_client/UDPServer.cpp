#include "UDPServer.h"
#include "SLAMData.h"

void addr::printAddrInfo(std::string msg){
	std::cout << msg << ip << ":" << port << " ..." << std::endl;
}

std::string make_daytime_string(){
	using namespace std; // For time_t, time and ctime;
	time_t now = time(0);
	return ctime(&now);
}

UDPServer::UDPServer(boost::asio::io_service& io_service, unsigned short port, unsigned short size, void(*fn)(std::vector<double>)) :
socket_(io_service, udp::endpoint(boost::asio::ip::address_v4::loopback(), port)),
data_buffer_(size,0)
{
	addr local{ socket_.local_endpoint().address().to_string(), socket_.local_endpoint().port() };
	local.printAddrInfo("UDP Server is starting at:");
	assign_callback(fn);
	start_receive();
}
void UDPServer::start_receive()
{
	socket_.async_receive_from(
		boost::asio::buffer((char*)&data_buffer_.front(), size*sizeof(double)), remote_endpoint_,
		boost::bind(&UDPServer::handle_receive, this,
		boost::asio::placeholders::error,
		boost::asio::placeholders::bytes_transferred));
	
	
}

void UDPServer::handle_receive(const boost::system::error_code& error,
	std::size_t bytes_transferred)
{
	if (!error || error == boost::asio::error::message_size)
	{
		
		//doHandle
		this->callback(data_buffer_);
		start_receive();
	}
}

void UDPServer::assign_callback(void(*fn)(std::vector<double>)){
	this->callback = fn;

}
