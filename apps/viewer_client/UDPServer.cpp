#include "UDPServer.h"


void addr::printAddrInfo(std::string msg){
	std::cout << msg << ip << ":" << port << " ..." << std::endl;
}

std::string make_daytime_string(){
	using namespace std; // For time_t, time and ctime;
	time_t now = time(0);
	return ctime(&now);
}

UDPServer::UDPServer(boost::asio::io_service& io_service) : socket_(io_service, udp::endpoint(boost::asio::ip::address_v4::loopback(), 14))
{
	addr local{ socket_.local_endpoint().address().to_string(), socket_.local_endpoint().port() };
	local.printAddrInfo("UDP Server is starting at:");

	start_receive();
}
void UDPServer::start_receive()
{
	socket_.async_receive_from(
		boost::asio::buffer(recv_buffer_), remote_endpoint_,
		boost::bind(&UDPServer::handle_receive, this,
		boost::asio::placeholders::error,
		boost::asio::placeholders::bytes_transferred));
	
	std::cout << recv_buffer_.data() << std::endl;
}

void UDPServer::handle_receive(const boost::system::error_code& error,
	std::size_t /*bytes_transferred*/)
{
	if (!error || error == boost::asio::error::message_size)
	{
		boost::shared_ptr<std::string> message(
			new std::string("received \n"));
		

		/*socket_.async_send_to(boost::asio::buffer(*message), remote_endpoint_,
			boost::bind(&UDPServer::handle_send, this, message,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred));*/

		start_receive();
	}
}

void UDPServer::handle_send(boost::shared_ptr<std::string> /*message*/,
	const boost::system::error_code& /*error*/,
	std::size_t /*bytes_transferred*/)
{
}
