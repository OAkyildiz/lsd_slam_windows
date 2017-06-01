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

UDPServer::UDPServer(boost::asio::io_service& io_service, viewer_client::SLAMData* slam, unsigned short port, unsigned short size) :
socket_(io_service, udp::endpoint(boost::asio::ip::address_v4::loopback(), port)),
data_buffer_(size,0),
slam(slam)
{
	addr local{ socket_.local_endpoint().address().to_string(), socket_.local_endpoint().port() };
	local.printAddrInfo("UDP Server is starting at:");


}


/*void UDPServer::handle_receive(const boost::system::error_code& error,
	std::size_t bytes_transferred){
	if (!error || error == boost::asio::error::message_size)
	{
		
		//doHandle
		//this->callback(data_buffer_);
		start_receive();start_receive
	}
}*/

//void UDPServer::assign_callback(void(*fn)(std::vector<double>)){
/*void UDPServer::assign_callback(){
	if (this->mode == 0)
		this->callback = &UDPServer::camParamsHandle;
	else if (mode == 1)
		this->callback = &UDPServer::camPoseHandle;
	else if (mode == 2)
		this->callback = &UDPServer::keyFramHandle;;

	// DEPRECATED (for now)
}*/



camParamsServer::camParamsServer(boost::asio::io_service& io_service, viewer_client::SLAMData* slam) : 
UDPServer(io_service, slam, PORT_CAM_PARAMS, 6){
	start_receive();
}

camPoseServer::camPoseServer(boost::asio::io_service& io_service, viewer_client::SLAMData* slam) : 
UDPServer(io_service, slam, PORT_CAM_POSE, 9){
	start_receive();
}

keyFrameServer::keyFrameServer(boost::asio::io_service& io_service, viewer_client::SLAMData* slam) :
UDPServer(io_service, slam, PORT_KEYFRAME, 7){
	start_receive();
}

void UDPServer::start_receive()
{
	//std::cout << "recieving " << std::endl;

	socket_.async_receive_from(
		boost::asio::buffer((char*)&data_buffer_.front(), size*sizeof(double)), remote_endpoint_,
		boost::bind(&UDPServer::handle_receive, this,
		boost::asio::placeholders::error,
		boost::asio::placeholders::bytes_transferred));


}

void  camParamsServer::handle_receive(const boost::system::error_code& error,
	std::size_t bytes_transferred){
	//std::cout << "you invoked camParam's handle: " << std::endl;
	if (!error || error == boost::asio::error::message_size){
		this->slam->camParamsHandle(this->data_buffer_);
		//std::cout << "Camera: " << this->slam->getCamera().fx << std::endl;
		std::cout << "Camera Params Received " << std::endl;
	}
	else
		start_receive();
	
}

void  camPoseServer::handle_receive(const boost::system::error_code& error,
	std::size_t bytes_transferred){
	if (!error || error == boost::asio::error::message_size){
	this->slam->camPoseHandle(this->data_buffer_);
	//std::cout << "Cam Pose: " << this->slam->get_lastCamPose().header.stamp.sec << std::endl;

	}
	start_receive();
}

void keyFrameServer::handle_receive(const boost::system::error_code& error,
	std::size_t bytes_transferred){
	if (!error || error == boost::asio::error::message_size){
		this->slam->keyFramHandle(this->data_buffer_);
	}
	start_receive();

}
