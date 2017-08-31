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

UDPServer::UDPServer(boost::asio::io_service& io_service, viewer_client::SLAMData* slam, unsigned short port, unsigned short sz) :
socket_(io_service, udp::endpoint(boost::asio::ip::address_v4::loopback(), port)),
size(sz),
float_buffer_(sz, 0),
slam(slam)
{
	addr local{ socket_.local_endpoint().address().to_string(), socket_.local_endpoint().port() };
	local.printAddrInfo("UDP Server is starting at:");


}

UDPServer::UDPServer(boost::asio::io_service& io_service, viewer_client::SLAMData* slam, unsigned short port, std::vector<float>* buffer) :
socket_(io_service, udp::endpoint(boost::asio::ip::address_v4::loopback(), port)),
slam(slam)
{
	float_buffer_ =*buffer;
	size = float_buffer_.size();
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
UDPServer(io_service, slam, PORT_CAM_POSE, 9),
double_buffer_(this->size, 0)
{
	
	start_receive();
}

keyFrameServer::keyFrameServer(boost::asio::io_service& io_service, viewer_client::SLAMData* slam) :
UDPServer(io_service, slam, PORT_KEYFRAME, 926103){
	start_receive();
}

keyFrameServer::keyFrameServer(boost::asio::io_service& io_service, viewer_client::SLAMData* slam, std::vector<float>* buffer) :
UDPServer(io_service, slam, PORT_KEYFRAME, buffer){
	start_receive();
}

void UDPServer::start_receive()
{
	//std::cout << "recieving " << std::endl;

	socket_.async_receive_from(
		boost::asio::buffer((char*)&float_buffer_.front(), size*sizeof(float)), remote_endpoint_,
		boost::bind(&UDPServer::handle_receive, this,
		boost::asio::placeholders::error,
		boost::asio::placeholders::bytes_transferred));


}
void  camPoseServer::start_receive(){
	//std::cout << "recieving " << std::endl;

	socket_.async_receive_from(
		boost::asio::buffer((char*)&double_buffer_.front(), size*sizeof(double)), remote_endpoint_,
		boost::bind(&camPoseServer::handle_receive, this,
		boost::asio::placeholders::error,
		boost::asio::placeholders::bytes_transferred));


}

void  camParamsServer::handle_receive(const boost::system::error_code& error,
	std::size_t bytes_transferred){
	//std::cout << "you invoked camParam's handle: " << std::endl;
	if (!error || error == boost::asio::error::message_size){
		this->slam->camParamsHandle(this->float_buffer_);
		//std::cout << "Camera: " << this->slam->getCamera().fx << std::endl;
		std::cout << "Camera Params Received " << std::endl;
	}
	else
		start_receive();
	
}

void  camPoseServer::handle_receive(const boost::system::error_code& error,
	std::size_t bytes_transferred){
	if (!error || error == boost::asio::error::message_size){
		this->slam->camPoseHandle(this->double_buffer_);
	//std::cout << "Cam Pose: " << this->slam->get_lastCamPose().header.stamp.sec << std::endl;
	}
	start_receive();
}

void keyFrameServer::handle_receive(const boost::system::error_code& error,
	std::size_t bytes_transferred){
	if (!error || error == boost::asio::error::message_size && float_buffer_.size()>10){
		std::cout << "Keyframe Received " << std::endl;
		this->slam->keyFramHandle(this->float_buffer_);
	}
	else { //autoresize
		if (slam->pixsize != 1){
			std::cout << "resizing " << std::endl;
			setSize(slam->pixsize * 3 + 3);
			float_buffer_.resize(size, 0);
			std::cout << "PointCloud* buffer size: " << float_buffer_.size() << std::endl;
			//*rgdb pixles with depth actually
			}
	}
	start_receive();

}
