// This module
#include "UDPServer.h"
/* 
UDPServer::UDPServer(boost::asio::io_context& io_context): 
    : _socket(io_context, udp::endpoint(udp::v4(), 1111))
{
	startReceive();
}

UDPServer::~UDPServer()
{
	_socket.close();
}

void UDPServer::startReceive() 
{
	_socket.async_receive_from(
		boost::asio::buffer(_recvBuffer), _remoteEndpoint,
		boost::bind(&UDPServer::handleReceive, this,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred));
}

void UDPServer::handleReceive(const boost::system::error_code& error,
                       std::size_t bytes_transferred) 
{
	if (!error || error == boost::asio::error::message_size) {

		auto message = std::make_shared<std::string>("Hello, World\n");

		_socket.async_send_to(boost::asio::buffer(*message), _remoteEndpoint,
			boost::bind(&UDPServer::handleSend, this, message,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
		startReceive();
	}
}

void UDPServer::handleSend(std::shared_ptr<std::string> message,
                    const boost::system::error_code& ec,
                    std::size_t bytes_transferred) 
{
} */