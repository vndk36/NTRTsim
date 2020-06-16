#ifndef UDPServer_H
#define UDPServer_H

#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

using boost::asio::ip::udp;

/* class UDPServer 
{
public:

    UDPServer(boost::asio::io_service& io_service);

	~UDPServer();

private:
    void handleReceive(const boost::system::error_code& error,
                       std::size_t bytes_transferred);

    void handleSend(std::shared_ptr<std::string> message,
                    const boost::system::error_code& ec,
                    std::size_t bytes_transferred);

    udp::socket _socket;
    udp::endpoint _remoteEndpoint;
    std::array<char, 1024> _recvBuffer;
};

/* 
int main() {
    try {
        boost::asio::io_service io_service;
        UDPServer server{io_service};
        io_service.run();
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
    }
    return 0;
} */

/*
#endif // UDPServer_H
 */

/* int main()
{
  try
  {
    boost::asio::io_service io_service;
    UDPServer server(io_service);
    io_service.run();
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
 */
class UDPServer
{
public:
  UDPServer(boost::asio::io_service& io_service)
    : socket_(io_service, udp::endpoint(udp::v4(), 13))
  {
    start_receive();
  }

private:
  void start_receive()
  {
    socket_.async_receive_from(
        boost::asio::buffer(recv_buffer_), remote_endpoint_,
        boost::bind(&UDPServer::handle_receive, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
  }
  void handle_receive(const boost::system::error_code& error,
      std::size_t /*bytes_transferred*/)
  {
    if (!error)
    {
      boost::shared_ptr<std::string> message(
          new std::string("Hello, World\n"));
      
      socket_.async_send_to(boost::asio::buffer(*message), remote_endpoint_,
          boost::bind(&UDPServer::handle_send, this, message,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
      start_receive();
    }
  }
  void handle_send(boost::shared_ptr<std::string> /*message*/,
      const boost::system::error_code& /*error*/,
      std::size_t /*bytes_transferred*/)
  {
  }

  udp::socket socket_;
  udp::endpoint remote_endpoint_;
  boost::array<char, 1> recv_buffer_;
};

#endif // UDPServer_H