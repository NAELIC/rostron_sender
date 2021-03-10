#pragma once

#include <boost/asio.hpp>
#include <string>

#include "ssl_simulation_robot_control.pb.h"

class UDPSender
{
public:
    UDPSender(std::string multicast_address, unsigned int port,
              boost::asio::io_context &io_context);
    void send(RobotControl &controlPacket);
private:
    boost::asio::ip::udp::endpoint endpoint_;
    boost::asio::ip::udp::socket socket_;
};