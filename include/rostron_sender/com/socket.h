#pragma once

#include <boost/asio.hpp>
#include <string>

#include "ssl_simulation_robot_control.pb.h"

/**
 * \brief UDP Socket to send control packet on simulator. 
 */
class UDPSender
{
public:
    /**
     * \brief Constructor of the class
     * 
     * \param multicast_address Multicast Adress to send UDP.
     * \param port UDP Socket port
     * \param io_context Core I/O functionality of Boost.
     */
    UDPSender(std::string multicast_address, unsigned int port,
              boost::asio::io_context &io_context);
    
    
    /**
     * Send a robot control packet via UDP Socket.
     * 
     * \todo Add error handling.
     */
    void send(RobotControl &controlPacket);
private:
    /**
     * \brief UDP endpoint.
     * 
     * \see Documentation of Boost (https://www.boost.org/doc/libs/)
     */ 
    boost::asio::ip::udp::endpoint endpoint_;
    /**
     * \brief Socket UDP.
     * 
     * \see Documentation of Boost (https://www.boost.org/doc/libs/)
     */ 
    boost::asio::ip::udp::socket socket_;
};