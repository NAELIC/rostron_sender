#include "rostron_sender/com/socket.h"

using namespace boost;

UDPSender::UDPSender(std::string multicast_address, unsigned int port,
                     asio::io_context &io_context)
    : endpoint_(asio::ip::make_address(multicast_address), port),
      socket_(io_context, endpoint_.protocol())
{
    socket_.set_option(asio::ip::multicast::hops(1));
}

void UDPSender::send(RobotControl &controlPacket)
{
    system::error_code ignored_error;
    boost::asio::streambuf b;
    std::ostream os(&b);
    controlPacket.SerializeToOstream(&os);
    socket_.send_to(b.data(), endpoint_, 0, ignored_error);
}