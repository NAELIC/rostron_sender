#include <cstdio>
#include "ssl_simulation_robot_control.pb.h"

#include "rclcpp/rclcpp.hpp"
#include "rostron_interfaces/msg/order.hpp"

#include "rostron_sender/net/udp_sender.h"

#include <boost/asio.hpp>

using std::placeholders::_1;

// TODO :
// - Nettoyer le code
// - Mettre le control du robot en fonction d'un paramètres.

boost::asio::io_context io;

class SimSender : public rclcpp::Node
{
public:
  SimSender()
      : Node("sim_sender"), sender_("0.0.0.0", 10301, io)
  {
    this->declare_parameter<int>("control_port", 10301);
    this->sender_ = UDPSender("0.0.0.0", this->get_parameter("control_port").as_int(), io);
    subscription_ = this->create_subscription<rostron_interfaces::msg::Order>(
        "order", 10, std::bind(&SimSender::topic_callback, this, _1));
  }

private:
  void topic_callback(const rostron_interfaces::msg::Order::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->id);
    auto control = RobotControl();
    auto command = control.add_robot_commands();

    // Todo : Check if message is valid.
    auto mv_command = command->mutable_move_command();
    auto local = mv_command->mutable_local_velocity();
    local->set_angular(msg->velocity.angular.z);
    local->set_forward(msg->velocity.linear.x);
    local->set_left(msg->velocity.linear.y);

    command->set_id(msg->id);
    // TODO : Add kick && dribbler

    sender_.send(control);
  }

  rclcpp::Subscription<rostron_interfaces::msg::Order>::SharedPtr subscription_;
  UDPSender sender_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimSender>());
  rclcpp::shutdown();
  return 0;
}
