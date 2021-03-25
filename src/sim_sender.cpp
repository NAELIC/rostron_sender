#include <cstdio>
#include "ssl_simulation_robot_control.pb.h"

#include "rclcpp/rclcpp.hpp"
#include "rostron_interfaces/msg/order.hpp"

#include "rostron_sender/net/udp_sender.h"

#include <boost/asio.hpp>

using std::placeholders::_1;

boost::asio::io_context io;

const std::string port_params = "port";

class SimSender : public rclcpp::Node
{
public:
  SimSender()
      : Node("sim_sender"), sender_("0.0.0.0", 10301, io)
  {
    this->declare_parameter<int>(port_params, 10301);
    this->sender_ = UDPSender("0.0.0.0", this->get_parameter(port_params).as_int(), io);
    subscription_ = this->create_subscription<rostron_interfaces::msg::Order>(
        "order", 10, std::bind(&SimSender::topic_callback, this, _1));
  }

private:
  // TODO (#1) : Check parameters
  // TODO (#2) : Donner la possibilité d'envoyer plusieurs ordres à des robots !
  void topic_callback(const rostron_interfaces::msg::Order::SharedPtr msg)
  {
    const auto max_ball_speed = 6.5;

    auto control = RobotControl();
    auto command = control.add_robot_commands();
    command->set_id(msg->id);

    // Speed
    auto mv_command = command->mutable_move_command();
    auto local = mv_command->mutable_local_velocity();
    local->set_angular(msg->velocity.angular.z);
    local->set_forward(msg->velocity.linear.x);
    local->set_left(msg->velocity.linear.y);
    
    // Kicker, Dribbler
    command->set_dribbler_speed(msg->hardware.spin_power);

    if (msg->hardware.kick_type == msg->hardware.FLAT_KICK)
    {
      command->set_kick_angle(0);
      command->set_kick_speed(max_ball_speed * msg->hardware.kick_power);
    }
    else if (msg->hardware.kick_type == msg->hardware.CHIP_KICK)
    {
      command->set_kick_angle(45);
      command->set_kick_speed(max_ball_speed * msg->hardware.kick_power);
    }

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
