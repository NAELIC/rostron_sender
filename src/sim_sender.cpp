#include <boost/asio.hpp>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "rostron_interfaces/msg/order.hpp"
#include "rostron_interfaces/msg/orders.hpp"
#include "rostron_sender/net/udp_sender.h"
#include "ssl_simulation_robot_control.pb.h"

using std::placeholders::_1;

boost::asio::io_context io;

/**
 * ROS Node to send robot control packet of a simulation.
 */
class SimSender : public rclcpp::Node
{
public:
  /**
   * Constructor.
   *
   * Create a UDP Server and two ROS subscribtion topic :
   * - order : send order for one robot.
   * - orders : send order for multiple robots.
   */
  SimSender() : Node("sim_sender"), sender_(create_socket()), max_ball_speed(6.5)
  {
    controls_subscription_ = this->create_subscription<rostron_interfaces::msg::Orders>(
        "orders", 10, std::bind(&SimSender::controls_callback, this, _1));
  }

  /**
   * Create a UDP Server Socket with ROS params.
   */
  UDPSender create_socket()
  {
    const auto port_params = "port";
    const auto multicast_params = "multicast_address";

    this->declare_parameter<int>(port_params, 10301);
    int port = this->get_parameter(port_params).as_int();

    this->declare_parameter<std::string>(multicast_params, "0.0.0.0");
    std::string multicast_address = this->get_parameter(multicast_params).as_string();

    RCLCPP_INFO(get_logger(), "Creating UDP Socket on %d...", port);

    return UDPSender(multicast_address, port, io);
  }

private:
  /**
   * \brief Callback to send control for multiple robots.
   */
  void controls_callback(const rostron_interfaces::msg::Orders::SharedPtr msg_orders)
  {
    auto control = RobotControl();
    RCLCPP_DEBUG(get_logger(), "Send control...");

    for (auto msg : msg_orders->orders)
    {

      auto command = control.add_robot_commands();
      command->set_id(msg.id);
      RCLCPP_DEBUG(get_logger(), "--- id : %d ", msg.id);

      // Speed
      auto mv_command = command->mutable_move_command();
      auto local = mv_command->mutable_local_velocity();
      local->set_forward(msg.velocity.linear.x);
      local->set_left(msg.velocity.linear.y);
      local->set_angular(msg.velocity.angular.z);

      RCLCPP_DEBUG(get_logger(), "------ forward (x) : %.2f ", msg.velocity.linear.x);
      RCLCPP_DEBUG(get_logger(), "------ left (y) : %.2f", msg.velocity.linear.y);
      RCLCPP_DEBUG(get_logger(), "------ angular (z) : %.2f", msg.velocity.angular.z);

      // Kicker, Dribbler
      command->set_dribbler_speed(msg.hardware.spin_power);

      if (msg.hardware.kick_type == msg.hardware.FLAT_KICK)
      {
        RCLCPP_DEBUG(get_logger(), "--- kick : FLAT");

        command->set_kick_angle(0);
        command->set_kick_speed(max_ball_speed * msg.hardware.kick_power);
      }
      else if (msg.hardware.kick_type == msg.hardware.CHIP_KICK)
      {
        RCLCPP_DEBUG(get_logger(), "--- kick : CHIP");
        command->set_kick_angle(45);
        command->set_kick_speed(max_ball_speed * msg.hardware.kick_power);
      }
    }
    sender_.send(control);
    RCLCPP_DEBUG(get_logger(), "End sending control...\n");
  }

  /**
   * \brief Subscribtion for "orders" topic.
   */
  rclcpp::Subscription<rostron_interfaces::msg::Orders>::SharedPtr controls_subscription_;

  /**
   * \brief UDP Sender socket server.
   * 
   * Wrapper function to handler an UDP socket to send commands of the simulator.
   */
  UDPSender sender_;

  /**
   * \brief Constant about the maximum ball speed of the ball.
   */
  float max_ball_speed;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimSender>());
  rclcpp::shutdown();
  return 0;
}
