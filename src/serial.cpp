#include <rclcpp/rclcpp.hpp>
#include <rostron_interfaces/msg/orders.hpp>
#include "serial/serial.h"
#include <iostream>

#define ACTION_ON (1 << 0)
#define ACTION_KICK1 (1 << 1)
#define ACTION_KICK2 (1 << 2)
#define ACTION_DRIBBLE (1 << 3)
#define ACTION_CHARGE (1 << 5)
#define ACTION_TARE_ODOM (1 << 7)

using std::placeholders::_1;

typedef struct
{
  uint8_t id;
  uint8_t actions;

  float x_speed; // Kinematic orders [mm/s]
  float y_speed;
  float t_speed; // Rotation in [mrad/s]

  uint8_t kickPower; // Kick power (this is a duration in [x25 uS])
} __attribute__((packed)) packet_robot;

/**
 * ROS Node to send robot control packet in serial.
 */
class SerialNode : public rclcpp::Node
{
public:
  /**
   * Constructor.
   *
   * Create a UDP Server and two ROS subscribtion topic :
   * - order : send order for one robot.
   * - orders : send order for multiple robots.
   */
  SerialNode() : Node("serial"), max_ball_speed(6.5)
  {
    controls_subscription_ = this->create_subscription<rostron_interfaces::msg::Orders>(
        "orders", 10, std::bind(&SerialNode::orders_callback, this, _1));

    std::string port("/dev/ttyACM0");
    unsigned long baud = 9600;

    serial::Timeout to(serial::Timeout::max(), 1000, 1, 10, 10);

    mySerial = new serial::Serial(port, baud, to);

    RCLCPP_INFO(get_logger(), "Serial open : %d", mySerial->isOpen());
  }

private:
  /**
   * \brief Callback to send control for multiple robots.
   */
  void orders_callback(const rostron_interfaces::msg::Orders::SharedPtr msg_orders)
  {
    RCLCPP_DEBUG(get_logger(), "Send control...");

    for (auto msg : msg_orders->orders)
    {
      packet_robot sending;

      RCLCPP_DEBUG(get_logger(), "--- id : %d ", msg.id);
      sending.id = msg.id;

      // Speed
      sending.x_speed = msg.velocity.linear.x;
      sending.y_speed = msg.velocity.linear.y;
      sending.t_speed = msg.velocity.angular.z;

      RCLCPP_DEBUG(get_logger(), "------ forward (x) : %.2f ", msg.velocity.linear.x);
      RCLCPP_DEBUG(get_logger(), "------ left (y) : %.2f", msg.velocity.linear.y);
      RCLCPP_DEBUG(get_logger(), "------ angular (z) : %.2f", msg.velocity.angular.z);

      // Kicker, Dribbler

      if (msg.hardware.kick_type == msg.hardware.FLAT_KICK)
      {
        RCLCPP_DEBUG(get_logger(), "--- kick : FLAT");
        RCLCPP_INFO(get_logger(), "TODO");
      }
      else if (msg.hardware.kick_type == msg.hardware.CHIP_KICK)
      {
        RCLCPP_DEBUG(get_logger(), "--- kick : CHIP");
        RCLCPP_INFO(get_logger(), "TODO");
      }
      mySerial->write((uint8_t*)&sending, sizeof(packet_robot));
    }


    RCLCPP_DEBUG(get_logger(), "End sending control...\n");
  }

  /**
   * \brief Subscribtion for "order" topic.
   * \deprecated
   */
  rclcpp::Subscription<rostron_interfaces::msg::Order>::SharedPtr control_subscription_;
  /**
   * \brief Subscribtion for "orders" topic.
   */
  rclcpp::Subscription<rostron_interfaces::msg::Orders>::SharedPtr controls_subscription_;

  /**
   * \brief Constant about the maximum ball speed of the ball.
   */
  float max_ball_speed;

  serial::Serial* mySerial;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialNode>());
  rclcpp::shutdown();
  return 0;
}
