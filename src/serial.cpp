#include <rclcpp/rclcpp.hpp>
#include "rostron_interfaces/msg/commands.hpp"

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
   * Subscribe to a commands topic to send on robots.
   * 
   * \todo Add a parameter to choose port, baudrate.
   */
  SerialNode() : Node("serial"), max_ball_speed(6.5)
  {
    commands_subscription_ = this->create_subscription<rostron_interfaces::msg::Commands>(
        "commands", 10, std::bind(&SerialNode::commands_callback, this, _1));

    std::string port("/dev/ttyACM0");
    unsigned long baud = 9600;

    serial::Timeout to(serial::Timeout::max(), 1000, 1, 10, 10);

    mySerial = new serial::Serial(port, baud, to);

    RCLCPP_INFO(get_logger(), "Serial open : %d", mySerial->isOpen());
  }

private:
  /**
   * \brief Callback to send commands for multiple robots.
   */
  void commands_callback(const rostron_interfaces::msg::Commands::SharedPtr msg_commands)
  {
    RCLCPP_DEBUG(get_logger(), "Send commands...");

    for (auto msg : msg_commands->commands)
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
        sending.actions |= ACTION_KICK1;
      }
      else if (msg.hardware.kick_type == msg.hardware.CHIP_KICK)
      {
        RCLCPP_DEBUG(get_logger(), "--- kick : CHIP");
        RCLCPP_INFO(get_logger(), "TODO");
        sending.actions |= ACTION_KICK2;
      }

      if(msg.hardware.spin_power > 0)
      {
        RCLCPP_DEBUG(get_logger(), "--- dribbler : ON");
        sending.actions |= ACTION_DRIBBLE;
        // sending.actions.spin_power = msg.hardware.spin_power;
      }

      mySerial->write((uint8_t*)&sending, sizeof(packet_robot));
    }


    RCLCPP_DEBUG(get_logger(), "End sending command...\n");
  }

  /**
   * \brief Subscribtion for "commands" topic.
   */
  rclcpp::Subscription<rostron_interfaces::msg::Commands>::SharedPtr commands_subscription_;


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
