#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "rostron_interfaces/msg/commands.hpp"
#include "rostron_sender/com/mainboard.h"

using std::placeholders::_1;

/**
 * ROS Node to send robot control packet in serial.
 */
class SerialNode : public rclcpp::Node {
   public:
    /**
     * Subscribe to a commands topic to send on robots.
     *
     * \todo Add a parameter to choose port, baudrate.
     */
    SerialNode()
        : Node("serial"), mainboard(create_mainboard()) {
        commands_subscription_ =
            this->create_subscription<rostron_interfaces::msg::Commands>(
                "commands", 10,
                std::bind(&SerialNode::commands_callback, this, _1));

        RCLCPP_INFO(get_logger(), "Serial open : %d",
                    mainboard.mySerial->isOpen());
    }

   private:
    Mainboard create_mainboard() {
        const auto usb_params = "usb_port";
        const auto baudrate_params = "baudrate";

        this->declare_parameter<std::string>(usb_params, "/dev/ttyACM0");
        std::string port = this->get_parameter(usb_params).as_string();

        this->declare_parameter<int>(baudrate_params, 9600);
        int baudrate = this->get_parameter(baudrate_params).as_int();

        return Mainboard(port, baudrate);
    }

    /**
     * \brief Callback to send commands for multiple robots.
     */
    void commands_callback(
        const rostron_interfaces::msg::Commands::SharedPtr msg_commands) {
        RCLCPP_DEBUG(get_logger(), "Send commands...");

        for (auto msg : msg_commands->commands) {
            packet_mainboard sending;

            RCLCPP_DEBUG(get_logger(), "--- id : %d ", msg.id);
            sending.id = msg.id;

            // Speed
            sending.x_speed = int(msg.velocity.linear.x / 1000);
            sending.y_speed = int(msg.velocity.linear.y / 1000);
            sending.t_speed = int(msg.velocity.angular.z / 1000);

            RCLCPP_DEBUG(get_logger(), "------ forward (x) : %.2f ",
                         msg.velocity.linear.x);
            RCLCPP_DEBUG(get_logger(), "------ left (y) : %.2f",
                         msg.velocity.linear.y);
            RCLCPP_DEBUG(get_logger(), "------ angular (z) : %.2f",
                         msg.velocity.angular.z);

            // Kicker, Dribbler

            if (msg.hardware.kick_type == msg.hardware.FLAT_KICK) {
                RCLCPP_DEBUG(get_logger(), "--- kick : FLAT");
                RCLCPP_INFO(get_logger(), "TODO");
                sending.actions |= ACTION_KICK1;
            } else if (msg.hardware.kick_type == msg.hardware.CHIP_KICK) {
                RCLCPP_DEBUG(get_logger(), "--- kick : CHIP");
                RCLCPP_INFO(get_logger(), "TODO");
                sending.actions |= ACTION_KICK2;
            }

            if (msg.hardware.spin_power > 0) {
                RCLCPP_DEBUG(get_logger(), "--- dribbler : ON");
                sending.actions |= ACTION_DRIBBLE;
                // sending.actions.spin_power = msg.hardware.spin_power;
            }
            mainboard.addPacket((uint8_t *)&sending);
        }

        mainboard.send();
        RCLCPP_DEBUG(get_logger(), "End sending command...\n");
    }

    /**
     * \brief Subscribtion for "commands" topic.
     */
    rclcpp::Subscription<rostron_interfaces::msg::Commands>::SharedPtr
        commands_subscription_;

    Mainboard mainboard;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialNode>());
    rclcpp::shutdown();
    return 0;
}
