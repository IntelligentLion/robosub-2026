#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

class CommandAdapter : public rclcpp::Node
{
public:
  CommandAdapter()
  : Node("command_adapter")
  {
    command_sub_ = this->create_subscription<std_msgs::msg::String>(
      "movement_info",
      10,
      std::bind(&CommandAdapter::command_callback, this, std::placeholders::_1));

    thruster_pub_ = this->create_publisher<std_msgs::msg::Int32>("thruster_pwm", 10);
  }

private:
  void command_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    std_msgs::msg::Int32 thruster_msg;

    if (msg->data == "Submerge")
    {
      thruster_msg.data = 1400;
    }
    else
    {
      thruster_msg.data = 1500;
    }

    thruster_pub_->publish(thruster_msg);
    RCLCPP_INFO(
      this->get_logger(),
      "Mapped movement command '%s' to thruster_pwm=%d",
      msg->data.c_str(),
      thruster_msg.data);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr thruster_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CommandAdapter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
