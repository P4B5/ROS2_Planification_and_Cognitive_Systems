#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("node_scan_pub");
  auto publisher = node->create_publisher<std_msgs::msg::String>(
    "chatter", rclcpp::QoS(100).transient_local());
  
  std_msgs::msg::String message;
  int counter = 0;

  rclcpp::Rate loop_rate(500ms);
  while (rclcpp::ok()) {
    message.data = "Hello, world! " + std::to_string(counter++);

    RCLCPP_INFO(node->get_logger(), "Publishing [%s]", message.data.c_str());
    
    publisher->publish(message);

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}