#ifndef LASER_SCAN_PUB_HPP_
#define LASER_SCAN_PUB_HPP_

#include "std_msgs/msg/string.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace example_test
{

class ExampleSubscriber : public rclcpp_lifecycle::LifecycleNode
{
public:
  ExampleSubscriber();

private:
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &);

protected:
  virtual void message_callback(const std_msgs::msg::String::SharedPtr msg);
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

}  // namespace example_test

#endif  // EXAMPLE_TEST__EXAMPLESUBSCRIBER_HPP_