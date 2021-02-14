#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "time.h"

using namespace std::chrono_literals;

const int rand_max = 4.0;
const int MAX_LECTURES = 100; //max number of lectures in our laser

rclcpp::Node::SharedPtr node_sub = nullptr;

//function to generate random values from 1.0 to 4.0
//revise values!!! there are some values less than 1.0
double doubleRand() {
  return rand_max * double(rand()) / (double(RAND_MAX) + 1.0);
}

void sub_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser)
{
  RCLCPP_INFO(node_sub->get_logger(), "I heard: [%s]", laser->angle_max);
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node_pub = rclcpp::Node::make_shared("node_scan_pub");
  auto publisher = node_pub->create_publisher<sensor_msgs::msg::LaserScan>(
    "laser_data", rclcpp::QoS(100).transient_local());
  
  sensor_msgs::msg::LaserScan laser_message;

  node_sub = rclcpp::Node::make_shared("node_scan_sub");
  auto subscription = node_sub->create_subscription<sensor_msgs::msg::LaserScan>(
    "laser_data", rclcpp::QoS(100).transient_local(), sub_callback);

 
  rclcpp::Rate loop_rate(1000ms); //1000ms = 1s = 1Hz

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_pub);
  executor.add_node(node_sub);
  
  while (rclcpp::ok()) {

    laser_message.angle_min = 0;   //min angle of laser
    laser_message.angle_max = 360; //max angle of laser

    //define here 100 random lectures of the laser
    //float laser_intensities[MAX_LECTURES];
    //laser_message.intensities = laser_intensities

    RCLCPP_INFO(node_pub->get_logger(), "rand number [%f]", doubleRand());


    // RCLCPP_INFO(node->get_logger(), "Publishing [%s]", message.data.c_str());
    publisher->publish(laser_message);
    RCLCPP_INFO(node_pub->get_logger(), "Publishing laser data...");
    executor.spin_some();
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}