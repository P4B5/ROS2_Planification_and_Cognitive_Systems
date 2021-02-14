



#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "time.h"

using namespace std::chrono_literals;

const int rand_max = 4.0;
const int MAX_LECTURES = 100; //max number of lectures in our laser


//function to generate random values from 1.0 to 4.0
//revise values!!! there are some values less than 1.0
double doubleRand() {
  return rand_max * double(rand()) / (double(RAND_MAX) + 1.0);
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("node_scan_pub");
  auto publisher = node->create_publisher<sensor_msgs::msg::LaserScan>(
    "laser_data", rclcpp::QoS(100).transient_local());
  
  sensor_msgs::msg::LaserScan laser_message;

 
  rclcpp::Rate loop_rate(1000ms); //1000ms = 1s = 1Hz
  while (rclcpp::ok()) {

    laser_message.angle_min = 0;   //min angle of laser
    laser_message.angle_max = 360; //max angle of laser

    //define here 100 random lectures of the laser
    //float laser_intensities[MAX_LECTURES];
    //laser_message.intensities = laser_intensities

    RCLCPP_INFO(node->get_logger(), "rand number [%f]", doubleRand());

  
    // RCLCPP_INFO(node->get_logger(), "Publishing [%s]", message.data.c_str());
    publisher->publish(laser_message);
    RCLCPP_INFO(node->get_logger(), "Publishing laser data...");
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}