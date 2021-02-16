#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "time.h"
#include "cmath"
#include "vector"

#include <iostream>
#include <string>
#include <random>

using namespace std::chrono_literals;


const int MAX_RANGE = 360;


const int rand_max = 4.0;
const int MAX_LECTURES = 100; //max number of lectures in our laser

rclcpp::Node::SharedPtr node_sub = nullptr;

//function to generate random values from 1.0 to 4.0
//revise values!!! there are some values less than 1.0
/*float floatRand() {
  return distribution(generator);
}*/

void sub_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser)
{
  RCLCPP_INFO(node_sub->get_logger(), "I heard: [%f]", laser->angle_max);
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node_pub = rclcpp::Node::make_shared("node_scan_pub");
  auto publisher = node_pub->create_publisher<sensor_msgs::msg::LaserScan>(
    "laser_data", rclcpp::QoS(1).best_effort());
  sensor_msgs::msg::LaserScan laser_message;

  node_sub = rclcpp::Node::make_shared("node_scan_sub");
  auto subscription = node_sub->create_subscription<sensor_msgs::msg::LaserScan>(
    "laser_data", rclcpp::QoS(100).best_effort(), sub_callback);

  /*
    band width = 1 lecture per second
    QoS Best effort --> laser data is admisable to lost casual info
  */

  rclcpp::Rate loop_rate(1000ms); //1000ms = 1s = 1Hz

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_pub);
  executor.add_node(node_sub);


  while (rclcpp::ok()) {
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(4.0,1.0);

    std::vector<float> ranges;

    //add 100 random lectures to the ranges array --> implement 3.6 each position
    for (int i = 0; i < 100; i++)
    {
      float number = distribution(generator);
      ranges.push_back(number);
      RCLCPP_INFO(node_pub->get_logger(), "rand number [%f]", number);
    }

    //message data
    laser_message.angle_min = 0;      //min angle of laser in rad
    laser_message.angle_max = 2*M_PI; //max angle of laser in rad
    laser_message.angle_increment = M_PI/50;
    laser_message.scan_time = 1.0;
    laser_message.ranges = ranges;




    publisher->publish(laser_message);
    RCLCPP_INFO(node_pub->get_logger(), "Publishing laser data...");
    executor.spin_once();
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
