#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "time.h"
#include "cmath"
#include "vector"
#include <random>

using namespace std::chrono_literals;


rclcpp::Node::SharedPtr node_sub = nullptr;

void sub_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser)
{
  RCLCPP_INFO(node_sub->get_logger(), "I heard: [%f]", laser->ranges[0]);
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

    std::vector<float> values;

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::normal_distribution<float> distribution (4,1.0);


    //add 100 random lectures to the ranges array --> implement 3.6 each position
    for (int i = 0; i < 100; i++)
    {
      values.push_back(distribution(generator)); 
    }
    
    //message data

    laser_message.angle_min = -M_PI; //min angle of laser in rad
    laser_message.angle_max = M_PI; //max angle of laser in rad
    laser_message.angle_increment = M_PI/50;
    laser_message.scan_time = 1.0;
    laser_message.ranges = values;

    publisher->publish(laser_message);
    RCLCPP_INFO(node_pub->get_logger(), "Publishing laser data...");
    executor.spin_once();
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}