// Copyright 2021 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "cmath"
#include "vector"
#include <random>

using namespace std::chrono_literals;

/*
ros2 topic hz /laser_data --> It shows it goes 1 Hz
ros2 topic bw /laser_data --> It shows the band width, 460 b/s
QoS Best effort --> laser data is admisable to lost casual info
*/

const int MAX_LECTURES = 100;  // max number of lectures in our laser

rclcpp::Node::SharedPtr node_sub = nullptr;

void sub_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser)
{
  float max = 0.0;
  float min = 10.0;
  float accum = 0.0;
  for (int i = 0; i < MAX_LECTURES; i++) {
    if (laser->ranges[i] >= max) {
      max = laser->ranges[i];
    }
    if (laser->ranges[i] <= min) {
      min = laser->ranges[i];
    }

    accum += laser->ranges[i];
  }
  float average = accum / MAX_LECTURES;

  RCLCPP_INFO(node_sub->get_logger(), "Max: [%f]", max);
  RCLCPP_INFO(node_sub->get_logger(), "Min: [%f]", min);
  RCLCPP_INFO(node_sub->get_logger(), "Average: [%f]", average);
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
  rclcpp::Rate loop_rate(1000ms);  // 1000ms = 1s = 1Hz

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_pub);
  executor.add_node(node_sub);


  while (rclcpp::ok()) {
<<<<<<< HEAD
    
=======
>>>>>>> f6f6244bd93c301077e5cf69b65f5a3172b100be
    std::vector<float> values;

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::normal_distribution<float> distribution(4, 1.0);


    // add 100 random lectures to the ranges array --> implement 3.6 each position
    for (int i = 0; i < MAX_LECTURES; i++) {
      values.push_back(distribution(generator));
<<<<<<< HEAD
      // intensities.push_back(distribution(generator));
=======
>>>>>>> f6f6244bd93c301077e5cf69b65f5a3172b100be
    }

    laser_message.header.frame_id = "scan_fake_frame";
    laser_message.angle_min = -M_PI;  // min angle of laser in rad
    laser_message.angle_max = M_PI;  // max angle of laser in rad
    laser_message.angle_increment = M_PI / 50;  // Increment the angle each 3.6º
    laser_message.scan_time = 1.0;
    laser_message.range_min = 0.0;
    laser_message.range_max = 100.0;
    laser_message.ranges = values;
<<<<<<< HEAD
=======

>>>>>>> f6f6244bd93c301077e5cf69b65f5a3172b100be

    publisher->publish(laser_message);
    RCLCPP_INFO(node_pub->get_logger(), "Publishing laser data...");
    executor.spin_once();
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
