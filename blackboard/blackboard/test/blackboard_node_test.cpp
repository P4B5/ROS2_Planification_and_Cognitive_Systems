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

#include <string>

#include "gtest/gtest.h"

#include "blackboard/BlackBoardNode.hpp"
#include "blackboard/BlackBoardClient.hpp"

#include "rclcpp/rclcpp.hpp"

TEST(blackboard_node, add_get_entry)
{
  auto blackboard = blackboard::BlackBoardNode::make_shared();
  auto client_1 = blackboard::BlackBoardClient::make_shared();

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::executor::ExecutorArgs(), 8);
  exe.add_node(blackboard->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  auto entry_1 = blackboard::Entry<bool>::make_shared(true);

  auto entry_base = entry_1->to_base();
  auto entry_2 = blackboard::Entry<std::string>::make_shared("Hi!!");

  auto entry_3 = blackboard::Entry<float>::make_shared(3.5);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "base";
  tf.transform.translation.x = 1.5;
  tf.transform.translation.y = 2.5;
  tf.transform.translation.z = 0.0;
  tf.transform.rotation.x = 1;
  tf.transform.rotation.y = 1;
  tf.transform.rotation.z = 1;
  tf.transform.rotation.w = 1;
  auto entry_4 = blackboard::Entry<geometry_msgs::msg::TransformStamped>::make_shared(tf);

  octomap_msgs::msg::Octomap octomap;
  octomap.header.frame_id = "map";
  octomap.binary = true;
  octomap.id = "octomap";
  octomap.resolution = 5.0;
  octomap.data = {1, 2, 1, 4, 5};
  auto entry_5 = blackboard::Entry<octomap_msgs::msg::Octomap>::make_shared(octomap);

  geometry_msgs::msg::PoseStamped wp;
  wp.header.frame_id = "base";
  wp.pose.position.x = -1.0;
  wp.pose.position.y = 3.45;
  wp.pose.position.z = 0.0;
  wp.pose.orientation.x = 0.0;
  wp.pose.orientation.y = 0.0;
  wp.pose.orientation.z = 1.0;
  wp.pose.orientation.w = 0.0;
  auto entry_6 = blackboard::Entry<geometry_msgs::msg::PoseStamped>::make_shared(wp);

  client_1->add_entry("my_entry_1", entry_1->to_base());
  client_1->add_entry("my_entry_2", entry_2->to_base());
  client_1->add_entry("my_entry_3", entry_3->to_base());
  client_1->add_entry("my_entry_4", entry_4->to_base());
  client_1->add_entry("my_entry_5", entry_5->to_base());
  client_1->add_entry("my_entry_6", entry_6->to_base());

  auto entry_1_got = blackboard::as<bool>(client_1->get_entry("my_entry_1"));
  auto entry_2_got = blackboard::as<std::string>(client_1->get_entry("my_entry_2"));
  auto entry_3_got = blackboard::as<float>(client_1->get_entry("my_entry_3"));
  auto entry_4_got = blackboard::as<geometry_msgs::msg::TransformStamped>(client_1->get_entry("my_entry_4"));
  auto entry_5_got = blackboard::as<octomap_msgs::msg::Octomap>(client_1->get_entry("my_entry_5"));
  auto entry_6_got = blackboard::as<geometry_msgs::msg::PoseStamped>(client_1->get_entry("my_entry_6"));

  ASSERT_TRUE(entry_1_got->data_);
  ASSERT_EQ(entry_2_got->data_, "Hi!!");
  ASSERT_FLOAT_EQ(entry_3_got->data_, 3.5);
  ASSERT_EQ(entry_4_got->data_, tf);
  ASSERT_EQ(entry_5_got->data_, octomap);
  ASSERT_EQ(entry_6_got->data_, wp);

  finish = true;
  t.join();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
