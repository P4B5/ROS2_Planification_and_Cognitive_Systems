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

#ifndef BLACKBOARD__BLACKBOARDNODE_HPP_
#define BLACKBOARD__BLACKBOARDNODE_HPP_

#include <memory>

#include "blackboard_msgs/srv/add_entry.hpp"
#include "blackboard_msgs/srv/get_entry.hpp"

#include "blackboard/BlackBoard.hpp"

#include "rclcpp/rclcpp.hpp"

namespace blackboard
{

class BlackBoardNode : public rclcpp::Node
{
public:
  using Ptr = std::shared_ptr<BlackBoardNode>;
  static Ptr make_shared()
  {
    return std::make_shared<BlackBoardNode>();
  }

  BlackBoardNode();

  void add_entry_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<blackboard_msgs::srv::AddEntry::Request> request,
    const std::shared_ptr<blackboard_msgs::srv::AddEntry::Response> response);

  void get_entry_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<blackboard_msgs::srv::GetEntry::Request> request,
    const std::shared_ptr<blackboard_msgs::srv::GetEntry::Response> response);

private:
  BlackBoard blackboard_;

  rclcpp::Service<blackboard_msgs::srv::AddEntry>::SharedPtr
    add_entry_service_;

  rclcpp::Service<blackboard_msgs::srv::GetEntry>::SharedPtr
    get_entry_service_;
};

}  // namespace blackboard

#endif  // BLACKBOARD__BLACKBOARDNODE_HPP_
