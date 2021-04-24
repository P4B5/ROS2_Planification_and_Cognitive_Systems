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

#include <type_traits>
#include <string>
#include <memory>

#include "blackboard_msgs/srv/add_entry.hpp"
#include "blackboard_msgs/msg/entry.hpp"

#include "blackboard/BlackBoardNode.hpp"

#include "rclcpp/rclcpp.hpp"

namespace blackboard
{

BlackBoardNode::BlackBoardNode()
: Node("blackboard")
{
  add_entry_service_ = create_service<blackboard_msgs::srv::AddEntry>(
    "blackboard/add_entry",
    std::bind(
      &BlackBoardNode::add_entry_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  get_entry_service_ = create_service<blackboard_msgs::srv::GetEntry>(
    "blackboard/get_entry",
    std::bind(
      &BlackBoardNode::get_entry_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));
}

void
BlackBoardNode::add_entry_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<blackboard_msgs::srv::AddEntry::Request> request,
  const std::shared_ptr<blackboard_msgs::srv::AddEntry::Response> response)
{
  (void)request_header;
  switch (request->entry.type) {
    case blackboard_msgs::msg::Entry::BOOL_TYPE:
      {
        auto entry = blackboard::Entry<bool>::make_shared(request->entry.bool_entry);
        blackboard_.add_entry(request->entry.key, entry->to_base());
      }
      break;
    case blackboard_msgs::msg::Entry::STRING_TYPE:
      {
        auto entry = blackboard::Entry<std::string>::make_shared(request->entry.string_entry);
        blackboard_.add_entry(request->entry.key, entry->to_base());
      }
      break;
    case blackboard_msgs::msg::Entry::FLOAT_TYPE:
      {
        auto entry = blackboard::Entry<float>::make_shared(request->entry.float_entry);
        blackboard_.add_entry(request->entry.key, entry->to_base());
      }
      break;
    case blackboard_msgs::msg::Entry::TRANSFORM_TYPE:
      {
        auto entry = blackboard::Entry<geometry_msgs::msg::TransformStamped>::make_shared(request->entry.transform_entry);
        blackboard_.add_entry(request->entry.key, entry->to_base());
      }
      break;
    case blackboard_msgs::msg::Entry::OCTOMAP_TYPE:
      {
        auto entry = blackboard::Entry<octomap_msgs::msg::Octomap>::make_shared(request->entry.octomap_entry);
        blackboard_.add_entry(request->entry.key, entry->to_base());
      }
      break;
    default:
      break;
  }

  response->success = true;
}

void
BlackBoardNode::get_entry_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<blackboard_msgs::srv::GetEntry::Request> request,
  const std::shared_ptr<blackboard_msgs::srv::GetEntry::Response> response)
{
  (void)request_header;
  auto entry = blackboard_.get_entry(request->key);

  if (entry->get_type() == EntryBase::BOOL) {
    response->entry.type = blackboard_msgs::msg::Entry::BOOL_TYPE;
    response->entry.key = request->key;
    response->entry.bool_entry = blackboard::as<bool>(entry)->data_;
  }

  if (entry->get_type() == EntryBase::STRING) {
    response->entry.type = blackboard_msgs::msg::Entry::STRING_TYPE;
    response->entry.key = request->key;
    response->entry.string_entry = blackboard::as<std::string>(entry)->data_;
  }

  if (entry->get_type() == EntryBase::FLOAT) {
    response->entry.type = blackboard_msgs::msg::Entry::FLOAT_TYPE;
    response->entry.key = request->key;
    response->entry.float_entry = blackboard::as<float>(entry)->data_;
  }

  if (entry->get_type() == EntryBase::TRANSFORM) {
    response->entry.type = blackboard_msgs::msg::Entry::TRANSFORM_TYPE;
    response->entry.key = request->key;
    response->entry.transform_entry = blackboard::as<geometry_msgs::msg::TransformStamped>(entry)->data_;
  }

  if (entry->get_type() == EntryBase::OCTOMAP) {
    response->entry.type = blackboard_msgs::msg::Entry::OCTOMAP_TYPE;
    response->entry.key = request->key;
    response->entry.octomap_entry = blackboard::as<octomap_msgs::msg::Octomap>(entry)->data_;
  }

  response->success = true;
}

}  // namespace blackboard
