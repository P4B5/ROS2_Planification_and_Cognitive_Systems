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
#include <memory>

#include "blackboard_msgs/srv/add_entry.hpp"
#include "blackboard_msgs/srv/get_entry.hpp"

#include "blackboard/BlackBoardClient.hpp"

#include "rclcpp/rclcpp.hpp"

namespace blackboard
{

BlackBoardClient::BlackBoardClient()
{
  client_node_ = rclcpp::Node::make_shared("blackboard_client");

  add_entry_client_ = client_node_->create_client<blackboard_msgs::srv::AddEntry>(
    "blackboard/add_entry");
  get_entry_client_ = client_node_->create_client<blackboard_msgs::srv::GetEntry>(
    "blackboard/get_entry");
}

void
BlackBoardClient::add_entry(const std::string & key, EntryBase::Ptr entry)
{
  while (!add_entry_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return;
    }
    RCLCPP_ERROR_STREAM(
      client_node_->get_logger(),
      add_entry_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<blackboard_msgs::srv::AddEntry::Request>();

  if (entry->get_type() == EntryBase::BOOL) {
    request->entry.type = blackboard_msgs::msg::Entry::BOOL_TYPE;
    request->entry.key = key;
    request->entry.bool_entry = blackboard::as<bool>(entry)->data_;
  }

  if (entry->get_type() == EntryBase::STRING) {
    request->entry.type = blackboard_msgs::msg::Entry::STRING_TYPE;
    request->entry.key = key;
    request->entry.string_entry = blackboard::as<std::string>(entry)->data_;
  }

  if (entry->get_type() == EntryBase::FLOAT) {
    request->entry.type = blackboard_msgs::msg::Entry::FLOAT_TYPE;
    request->entry.key = key;
    request->entry.float_entry = blackboard::as<float>(entry)->data_;
  }

  if (entry->get_type() == EntryBase::TRANSFORM) {
    request->entry.type = blackboard_msgs::msg::Entry::TRANSFORM_TYPE;
    request->entry.key = key;
    request->entry.transform_entry = blackboard::as<geometry_msgs::msg::TransformStamped>(entry)->data_;
  }

  if (entry->get_type() == EntryBase::OCTOMAP) {
    request->entry.type = blackboard_msgs::msg::Entry::OCTOMAP_TYPE;
    request->entry.key = key;
    request->entry.octomap_entry = blackboard::as<octomap_msgs::msg::Octomap>(entry)->data_;
  }

  auto future_result = add_entry_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(client_node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return;
  }

  if (!future_result.get()->success) {
    RCLCPP_ERROR_STREAM(
      client_node_->get_logger(),
      add_entry_client_->get_service_name() << ": Error");
  }
}

EntryBase::Ptr
BlackBoardClient::get_entry(const std::string & key)
{
  while (!get_entry_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return nullptr;
    }
    RCLCPP_ERROR_STREAM(
      client_node_->get_logger(),
      get_entry_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<blackboard_msgs::srv::GetEntry::Request>();

  request->key = key;

  auto future_result = get_entry_client_->async_send_request(request);

  EntryBase::Ptr ret = nullptr;
  if (rclcpp::spin_until_future_complete(client_node_, future_result, std::chrono::seconds(1)) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    switch (future_result.get()->entry.type) {
      case blackboard_msgs::msg::Entry::BOOL_TYPE:
        {
          ret = blackboard::Entry<bool>::make_shared(future_result.get()->entry.bool_entry);
          return ret;
        }
        break;
      case blackboard_msgs::msg::Entry::STRING_TYPE:
        {
          ret = blackboard::Entry<std::string>::make_shared(
            future_result.get()->entry.string_entry);
          return ret;
        }
        break;
      case blackboard_msgs::msg::Entry::FLOAT_TYPE:
        {
          ret = blackboard::Entry<float>::make_shared(future_result.get()->entry.float_entry);
          return ret;
        }
        break;
      case blackboard_msgs::msg::Entry::TRANSFORM_TYPE:
        {
          ret = blackboard::Entry<geometry_msgs::msg::TransformStamped>::make_shared(future_result.get()->entry.transform_entry);
          return ret;
        }
        break;
      case blackboard_msgs::msg::Entry::OCTOMAP_TYPE:
        {
          ret = blackboard::Entry<octomap_msgs::msg::Octomap>::make_shared(future_result.get()->entry.octomap_entry);
          return ret;
        }
        break;
      default:
        break;
    }
  } else {
    RCLCPP_ERROR_STREAM(
      client_node_->get_logger(),
      add_entry_client_->get_service_name() << ": Error calling service");
  }

  if (!future_result.get()->success) {
    RCLCPP_ERROR_STREAM(
      client_node_->get_logger(),
      add_entry_client_->get_service_name() << ": Error");
  }

  return ret;
}

}  // namespace blackboard
