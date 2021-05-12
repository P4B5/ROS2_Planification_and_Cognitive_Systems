// Copyright 2019 Intelligent Robotics Lab
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

#include <memory>
#include <string>
#include <array>
#include <iostream>
#include <vector>
#include <functional>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "blackboard/BlackBoard.hpp"
#include "blackboard/BlackBoardNode.hpp"
#include "blackboard/BlackBoardClient.hpp"


class SyncNode : public rclcpp::Node
{
public:
    SyncNode() : rclcpp::Node("sync_node", rclcpp::NodeOptions())
    { 
        declare_parameter("waypoints");
        declare_parameter("waypoint_coords");
  
        std::vector<std::string> wp_names;
        get_parameter_or("waypoints", wp_names, {});

        for (auto & wp : wp_names) {
            declare_parameter("waypoint_coords." + wp);

            std::vector<double> coords;
            get_parameter_or("waypoint_coords." + wp, coords, {});
            std::cout << wp << ": [" << coords[0] << ", " << coords[1] << ", " << coords[2] << "] "<< std::endl;
            
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "/map";
            pose.header.stamp = now();
            pose.pose.position.x = coords[0];
            pose.pose.position.y = coords[1];
            pose.pose.position.z = coords[2];

            auto pose_bb = blackboard::Entry<geometry_msgs::msg::PoseStamped>::make_shared(pose);   
            client_sync->add_entry(wp, pose_bb->to_base());


            geometry_msgs::msg::TransformStamped transform;
            transform.header.frame_id = "/map";
            transform.child_frame_id = "/"+wp;
            transform.header.stamp = now();
            transform.transform.translation.x = coords[0];
            transform.transform.translation.y = coords[1];
            transform.transform.translation.z = coords[2];

            auto transform_bb = blackboard::Entry<geometry_msgs::msg::TransformStamped>::make_shared(transform);   
            client_sync->add_entry(wp+"_tf", transform_bb->to_base());
        }

        object_sub_ = this->create_subscription<std_msgs::msg::String>("/object_discover", 10, std::bind(&SyncNode::current_object_callback, this, std::placeholders::_1));
        tf_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>("/tf_object", 10, std::bind(&SyncNode::current_tf_callback, this,std::placeholders::_1));
    }

    void current_object_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        auto object_bb = blackboard::Entry<bool>::make_shared(true); 
        client_sync->add_entry(msg->data, object_bb->to_base());

        std::cout << msg->data << std::endl;
    }

    void current_tf_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped tf_object;
        tf_object.header.frame_id = msg->header.frame_id;
        tf_object.child_frame_id = msg->child_frame_id;
        tf_object.header.stamp = msg->header.stamp;
        tf_object.transform.translation.x = msg->transform.translation.x;
        tf_object.transform.translation.y = msg->transform.translation.y;
        tf_object.transform.translation.z = msg->transform.translation.z;

        auto tf_object_bb = blackboard::Entry<geometry_msgs::msg::TransformStamped>::make_shared(tf_object);   
        client_sync->add_entry(msg->child_frame_id+"_tf", tf_object_bb->to_base());
    }
    
private:
    std::shared_ptr<blackboard::BlackBoardClient> client_sync = blackboard::BlackBoardClient::make_shared();
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr tf_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr object_sub_;
};