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
#include "blackboard/BlackBoard.hpp"


class SyncNode : public rclcpp::Node
{
public:
    SyncNode() : rclcpp::Node("node", rclcpp::NodeOptions())
    {

        declare_parameter("waypoints");
        declare_parameter("waypoint_coords");
  
        std::vector<std::string> wp_names;
        get_parameter_or("waypoints", wp_names, {});

        for (auto & wp : wp_names) {
            declare_parameter("waypoint_coords." + wp);

            std::vector<double> coords;
            get_parameter_or("waypoint_coords." + wp, coords, {});
            std::cout << ": [" << coords[0] << ", " << coords[1] << ", " << coords[2] << "] "<< std::endl;
            
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "/map";
            pose.header.stamp = now();
            pose.pose.position.x = coords[0];
            pose.pose.position.y = coords[1];
            pose.pose.position.z = coords[2];

            auto pose_bb = blackboard::Entry<geometry_msgs::msg::PoseStamped>::make_shared(pose);   
            blackboard->add_entry(wp, pose_bb->to_base());

        }
    }


    std::shared_ptr<blackboard::BlackBoard> blackboard = blackboard::BlackBoard::make_shared();



};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SyncNode>();   
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
