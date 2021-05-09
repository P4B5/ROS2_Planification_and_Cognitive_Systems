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
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/point_cloud2.hpp>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>
#include <math.h>
#include <memory>
#include <string>
#include <map>
#include <algorithm>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>

#include <pcl/PCLHeader.h>
#include <std_msgs/msg/header.hpp>

#include <pcl/PCLImage.h>
#include <sensor_msgs/msg/image.hpp>

#include <pcl/PCLPointField.h>
#include <sensor_msgs/msg/point_field.hpp>

#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/PointIndices.h>
#include <pcl_msgs/msg/point_indices.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl_msgs/msg/model_coefficients.hpp>

#include <pcl/Vertices.h>
#include <pcl_msgs/msg/vertices.hpp>

#include <pcl/PolygonMesh.h>
#include <pcl_msgs/msg/polygon_mesh.hpp>

enum {IDX_h, IDX_H, IDX_s, IDX_S, IDX_v, IDX_V, NUM_HSV};

typedef struct
{
  float hsv[NUM_HSV];
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr hsv_subs[NUM_HSV];
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cloud_pub_;
}
HSVInfo;

namespace Filter
{
using std::placeholders::_1 ;
class Filter : public rclcpp::Node
{
  Filter() : Node("image_sub") //, buffer_() , listener_(buffer_)
  {
    //initHSV();
    image_sub_3D_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/kinect_range/image_depth", 10, std::bind(&Filter::imageCb, this, _1)); // camera/depth/points     camera/rgb/image_raw

  }

  void imageCb(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_in)
  {
    sensor_msgs::msg::PointCloud2 cloud;
   /* try
    {
      pcl_ros::transformPointCloud("map", *cloud_in, cloud, tfListener_);
    }
    catch(tf::TransformException & ex)
    {
      ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
      return;
    }*/


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(cloud, *pcrgb);

    for (int i=0; i < MAX_CHANNELS; i++)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcrgb_out(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
      for (it = pcrgb->begin(); it != pcrgb->end(); ++it)
      {
        if (!std::isnan(it->x))
        {
          pcl::PointXYZHSV hsv;
          pcl::PointXYZRGBtoXYZHSV(*it, hsv);

          if (isValid(i, hsv))
            pcrgb_out->push_back(*it);
        }
      }
    

      sensor_msgs::msg::PointCloud2 cloud_out;
      pcl::toROSMsg(*pcrgb_out, cloud_out);

      cloud_out.header.frame_id = cloud_in->header.frame_id;
      //cloud_out.header.stamp = rclcpp::spin(Node->get_node_base_interface());
    }
  }


  
private:

  bool isValid(int channel, const pcl::PointXYZHSV& hsv)
  {
    pcl::PointXYZHSV hsv_scaled;
    hsv_scaled.s = hsv.s * 100.0;
    hsv_scaled.v = hsv.v * 100.0;
    hsv_scaled.h = hsv.h;

    if ( hsv_scaled.h < hsvFilters_[channel].hsv[IDX_h] ||  hsv_scaled.h > hsvFilters_[channel].hsv[IDX_H] ||
        hsv_scaled.s < hsvFilters_[channel].hsv[IDX_s] ||  hsv_scaled.s > hsvFilters_[channel].hsv[IDX_S] ||
        hsv_scaled.v < hsvFilters_[channel].hsv[IDX_v] ||  hsv_scaled.v > hsvFilters_[channel].hsv[IDX_V])
        {
      return false;
        }
        
    else
    {
      
      return true;
    }
      
  }

  /*void initHSV()
  {
    for (int i = 0; i < MAX_CHANNELS; i++)
    {
      hsvFilters_[i].hsv[IDX_h] = hsvFilters_[i].hsv[IDX_s] = hsvFilters_[i].hsv[IDX_v] = 0.0;
      hsvFilters_[i].hsv[IDX_S] = hsvFilters_[i].hsv[IDX_V] = 255.0;
      hsvFilters_[i].hsv[IDX_H] = 360.0;

      char topic_id[256];
      sprintf(topic_id, "/hsv_filter/msg/%d/h", i);
      hsvFilters_[i].hsv_subs[IDX_h] = this->create_subscription<sensor_msgs::msg::Image>(topic_id, 1, std::bind(&Filter::hsvCB, this, _1) );
      sprintf(topic_id, "/hsv_filter/%d/H", i);
      hsvFilters_[i].hsv_subs[IDX_H] = this->create_subscription<sensor_msgs::msg::Image>(topic_id, 1, std::bind(&Filter::hsvCB, this, _1) );
      sprintf(topic_id, "/hsv_filter/%d/s", i);
      hsvFilters_[i].hsv_subs[IDX_s] = this->create_subscription<sensor_msgs::msg::Image>(topic_id, 1, std::bind(&Filter::hsvCB, this, _1) );
      sprintf(topic_id, "/hsv_filter/%d/S", i);
      hsvFilters_[i].hsv_subs[IDX_S] = this->create_subscription<sensor_msgs::msg::Image>(topic_id, 1, std::bind(&Filter::hsvCB, this, _1) );

      sprintf(topic_id, "/hsv_filter/%d/v", i);
      hsvFilters_[i].hsv_subs[IDX_v] = this->create_subscription<sensor_msgs::msg::Image>(topic_id, 1, std::bind(&Filter::hsvCB, this, _1) );
      sprintf(topic_id, "/hsv_filter/%d/V", i);
      hsvFilters_[i].hsv_subs[IDX_V] = this->create_subscription<sensor_msgs::msg::Image>(topic_id, 1, std::bind(&Filter::hsvCB, this, _1) );

      sprintf(topic_id, "/cloud_filtered/%d", i);
      hsvFilters_[i].cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2><sensor_msgs::msg::Image>(topic_id, 1, false);
    }
  }*/
  
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr image_sub_3D_; 
   static const int MAX_CHANNELS = 3;
  int x_ = 0;
  int y_ = 0;

  HSVInfo hsvFilters_[MAX_CHANNELS];

};  
}



int main(int argc, char * argv[])
{
  auto node_pub = rclcpp::Node::make_shared("node_filter");
  return 0;
}

