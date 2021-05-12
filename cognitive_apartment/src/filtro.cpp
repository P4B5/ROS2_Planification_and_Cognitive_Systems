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

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>



enum {IDX_h_red, IDX_H_red, IDX_s_red, IDX_S_red, IDX_v_red, IDX_V_red, NUM_HSV,
      IDX_h_blue, IDX_H_blue, IDX_s_blue, IDX_S_blue, IDX_v_blue, IDX_V_blue,
      IDX_h_green, IDX_H_green, IDX_s_green, IDX_S_green, IDX_v_green, IDX_V_green,
      IDX_h_yellow, IDX_H_yellow, IDX_s_yellow, IDX_S_yellow, IDX_v_yellow, IDX_V_yellow,
      IDX_h_orange, IDX_H_orange, IDX_s_orange, IDX_S_orange, IDX_v_orange, IDX_V_orange};

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
    initHSV();
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

    //tf2::doTransform(*cloud, *cloud, "map");

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

          if (isValid_green(i, hsv))
          {
            pcrgb_out->push_back(*it);
          }
          if (isValid_red(i, hsv))
          {
            pcrgb_out->push_back(*it);
          }
          if (isValid_blue(i, hsv))
          {
            pcrgb_out->push_back(*it);
          }
          if (isValid_yellow(i, hsv))
          {
            pcrgb_out->push_back(*it);
          }
          if (isValid_orange(i, hsv))
          {
            pcrgb_out->push_back(*it);
          }
            
        }
      }
    

      sensor_msgs::msg::PointCloud2 cloud_out;
      pcl::toROSMsg(*pcrgb_out, cloud_out);

      cloud_out.header.frame_id = cloud_in->header.frame_id;
      //cloud_out.header.stamp = rclcpp::spin(Node->get_node_base_interface());
    }
  }


  
private:

  bool isValid_green(int channel, const pcl::PointXYZHSV& hsv)
  {
    pcl::PointXYZHSV hsv_scaled;
    hsv_scaled.s = hsv.s * 100.0;
    hsv_scaled.v = hsv.v * 100.0;
    hsv_scaled.h = hsv.h;

    if ( hsv_scaled.h < hsvFilters_[channel].hsv[IDX_h_green] ||  hsv_scaled.h > hsvFilters_[channel].hsv[IDX_H_green] ||
        hsv_scaled.s < hsvFilters_[channel].hsv[IDX_s_green] ||  hsv_scaled.s > hsvFilters_[channel].hsv[IDX_S_green] ||
        hsv_scaled.v < hsvFilters_[channel].hsv[IDX_v_green] ||  hsv_scaled.v > hsvFilters_[channel].hsv[IDX_V_green])
        {
      return false;
        }
    else
    {
      return true;
    }
  }

  bool isValid_yellow(int channel, const pcl::PointXYZHSV& hsv)
  {
    pcl::PointXYZHSV hsv_scaled;
    hsv_scaled.s = hsv.s * 100.0;
    hsv_scaled.v = hsv.v * 100.0;
    hsv_scaled.h = hsv.h;

    if ( hsv_scaled.h < hsvFilters_[channel].hsv[IDX_h_yellow] ||  hsv_scaled.h > hsvFilters_[channel].hsv[IDX_H_yellow] ||
        hsv_scaled.s < hsvFilters_[channel].hsv[IDX_s_yellow] ||  hsv_scaled.s > hsvFilters_[channel].hsv[IDX_S_yellow] ||
        hsv_scaled.v < hsvFilters_[channel].hsv[IDX_v_yellow] ||  hsv_scaled.v > hsvFilters_[channel].hsv[IDX_V_yellow])
        {
      return false;
        }
        
    else
    {
      return true;
    }
  }

  bool isValid_blue(int channel, const pcl::PointXYZHSV& hsv)
  {
    pcl::PointXYZHSV hsv_scaled;
    hsv_scaled.s = hsv.s * 100.0;
    hsv_scaled.v = hsv.v * 100.0;
    hsv_scaled.h = hsv.h;

    if ( hsv_scaled.h < hsvFilters_[channel].hsv[IDX_h_blue] ||  hsv_scaled.h > hsvFilters_[channel].hsv[IDX_H_blue] ||
        hsv_scaled.s < hsvFilters_[channel].hsv[IDX_s_blue] ||  hsv_scaled.s > hsvFilters_[channel].hsv[IDX_S_blue] ||
        hsv_scaled.v < hsvFilters_[channel].hsv[IDX_v_blue] ||  hsv_scaled.v > hsvFilters_[channel].hsv[IDX_V_blue])
        {
      return false;
        }
        
    else
    {
      return true;
    }
  }

  bool isValid_red(int channel, const pcl::PointXYZHSV& hsv)
  {
    pcl::PointXYZHSV hsv_scaled;
    hsv_scaled.s = hsv.s * 100.0;
    hsv_scaled.v = hsv.v * 100.0;
    hsv_scaled.h = hsv.h;

    if ( hsv_scaled.h < hsvFilters_[channel].hsv[IDX_h_red] ||  hsv_scaled.h > hsvFilters_[channel].hsv[IDX_H_red] ||
        hsv_scaled.s < hsvFilters_[channel].hsv[IDX_s_red] ||  hsv_scaled.s > hsvFilters_[channel].hsv[IDX_S_red] ||
        hsv_scaled.v < hsvFilters_[channel].hsv[IDX_v_red] ||  hsv_scaled.v > hsvFilters_[channel].hsv[IDX_V_red])
        {
      return false;
        }
        
    else
    {
      return true;
    }
  }

  bool isValid_orange(int channel, const pcl::PointXYZHSV& hsv)
  {
    pcl::PointXYZHSV hsv_scaled;
    hsv_scaled.s = hsv.s * 100.0;
    hsv_scaled.v = hsv.v * 100.0;
    hsv_scaled.h = hsv.h;

    if ( hsv_scaled.h < hsvFilters_[channel].hsv[IDX_h_orange] ||  hsv_scaled.h > hsvFilters_[channel].hsv[IDX_H_orange] ||
        hsv_scaled.s < hsvFilters_[channel].hsv[IDX_s_orange] ||  hsv_scaled.s > hsvFilters_[channel].hsv[IDX_S_orange] ||
        hsv_scaled.v < hsvFilters_[channel].hsv[IDX_v_orange] ||  hsv_scaled.v > hsvFilters_[channel].hsv[IDX_V_orange])
        {
      return false;
        }
        
    else
    {
      return true;
    }
  }

  void initHSV()
  {
    for (int i = 0; i < MAX_CHANNELS; i++)
    {
      hsvFilters_[i].hsv[IDX_h_green] = 30.0;
      hsvFilters_[i].hsv[IDX_s_green] = 60;
      hsvFilters_[i].hsv[IDX_v_green] = 50.0;
      hsvFilters_[i].hsv[IDX_S_green] = 179;
      hsvFilters_[i].hsv[IDX_V_green] = 203.0;
      hsvFilters_[i].hsv[IDX_H_green] = 82.0;

      hsvFilters_[i].hsv[IDX_h_orange] = 25.0;
      hsvFilters_[i].hsv[IDX_s_orange] = 100.0;
      hsvFilters_[i].hsv[IDX_v_orange] = 20.0;
      hsvFilters_[i].hsv[IDX_S_orange] = 255.0;
      hsvFilters_[i].hsv[IDX_V_orange] = 255.0;
      hsvFilters_[i].hsv[IDX_H_orange] = 25.0;

      hsvFilters_[i].hsv[IDX_h_red] = 105.0;
      hsvFilters_[i].hsv[IDX_s_red] = 0.0;
      hsvFilters_[i].hsv[IDX_v_red] = 70.0;
      hsvFilters_[i].hsv[IDX_S_red] = 255.0;
      hsvFilters_[i].hsv[IDX_V_red] = 121.0;
      hsvFilters_[i].hsv[IDX_H_red] = 125.0;

      hsvFilters_[i].hsv[IDX_h_blue] = 0.0;
      hsvFilters_[i].hsv[IDX_s_blue] = 176.0;
      hsvFilters_[i].hsv[IDX_v_blue] = 70.0;
      hsvFilters_[i].hsv[IDX_S_blue] = 223.0;
      hsvFilters_[i].hsv[IDX_V_blue] = 93.0;
      hsvFilters_[i].hsv[IDX_H_blue] = 153.0;

      hsvFilters_[i].hsv[IDX_h_yellow] = 90.0;
      hsvFilters_[i].hsv[IDX_s_yellow] = 0.0;
      hsvFilters_[i].hsv[IDX_v_yellow] = 70.0;
      hsvFilters_[i].hsv[IDX_S_yellow] = 255.0;
      hsvFilters_[i].hsv[IDX_V_yellow] = 192.0;
      hsvFilters_[i].hsv[IDX_H_yellow] = 125.0;
    }
  }
  
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr image_sub_3D_; 

  static const int MAX_CHANNELS = 3;
  int detected_ = 0;
   
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

