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

using std::placeholders::_1 ;

enum {IDX_h_red, IDX_H_red, IDX_s_red, IDX_S_red, IDX_v_red, IDX_V_red, NUM_HSV,
      IDX_h_blue, IDX_H_blue, IDX_s_blue, IDX_S_blue, IDX_v_blue, IDX_V_blue,
      IDX_h_green, IDX_H_green, IDX_s_green, IDX_S_green, IDX_v_green, IDX_V_green,
      IDX_h_yellow, IDX_H_yellow, IDX_s_yellow, IDX_S_yellow, IDX_v_yellow, IDX_V_yellow,
      IDX_h_orange, IDX_H_orange, IDX_s_orange, IDX_S_orange, IDX_v_orange, IDX_V_orange};

typedef struct
{
  float hsv[NUM_HSV];
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr hsv_subs[NUM_HSV];
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cloud_pub_;
}
HSVInfo;




class Filter : public rclcpp::Node
{

  public:

    Filter() : Node("filter")
    {

      std::cout << "init callback\n";
      image_sub_3D_ = this->create_subscription<sensor_msgs::msg::Image>("/kinect_color/image_raw", 10, std::bind(&Filter::imageCb, this, std::placeholders::_1));
      object_publisher_ = this->create_publisher<std_msgs::msg::String>("/object_discover", 10);
    }

    void imageCb(const sensor_msgs::msg::Image::SharedPtr cloud_in)
    {
 
        
      int pos_x, pos_y;

      cv_bridge::CvImagePtr cv_ptr, cv_imageout;
      cv_ptr = cv_bridge::toCvCopy(cloud_in, sensor_msgs::image_encodings::BGR8);

      cv::Mat hsv;
      cv:cvtColor(cv_ptr->image , hsv, CV_RGB2HSV);

      int height = hsv.rows;
      int width = hsv.cols;
      int step = hsv.step;
      int channels = 3;

      x_ = 0;
      y_ = 0;
      counter_ = 0;
      for ( int i=0; i < height; i++ )
      {
          for ( int j=0; j < width; j++ )
          {
              int posdata = i * step + j * channels;

              // Red
              if ((hsv.data[posdata] >= 102) && (hsv.data[posdata] <= 125)
              && (hsv.data[posdata+1]  >=0) && (hsv.data[posdata+1] <= 255)
              && (hsv.data[posdata+2]  >=70) && (hsv.data[posdata+2] <= 121))
              {
                  x_ += j;
                  y_ += i;
                  counter_++;
                  detected_= 1; 
                  auto message = std_msgs::msg::String();
                  message.data = "fire_extinguisher";
                  object_publisher_->publish(message);
                  
              }
              // Green 
              if ((hsv.data[posdata] >= 30) && (hsv.data[posdata] <= 82)
              && (hsv.data[posdata+1]  >=60) && (hsv.data[posdata+1] <= 189)
              && (hsv.data[posdata+2]  >=50) && (hsv.data[posdata+2] <= 203))
              {
                  x_ += j;
                  y_ += i;
                  counter_++;
                  detected_= 2; 
                  auto message = std_msgs::msg::String();
                  message.data = "bowl";
                  object_publisher_->publish(message);
              }
              // Yellow
              if ((hsv.data[posdata] >= 90) && (hsv.data[posdata] <= 92)
              && (hsv.data[posdata+1]  >=0) && (hsv.data[posdata+1] <= 255)
              && (hsv.data[posdata+2]  >=70) && (hsv.data[posdata+2] <= 142))
              {
                  x_ += j;
                  y_ += i;
                  counter_++;
                  detected_= 3; 
                  auto message = std_msgs::msg::String();
                  message.data = "rubber_duck";
                  object_publisher_->publish(message);
              }
              // Blue
              if ((hsv.data[posdata] >= 0) && (hsv.data[posdata] <= 153)
              && (hsv.data[posdata+1]  >= 176) && (hsv.data[posdata+1] <= 223)
              && (hsv.data[posdata+2]  >= 70) && (hsv.data[posdata+2] <= 92))
              {
                  x_ += j;
                  y_ += i;
                  counter_++;
                  detected_= 4; 
                  auto message = std_msgs::msg::String();
                  message.data = "computer";
                  object_publisher_->publish(message);
              }
              // Orange
              if ((hsv.data[posdata] >= 10) && (hsv.data[posdata] <= 25)
                && (hsv.data[posdata+1]  >= 100) && (hsv.data[posdata+1] <= 255)
                && (hsv.data[posdata+2]  >= 20) && (hsv.data[posdata+2] <= 255))
                {
                    x_ += j;
                    y_ += i;
                    counter_++;
                    detected_= 5; 
                    auto message = std_msgs::msg::String();
                    message.data = "orange";
                    object_publisher_->publish(message);
                }
           }
      }
    
      std::cout << "object deteted: " << detected_ << "\n";
      if (counter_ > 100)
      {
        int pos_x = x_/counter_;
        int pos_y = y_/counter_; 
      }
  
    }


private:

  private:

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_3D_; 

    static const int MAX_CHANNELS = 3;
    int detected_= 0; 
    int counter_ = 0;
      
    int x_ = 0;
    int y_ = 0;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr object_publisher_;

};  
