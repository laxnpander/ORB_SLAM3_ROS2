#ifndef __UTILITY_HPP__
#define __UTILITY_HPP__

#include "rclcpp/rclcpp.hpp"

class Utility
{
public:
  static double StampToSec(builtin_interfaces::msg::Time stamp)
  {
    double seconds = stamp.sec + (stamp.nanosec * pow(10,-9));
    return seconds;
  }
  
  static cv::Mat toCvMat(const sensor_msgs::msg::Image::UniquePtr &msg)
  {
      // Convert ROS2 raw image to cv::Mat (BGR8 by default)
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
          cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception &e)
      {
          RCLCPP_ERROR(rclcpp::get_logger("func1"), "cv_bridge exception: %s", e.what());
          return cv::Mat();
      }
      return cv_ptr->image;
  }

  static cv::Mat toCvMat(const sensor_msgs::msg::CompressedImage::UniquePtr &msg)
  {
      // Decode compressed image into cv::Mat
      try
      {
          // msg->format might be "jpeg" or "png"
          cv::Mat compressedData(1, msg->data.size(), CV_8UC1, const_cast<unsigned char*>(msg->data.data()));
          return cv::imdecode(compressedData, cv::IMREAD_COLOR); // BGR by default
      }
      catch (cv::Exception &e)
      {
          RCLCPP_ERROR(rclcpp::get_logger("func2"), "OpenCV imdecode exception: %s", e.what());
          return cv::Mat();
      }
  }
};

#endif
