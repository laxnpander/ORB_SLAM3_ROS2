#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    declare_parameter("img_topic", "/camera/image_raw");
    std::string topic_img = get_parameter("img_topic").as_string();
    
    declare_parameter("raw", true);
    bool is_raw = get_parameter("raw").as_bool();
    
    m_SLAM = pSLAM;
    
    if (is_raw)
    {
        m_image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
            topic_img,
            10,
            [this](const sensor_msgs::msg::Image::UniquePtr &msg)
            {
                double time = Utility::StampToSec(msg->header.stamp);
                cv::Mat img = Utility::toCvMat(msg);
                m_SLAM->TrackMonocular(img, time);
            });
    }
    else
    {
        m_image_subscriber = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            topic_img,
            10,
            [this](const sensor_msgs::msg::CompressedImage::UniquePtr &msg)
            {
                double time = Utility::StampToSec(msg->header.stamp);
                cv::Mat img = Utility::toCvMat(msg);
                m_SLAM->TrackMonocular(img, time);
            });
    }
    
    std::cout << "slam changed" << std::endl;
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}
