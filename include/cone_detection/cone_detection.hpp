#ifndef CONE_DETECTION_COMMON_INCLUDES_HPP_
#define CONE_DETECTION_COMMON_INCLUDES_HPP_

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <cv_bridge/cv_bridge.h>
#include <mutex>

// C++ includes
#include <iostream>
#include <experimental/filesystem>
#include <opencv2/opencv.hpp>
#include <ryusei/common/logger.hpp>
#include <ryusei/common/defs.hpp>
#include <ryusei/common/math.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <fstream>
#include <cstring>
#include <string.h>

#include<cone.hpp>

using sensor_msgs::msg::Image;
using namespace project_ryusei;
using namespace cv;
using namespace std;

#define DEG_TO_RAD (M_PI / 180.0)

namespace cone_detector
{
  class Recognition : public rclcpp::Node
  {
  public:
    Recognition(rclcpp::NodeOptions options);
    ~Recognition();

  private:
    std::unique_ptr<std::thread> thread_;
    /***  === Subscribers ===  ***/
    rclcpp::Subscription<Image>::SharedPtr sub_img_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr sub_pcd_;

    /***  === Publishers ===  ***/
    rclcpp::Publisher<Image>::SharedPtr pub_result_image_;
    rclcpp::Publisher<Image>::SharedPtr pub_range_image_;
    rclcpp::Publisher<Image>::SharedPtr pub_ref_image_;

    /***  === Data buffer ===  ***/
    sensor_msgs::msg::PointCloud::SharedPtr latest_pcd_;
    Image::SharedPtr latest_image_;
    rclcpp::Time image_stamp_;
    rclcpp::Time range_img_stamp_;
    rclcpp::Time ref_img_stamp_;
    rclcpp::Time pcd_stamp_;

    /***  === Mutex ===  ***/
    std::mutex data_mutex_;

    /***  === Processing ===  ***/
    void onImageSubscribed(Image::SharedPtr img);
    void onPointcloudSubscribed(const sensor_msgs::msg::PointCloud::SharedPtr msg);
    void initTopic();
    void convertPointCloudToLidarData(const sensor_msgs::msg::PointCloud::SharedPtr& pointcloud, std::vector<LidarData>& lidar_data);
    void run();
    void ROSImageToCVImage(const Image &src, cv::Mat &dst);
    void cvImageToROSImage(const cv::Mat &src, Image &dst);
    void publishResultImage(const cv::Mat &camera_img);
    void publishRangeImage(const cv::Mat &range_img);
    void publishReflectanceImage(const cv::Mat &ref_img);
    void publishSignalState(const string &signal_state);
    void SignalImagePublisher(Mat &camera_img);
  };
} /* namespace cone_detector */

#endif 