#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_msgs/msg/int32.hpp>
#include <fstream>
#include <algorithm>
#include <filesystem>
#include <vector>
#include <string>
#include <cmath>

namespace cone_detector {

  struct PointXYZI
{
  float x;
  float y;
  float z;
  float intensity;
};

struct PointXYZ
{
  float x;
  float y;
  float z;
  float intensity;
};

class LogPcdPublisher : public rclcpp::Node
{
public:
  LogPcdPublisher(const rclcpp::NodeOptions & options)
  : Node("log_pcd_pub", options)
  {
    folder_path_ = this->declare_parameter<std::string>("folder_path", "");
    if (folder_path_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No folder path provided in YAML file.");
      return;
    }

    load_pcd_files();
    if (pcd_files_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No .pcd files found in the specified folder.");
      return;
    }

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>("/lidar/points", 10);
    index_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
      "/log_sync/index", 10,
      std::bind(&LogPcdPublisher::on_index_received, this, std::placeholders::_1));
  }

private:

void load_pcd_files()
{
  for (const auto & entry : std::filesystem::directory_iterator(folder_path_)) {
    if (entry.is_regular_file() &&
        entry.path().extension() == ".pcd") {
      pcd_files_.push_back(entry.path().string());
    }
  }

  std::sort(pcd_files_.begin(), pcd_files_.end());
}

sensor_msgs::msg::PointCloud readPCDBinary(const std::string & filename)
{
  std::ifstream file(filename, std::ios::binary);

  if (!file) {
    throw std::runtime_error("Cannot open file: " + filename);
  }

  std::string line;
  size_t point_num = 0;
  bool binary_found = false;

  while (std::getline(file, line)) {

    if (line.rfind("POINTS", 0) == 0) {
      point_num = std::stoul(line.substr(7));
    }
    else if (line == "DATA binary") {
      binary_found = true;
      break;
    }
  }

  if (!binary_found) {
    throw std::runtime_error("DATA binary not found.");
  }

  struct RawPoint
  {
    float x;
    float y;
    float z;
    float intensity;
  };

  sensor_msgs::msg::PointCloud cloud;

  cloud.points.reserve(point_num);

  cloud.channels.resize(2);

  cloud.channels[0].name = "range";
  cloud.channels[0].values.reserve(point_num);

  cloud.channels[1].name = "reflectivity";
  cloud.channels[1].values.reserve(point_num);

  RawPoint p;

  for (size_t i = 0; i < point_num; i++) {

    file.read(reinterpret_cast<char *>(&p), sizeof(RawPoint));

    if (!file) {
      throw std::runtime_error("Failed to read point data.");
    }

    geometry_msgs::msg::Point32 point;

    point.x = p.x;
    point.y = p.y;
    point.z = p.z;

    cloud.points.push_back(point);

    float range =
      std::sqrt(
        p.x * p.x +
        p.y * p.y +
        p.z * p.z);

    cloud.channels[0].values.push_back(range);

    cloud.channels[1].values.push_back(p.intensity);
  }

  return cloud;
}

void on_index_received(const std_msgs::msg::Int32::SharedPtr msg)
{
  int idx = msg->data;

  if (idx < 0 ||
      static_cast<size_t>(idx) >= pcd_files_.size()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Invalid index received: %d",
      idx);
    return;
  }

  sensor_msgs::msg::PointCloud ros_msg;

  try {
    ros_msg = readPCDBinary(pcd_files_[idx]);
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    return;
  }

  ros_msg.header.frame_id = "lidar";
  ros_msg.header.stamp = this->now();

  publisher_->publish(ros_msg);
}

  std::string folder_path_;
  std::vector<std::string> pcd_files_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr index_subscriber_;
};

} // namespace crosswalk_signal

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(cone_detector::LogPcdPublisher)