#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/int32.hpp>

#include <fstream>
#include <algorithm>
#include <filesystem>
#include <vector>
#include <string>
#include <cmath>

#include <ryusei/common/defs.hpp>

using namespace project_ryusei;

namespace cone_detector
{

class LogPosePublisher : public rclcpp::Node
{
public:

  LogPosePublisher(const rclcpp::NodeOptions & options)
  : Node("log_pose_pub", options)
  {
    RCLCPP_INFO(this->get_logger(), "LogPosePublisher started.");
    folder_path_ =
      this->declare_parameter<std::string>("folder_path", "");

    if (folder_path_.empty()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "No folder path provided.");
      return;
    }

    load_json_files();

    if (json_files_.empty()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "No json files found.");
      return;
    }


    publisher_ =
      this->create_publisher<geometry_msgs::msg::Pose>(
        "/pose",
        10);


    index_subscriber_ =
      this->create_subscription<std_msgs::msg::Int32>(
        "/log_sync/index",
        10,
        std::bind(
          &LogPosePublisher::on_index_received,
          this,
          std::placeholders::_1));
  }


private:


void load_json_files()
{
  for (const auto & entry :
       std::filesystem::directory_iterator(folder_path_))
  {
    if (entry.is_regular_file() &&
        entry.path().extension() == ".json")
    {
      json_files_.push_back(entry.path().string());
    }
  }

  std::sort(
    json_files_.begin(),
    json_files_.end());
}



float getValue(const std::string & line)
{
  size_t pos = line.find(":");

  if (pos == std::string::npos)
    return 0.0f;


  std::string value =
    line.substr(pos + 1);


  value.erase(
    std::remove(
      value.begin(),
      value.end(),
      '"'),
    value.end());


  value.erase(
    std::remove(
      value.begin(),
      value.end(),
      ','),
    value.end());


  return std::stof(value);
}



void readJSONPose(
  const std::string & filename,
  Pose3D & pose)
{
  std::ifstream f(filename);

  if (!f)
  {
    throw std::runtime_error(
      "Cannot open JSON: " + filename);
  }


  std::string line;
  bool inPoseBlock = false;


  while (std::getline(f, line))
  {

    if (!inPoseBlock &&
        line.find("\"Pose\"") != std::string::npos)
    {
      inPoseBlock = true;
      continue;
    }


    if (inPoseBlock)
    {

      if (line.find("}") != std::string::npos)
        break;


      if (line.find("\"X\"") != std::string::npos)
        pose.x = getValue(line);


      else if (line.find("\"Y\"") != std::string::npos)
        pose.y = getValue(line);


      else if (line.find("\"Z\"") != std::string::npos)
        pose.z = getValue(line);


      else if (line.find("\"Roll\"") != std::string::npos)
        pose.roll = getValue(line);


      else if (line.find("\"Pitch\"") != std::string::npos)
        pose.pitch = getValue(line);


      else if (line.find("\"Yaw\"") != std::string::npos ||
               line.find("\"yaw\"") != std::string::npos)
        pose.yaw = getValue(line);
    }
  }
}



geometry_msgs::msg::Pose
pose3DToROS(const Pose3D &pose)
{
  geometry_msgs::msg::Pose msg;

  msg.position.x = pose.x;
  msg.position.y = pose.y;
  msg.position.z = pose.z;

  // degree → radian
  double roll  = pose.roll  * M_PI / 180.0;
  double pitch = pose.pitch * M_PI / 180.0;
  double yaw   = pose.yaw   * M_PI / 180.0;

  // roll pitch yaw → quaternion
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  msg.orientation.w =
      cr * cp * cy +
      sr * sp * sy;

  msg.orientation.x =
      sr * cp * cy -
      cr * sp * sy;

  msg.orientation.y =
      cr * sp * cy +
      sr * cp * sy;

  msg.orientation.z =
      cr * cp * sy -
      sr * sp * cy;

  return msg;
}


void on_index_received(
  const std_msgs::msg::Int32::SharedPtr msg)
{

  int idx = msg->data;


  if (idx < 0 ||
      static_cast<size_t>(idx) >= json_files_.size())
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Invalid index: %d",
      idx);
    return;
  }


  Pose3D pose{};


  try
  {
    readJSONPose(
      json_files_[idx],
      pose);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "%s",
      e.what());
    return;
  }

  auto ros_pose =
    pose3DToROS(pose);

  publisher_->publish(ros_pose);
}



private:

std::string folder_path_;

std::vector<std::string> json_files_;

rclcpp::Publisher<
  geometry_msgs::msg::Pose
>::SharedPtr publisher_;

rclcpp::Subscription<
  std_msgs::msg::Int32
>::SharedPtr index_subscriber_;

};


} // namespace cone_detector


#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(cone_detector::LogPosePublisher)