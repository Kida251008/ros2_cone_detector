#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <filesystem>

namespace cone_detector {

class LogImgPublisher : public rclcpp::Node
{
public:
  LogImgPublisher(const rclcpp::NodeOptions & options)
  : Node("log_img_pub", options), current_image_idx_(0)
  {
    folder_path_ = this->declare_parameter<std::string>("folder_path", "");
    if (folder_path_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No folder path provided in YAML file.");
      return;
    }

    load_image_files();
    if (image_files_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No image files found in the specified folder.");
      return;
    }

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera1/image", 10);
    index_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/log_sync/index", 10);

    publish_image();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&LogImgPublisher::timer_callback, this));
  }

private:
  void load_image_files()
  {
    for (const auto& entry : std::filesystem::directory_iterator(folder_path_)) {
      if (entry.is_regular_file()) {
        std::string file_path = entry.path().string();
        if (is_image_file(file_path)) {
          image_files_.push_back(file_path);
        }
      }
    }
    std::sort(image_files_.begin(), image_files_.end());
  }

  bool is_image_file(const std::string& file_path)
  {
    std::vector<std::string> valid_extensions = {".jpg", ".jpeg", ".png", ".bmp", ".tiff"};
    std::string extension = file_path.substr(file_path.find_last_of("."));
    for (const auto& ext : valid_extensions) {
      if (extension == ext) return true;
    }
    return false;
  }

  void cvImage2ROSImage(const cv::Mat &src, sensor_msgs::msg::Image &dst)
  {
    dst.height = src.rows;
    dst.width = src.cols;
    dst.encoding = (src.type() == CV_8UC1) ? "mono8" : "bgr8";
    dst.step = static_cast<uint32_t>(src.step);
    size_t size = src.step * src.rows;
    dst.data.resize(size);
    memcpy(&dst.data[0], src.data, size);
    dst.header.frame_id = "camera";
    dst.header.stamp = this->now();
  }

  void publish_image()
  {
    if (current_image_idx_ >= image_files_.size()) return;

    cv::Mat image = cv::imread(image_files_[current_image_idx_]);
    if (image.empty()) return;

    // cv::imshow("Image Display", image);
    // cv::waitKey(30);

    sensor_msgs::msg::Image img_msg;
    cvImage2ROSImage(image, img_msg);
    publisher_->publish(img_msg);

    std_msgs::msg::Int32 index_msg;
    index_msg.data = static_cast<int>(current_image_idx_);
    index_publisher_->publish(index_msg);
  }

  void next_image()
  { 
    current_image_idx_ = (current_image_idx_ + 1) % image_files_.size();
    publish_image();
  }
  
  void previous_image() {
    current_image_idx_ = (current_image_idx_ == 0) ? image_files_.size() - 1 : current_image_idx_ - 1;
    publish_image();
  }

  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "timer callback");
    next_image();
    // int key = cv::waitKey(30);
    // if (key == 'd') next_image();
    // else if (key == 'a') previous_image();
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr index_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string folder_path_;
  std::vector<std::string> image_files_;
  size_t current_image_idx_;
};

} // namespace cone_detector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(cone_detector::LogImgPublisher)