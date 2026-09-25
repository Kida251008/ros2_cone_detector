#include <cone_detection/cone_detection.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <memory>

using sensor_msgs::msg::Image;

using namespace std;
using namespace cv;
cone_detector::ConeDetector cone_detector_;
cone_detector::ObstacleDetector obstacle_detector_;

namespace cone_detector
{

Recognition::Recognition(rclcpp::NodeOptions options) : Node("cone_detector", options)
{
  initTopic();
  obstacle_detector_.init(init_path);
  thread_ = std::make_unique<thread>(&Recognition::run, this);
  RCLCPP_INFO(this->get_logger(), "Recognition node initialized.");
}

Recognition::~Recognition()
{
  running_ = false;

  if (thread_ && thread_->joinable()) {
    thread_->join();
  }
  thread_.reset();
}

void Recognition::initTopic()
{
  using std::placeholders::_1;
  /***  サブスクライバ  ***/
  sub_img_ = this->create_subscription<Image>("/camera1/image", 10, std::bind(&Recognition::onImageSubscribed, this, _1));
  sub_pcd_ = this->create_subscription<PointCloud>("/lidar/points", 10, std::bind(&Recognition::onPointcloudSubscribed, this, _1));
  sub_pose_ = this->create_subscription<Pose>("/locator/pose", 10, std::bind(&Recognition::onLocatorPoseSubscribed, this, _1));

  /***  パブリッシャ  ***/
  pub_result_image_ = this->create_publisher<Image>("/cone_image", 10);
  pub_pointcloud_ = this->create_publisher<PointCloud>("/cone_point", 10);
}

void Recognition::onImageSubscribed(const Image::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_image_ = msg;
}

void Recognition::onPointcloudSubscribed(const PointCloud::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_pcd_ = msg;
}

void Recognition::onLocatorPoseSubscribed(Pose::SharedPtr pose)
{
  lock_guard<mutex> lock(mutex_pose_);
  pose_ptr_ = pose;
}

void Recognition::updatePose(const Pose::SharedPtr pose_ptr, Pose3D& pose)
{
  lock_guard<mutex> lock(mutex_pose_);
  if(pose_ptr_ == nullptr) return;
  pose.x = pose_ptr->position.x;
  pose.y = pose_ptr->position.y;
  pose.z = pose_ptr->position.z;
  quaternionToEuler(pose_ptr->orientation.w, pose_ptr->orientation.x, pose_ptr->orientation.y, pose_ptr->orientation.z, pose.roll, pose.pitch, pose.yaw);  
}

void Recognition::convertPointCloudToLidarData(const PointCloud::SharedPtr& pointcloud, std::vector<LidarData>& lidar_data)
{
  lidar_data.clear();

  const auto& points = pointcloud->points;
  const auto& channels = pointcloud->channels;

  int num_points = points.size();
  if (channels.size() < 2 || channels[0].values.size() != num_points || channels[1].values.size() != num_points) {
    RCLCPP_WARN(this->get_logger(), "Invalid channel size in PointCloud");
    return;
  }

  for (size_t i = 0; i < num_points; ++i) {
    const auto& pt = points[i];
    LidarData ld;
    ld.x = pt.x;
    ld.y = pt.y;
    ld.z = pt.z;
    ld.range = channels[0].values[i];
    ld.reflectivity = channels[1].values[i];
    lidar_data.push_back(ld);
  }
}

void Recognition::ROSImageToCVImage(const sensor_msgs::msg::Image &src, cv::Mat &dst)
{
  int cv_type;
  if (src.encoding == "mono8") {
    cv_type = CV_8UC1;
  } else if (src.encoding == "bgr8") {
    cv_type = CV_8UC3;
  } else if (src.encoding == "mono16") {
    cv_type = CV_16UC1;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unsupported image encoding: %s", src.encoding.c_str());
    return;
  }
  dst = cv::Mat(src.height, src.width, cv_type, const_cast<unsigned char*>(src.data.data()), src.step).clone();
}

void Recognition::cvImageToROSImage(const cv::Mat &src, Image &dst)
{
  dst.height = src.rows;
  dst.width = src.cols;
  if(src.type() == CV_8UC1) dst.encoding = "mono8";
  else if(src.type() == CV_8UC3) dst.encoding = "bgr8";
  dst.step = (uint32_t)(src.step);
  size_t size = src.step * src.rows;
  dst.data.resize(size);
  memcpy(&dst.data[0], src.data, size);
  dst.header.frame_id="img";
  dst.header.stamp = this->now();
}

void Recognition::publishResultImage(const cv::Mat &camera_img)
{
  /***  ROS2 Imageメッセージを作成  ***/
  auto ros_img = std::make_unique<Image>();
  /***  cv::MatをROS2 Imageに変換  ***/
  cvImageToROSImage(camera_img, *ros_img);
  /***  ヘッダー情報を設定  ***/
  ros_img->header.frame_id = "camera";
  ros_img->header.stamp = image_stamp_;
  /***  パブリッシュ  ***/
  pub_result_image_->publish(std::move(ros_img));
}

void Recognition::publishPointCloud(const std::vector<LidarData>& lidar_data)
{
  auto pointcloud = std::make_shared<PointCloud>();

  const size_t num_points = lidar_data.size();

  // PointCloudの点を確保
  pointcloud->points.resize(num_points);

  // channelを2つ用意
  // channel[0] = range
  // channel[1] = reflectivity
  pointcloud->channels.resize(2);

  pointcloud->channels[0].name = "range";
  pointcloud->channels[1].name = "reflectivity";

  pointcloud->channels[0].values.resize(num_points);
  pointcloud->channels[1].values.resize(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    const auto& ld = lidar_data[i];

    // XYZ
    pointcloud->points[i].x = ld.x;
    pointcloud->points[i].y = ld.y;
    pointcloud->points[i].z = ld.z;

    // Channel
    pointcloud->channels[0].values[i] = ld.range;
    pointcloud->channels[1].values[i] = ld.reflectivity;
  }

  pub_pointcloud_->publish(*pointcloud);
}


void Recognition::run()
{
  rclcpp::Rate loop(20);
  while (rclcpp::ok() && running_) {
    /***  カメラ画像も点群もどちらも受信して初めて処理を行う  ***/
    if (!latest_image_ || latest_pcd_ == nullptr) {
    // if (!latest_image_) {
      loop.sleep();
      continue;
    }
    ROSImageToCVImage(*latest_image_, cone_detector_.src_camera_img); /* ROS ImageをOpenCV Matに変換 */
    convertPointCloudToLidarData(latest_pcd_, cone_detector_.src_points); /* 点群変換 */
    updatePose(pose_ptr_, cone_detector_.current_pose);
    cone_detector_.loop_main(); /* メイン処理 */
    publishResultImage(cone_detector_.camera_img); /* 結果画像をパブリッシュ */
    publishPointCloud(cone_detector_.cone_points_obs_);
    imshow("cone_img", cone_detector_.camera_img);
    cv::waitKey(1);
    obstacle_detector_.main(cone_detector_.local_map_);
    /***  状態クリア（連続処理を避けるため）  ***/
    latest_pcd_ = nullptr;
    latest_image_ = nullptr;
    loop.sleep();
  }
}

} /* namespace cone_detector */

/*** Recognitionクラスをコンポーネントとして登録 ***/
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cone_detector::Recognition)