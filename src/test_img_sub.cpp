#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <functional>

#include <ryusei/common/math.hpp>

/* poseを変えてまだ修正してないから使えない */

/*** 名前空間を省略して利用できるように宣言 ***/
using sensor_msgs::msg::Image;
using geometry_msgs::msg::PoseStamped;
using namespace project_ryusei;



namespace cone_detector
{

class ImageSubscriber : public rclcpp::Node
{
public:

  ImageSubscriber(rclcpp::NodeOptions options);
  ~ImageSubscriber();


private:

  void onImageSubscribed(
    Image::SharedPtr img);


  void onPoseSubscribed(
    PoseStamped::SharedPtr pose);


  rclcpp::Subscription<Image>::SharedPtr sub_img_;

  rclcpp::Subscription<PoseStamped>::SharedPtr sub_pose_;

};



/*** コンストラクタ ***/
ImageSubscriber::ImageSubscriber(
  rclcpp::NodeOptions options)
: Node("test_img_sub", options)
{

  using std::placeholders::_1;


  /***************
   * Image subscriber
   ***************/
  sub_img_ =
    this->create_subscription<Image>(
      "/cone_image",
      10,
      std::bind(
        &ImageSubscriber::onImageSubscribed,
        this,
        _1));



  /***************
   * Pose subscriber
   ***************/
  sub_pose_ =
    this->create_subscription<PoseStamped>(
      "/pose",
      10,
      std::bind(
        &ImageSubscriber::onPoseSubscribed,
        this,
        _1));

}



/*** デストラクタ ***/
ImageSubscriber::~ImageSubscriber()
{

}



/*** 画像callback ***/
void ImageSubscriber::onImageSubscribed(
  Image::SharedPtr img)
{

  auto cv_img =
    cv_bridge::toCvShare(
      img,
      img->encoding);


  if(cv_img->image.empty())
    return;


  cv::imshow(
    "Test Image",
    cv_img->image);


  cv::waitKey(1);

}



/*** Pose callback ***/
void ImageSubscriber::onPoseSubscribed(
  PoseStamped::SharedPtr pose)
{

  double roll;
  double pitch;
  double yaw;


  /*
   * ROS Quaternion:
   * x,y,z,w
   *
   * quaternionToEuler:
   * w,x,y,z
   *
   * の順番なので注意
   */
  quaternionToEuler(
    pose->pose.orientation.w,
    pose->pose.orientation.x,
    pose->pose.orientation.y,
    pose->pose.orientation.z,
    roll,
    pitch,
    yaw);



  RCLCPP_INFO(
    this->get_logger(),

    "Pose received:"
    "\n position:"
    " x=%f"
    " y=%f"
    " z=%f"
    "\n orientation:"
    " roll=%f"
    " pitch=%f"
    " yaw=%f",

    pose->pose.position.x,
    pose->pose.position.y,
    pose->pose.position.z,

    roll,
    pitch,
    yaw);

}


} // namespace cone_detector



/*** ImageSubscriberクラスをコンポーネントとして登録 ***/

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(
  cone_detector::ImageSubscriber)