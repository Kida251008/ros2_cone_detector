#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

/*** 名前空間を省略して利用できるように宣言 ***/
using sensor_msgs::msg::Image;

namespace cone_detector
{

class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber(rclcpp::NodeOptions options);
  ~ImageSubscriber();
private:
  void onImageSubscribed(Image::SharedPtr img);
  rclcpp::Subscription<Image>::SharedPtr sub_img_;
};

/*** コンストラクタ ***/
ImageSubscriber::ImageSubscriber(rclcpp::NodeOptions options) : Node("test_img_sub", options)
{
  using std::placeholders::_1;
  /*** サブスクライバの初期化 ***/
  sub_img_ = this->create_subscription<Image>("/cone_image", 10, std::bind(&ImageSubscriber::onImageSubscribed, this, _1));
}

/*** デストラクタ ***/
ImageSubscriber::~ImageSubscriber()
{

}

/*** 画像をサブスクライブすると呼び出されるコールバック関数 ***/
void ImageSubscriber::onImageSubscribed(Image::SharedPtr img)
{
  /*** ROS2のImage型 -> OpenCVのMat型への変換 ***/
  auto cv_img = cv_bridge::toCvShare(img, img->encoding);
  /*** Mat型が空の場合コールバック関数を抜ける ***/
  if(cv_img->image.empty()) return;
  /*** 画像の描画 ***/
  cv::imshow("Test Image", cv_img->image);
  cv::waitKey(1);
}
}
/*** ImageSubscriberクラスをコンポーネントとして登録 ***/
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cone_detector::ImageSubscriber)