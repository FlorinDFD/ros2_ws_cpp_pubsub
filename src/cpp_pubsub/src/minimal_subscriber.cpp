#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>

class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber() : Node("image_subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera/image_raw",
      10,
      std::bind(&ImageSubscriber::callback, this, std::placeholders::_1)
    );
  }

private:
  void callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::imshow("Camera View", frame);
    cv::waitKey(1);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSubscriber>());
  rclcpp::shutdown();
  return 0;
}
