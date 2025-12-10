#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>

class CameraPublisher : public rclcpp::Node
{
public:
  CameraPublisher() : Node("camera_publisher")
  {
    // Publisher de imagini
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

    // Deschidem camera laptopului (0 sau 1 depinde de sistem)
    cap_.open(0);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Camera could not be opened!");
      return;
    }

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(30),
      std::bind(&CameraPublisher::timer_callback, this)
    );
  }

private:
  void timer_callback()
  {
    cv::Mat frame;
    cap_ >> frame;

    if (frame.empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty frame!");
      return;
    }

    // Conversie OpenCV → ROS Image
    auto msg = cv_bridge::CvImage(
      std_msgs::msg::Header(),
      "bgr8",
      frame
    ).toImageMsg();

    publisher_->publish(*msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture cap_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraPublisher>());
  rclcpp::shutdown();
  return 0;
}
