#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class CameraNode : public rclcpp::Node
{
public:
  CameraNode() : Node("camera_node")
  {
    pub_ = create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

    cap_.open(0, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(get_logger(), "Nu pot deschide camera (index 0).");
    } else {
      RCLCPP_INFO(get_logger(), "Camera deschisa OK.");
    }

    timer_ = create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&CameraNode::tick, this)
    );
  }

private:
  void tick()
  {
    if (!cap_.isOpened()) return;

    cv::Mat frame;
    cap_ >> frame;
    if (frame.empty()) return;

    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    pub_->publish(*msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture cap_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
  return 0;
}
