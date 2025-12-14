#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class FaceRecognizerNode : public rclcpp::Node
{
public:
  FaceRecognizerNode() : Node("face_recognizer")
  {
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      "face/face_crop", 10,
      std::bind(&FaceRecognizerNode::cb, this, std::placeholders::_1)
    );

    pub_debug_ = create_publisher<sensor_msgs::msg::Image>("face_recognizer/image_debug", 10);

    RCLCPP_INFO(get_logger(), "Face recognizer pornit (debug only)");
  }

private:
  void cb(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat face;
    try {
      face = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "cv_bridge convert error (face_recognizer)");
      return;
    }
    if (face.empty()) return;

    cv::Mat dbg = face.clone();
    cv::putText(dbg, "FACE CROP RECEIVED", {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0,255,0}, 2);

    auto out = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", dbg).toImageMsg();
    pub_debug_->publish(*out);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FaceRecognizerNode>());
  rclcpp::shutdown();
  return 0;
}
