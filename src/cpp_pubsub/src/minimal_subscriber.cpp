#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class KinectLikeNode : public rclcpp::Node
{
public:
  KinectLikeNode() : Node("kinect_node")
  {
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      "camera/image_raw", 10,
      std::bind(&KinectLikeNode::cb, this, std::placeholders::_1)
    );

    pub_crop_  = create_publisher<sensor_msgs::msg::Image>("face/face_crop", 10);
    pub_debug_ = create_publisher<sensor_msgs::msg::Image>("kinect/image_debug", 10);

    if (!face_cascade_.load("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml")) {
      RCLCPP_ERROR(get_logger(), "Nu pot incarca Haarcascade frontalface!");
    }

    RCLCPP_INFO(get_logger(), "Kinect-like node pornit");
  }

private:
  void cb(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat frame;
    try {
      frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "cv_bridge convert error (kinect)");
      return;
    }
    if (frame.empty()) return;

    std::vector<cv::Rect> faces;
    face_cascade_.detectMultiScale(frame, faces, 1.2, 5);

    // debug = frame + dreptunghi
    cv::Mat debug = frame.clone();

    // dacă detectează mai multe fețe, alegem cea mai mare (mai stabil)
    int best_idx = -1;
    int best_area = 0;
    for (int i = 0; i < (int)faces.size(); i++) {
      int a = faces[i].width * faces[i].height;
      if (a > best_area) { best_area = a; best_idx = i; }
    }

    if (best_idx >= 0) {
      auto f = faces[best_idx];
      cv::rectangle(debug, f, cv::Scalar(0,255,0), 2);

      cv::Mat crop = frame(f).clone(); // clone important (memorie separată)
      auto crop_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", crop).toImageMsg();
      pub_crop_->publish(*crop_msg);
    }

    auto dbg_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debug).toImageMsg();
    pub_debug_->publish(*dbg_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_crop_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_;
  cv::CascadeClassifier face_cascade_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinectLikeNode>());
  rclcpp::shutdown();
  return 0;
}
