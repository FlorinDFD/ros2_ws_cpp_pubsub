#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"

#include <opencv2/opencv.hpp>

class FaceDetectorNode : public rclcpp::Node
{
public:
  FaceDetectorNode()
  : Node("face_detector")
  {
    // Parametru pentru path-ul classifier-ului de față
    std::string cascade_path;
    this->declare_parameter<std::string>(
      "face_cascade_path",
      "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"
    );
    this->get_parameter("face_cascade_path", cascade_path);

    if (!face_cascade_.load(cascade_path)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Nu pot încărca classifier-ul pentru față de la: '%s'",
        cascade_path.c_str()
      );
    } else {
      RCLCPP_INFO(this->get_logger(), "Am încărcat classifier-ul de față din: '%s'", cascade_path.c_str());
    }

    // Ne abonăm la imaginile publicate de nodul tău (camera/image_raw)
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera/image_raw",
      10,
      std::bind(&FaceDetectorNode::image_callback, this, std::placeholders::_1)
    );
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // ROS Image -> OpenCV Mat
    cv::Mat frame;
    try {
      frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    if (frame.empty()) {
      RCLCPP_WARN(this->get_logger(), "Cadru gol primit!");
      return;
    }

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(gray, gray);

    std::vector<cv::Rect> faces;

    if (!face_cascade_.empty()) {
      face_cascade_.detectMultiScale(
        gray, faces,
        1.1,    // scale factor
        3,      // min neighbors
        0,
        cv::Size(80, 80)  // mărimea minimă a feței
      );
    }

    for (const auto & face : faces) {
      // Desenăm un dreptunghi pe față
      cv::rectangle(frame, face, cv::Scalar(0, 255, 0), 2);

      int x = face.x;
      int y = face.y;
      int w = face.width;
      int h = face.height;

      // Puncte "Kinect-like" aproximative din bounding box:
      cv::Point left_eye( x + w * 3 / 10, y + h * 3 / 10 );
      cv::Point right_eye( x + w * 7 / 10, y + h * 3 / 10 );
      cv::Point nose( x + w / 2,          y + h * 5 / 10 );
      cv::Point mouth_left(  x + w * 3 / 10, y + h * 7 / 10 );
      cv::Point mouth_right( x + w * 7 / 10, y + h * 7 / 10 );
      cv::Point chin( x + w / 2,          y + h * 9 / 10 );

      std::vector<cv::Point> landmarks = {
        left_eye, right_eye, nose, mouth_left, mouth_right, chin
      };

      for (const auto & p : landmarks) {
        cv::circle(frame, p, 3, cv::Scalar(0, 0, 255), cv::FILLED);
      }

      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Față detectată la x=%d, y=%d, w=%d, h=%d",
        face.x, face.y, face.width, face.height
      );
    }

    cv::imshow("Face detector (image_subscriber)", frame);
    cv::waitKey(1);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  cv::CascadeClassifier face_cascade_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FaceDetectorNode>());
  cv::destroyAllWindows();
  rclcpp::shutdown();
  return 0;
}
