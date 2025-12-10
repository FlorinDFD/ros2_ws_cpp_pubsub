#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>
#include "std_msgs/msg/bool.hpp"

class KinectNode : public rclcpp::Node
{
public:
    KinectNode() : Node("kinect_node")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw",
            10,
            std::bind(&KinectNode::image_callback, this, std::placeholders::_1));

        face_crop_pub_ = this->create_publisher<sensor_msgs::msg::Image>("face/face_crop", 10);
        face_detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("face/detected", 10);

        // Load Haar Cascade
        if (!face_cascade_.load("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml")) {
            RCLCPP_ERROR(this->get_logger(), "Nu pot încărca Haar cascade!");
        }

        RCLCPP_INFO(this->get_logger(), "Kinect node pornit!");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

        std::vector<cv::Rect> faces;
        face_cascade_.detectMultiScale(frame, faces, 1.1, 3);

        if (faces.empty()) {
            return;
        }

        // Luăm prima față
        cv::Rect face_rect = faces[0];

        // Trimitem semnal de detectare
        auto detected_msg = std_msgs::msg::Bool();
        detected_msg.data = true;
        face_detected_pub_->publish(detected_msg);

        // Crop face
        cv::Mat face_crop = frame(face_rect);

        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();

        auto face_msg = cv_bridge::CvImage(header, "bgr8", face_crop).toImageMsg();
        face_crop_pub_->publish(*face_msg);

        RCLCPP_INFO(this->get_logger(), "Trimis imagine crop-uită!");
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr face_detected_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr face_crop_pub_;
    cv::CascadeClassifier face_cascade_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinectNode>());
    rclcpp::shutdown();
    return 0;
}
