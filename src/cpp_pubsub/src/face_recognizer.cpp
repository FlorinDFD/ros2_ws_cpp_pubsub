#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>

class FaceRecognizerNode : public rclcpp::Node
{
public:
    FaceRecognizerNode()
    : Node("face_recognizer")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "face/face_crop",
            10,
            std::bind(&FaceRecognizerNode::image_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Face recognizer pornit!");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat face;

        try {
            face = cv_bridge::toCvCopy(msg, "bgr8")->image;
        }
        catch (const cv_bridge::Exception & e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
            return;
        }

        if (face.empty()) {
            RCLCPP_WARN(this->get_logger(), "Imagine goală primită în face_recognizer!");
            return;
        }

        // Inițializăm o singură dată fereastra
        static bool first_time = true;
        if (first_time) {
            cv::namedWindow("Face Recognizer Input", cv::WINDOW_NORMAL);
            cv::resizeWindow("Face Recognizer Input", 500, 500);  // Dimensiune FIXĂ
            first_time = false;
        }

        // Scalăm imaginea la 500x500
        cv::Mat resized;
        cv::resize(face, resized, cv::Size(500, 500));

        cv::imshow("Face Recognizer Input", resized);
        cv::waitKey(1);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Face Recognizer: crop primit!"
        );
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FaceRecognizerNode>());
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
