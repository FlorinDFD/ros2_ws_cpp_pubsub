#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QProcess>
#include <QThread>
#include <QMetaObject>
#include <QPixmap>
#include <QImage>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <atomic>

// -------- ROS in QThread --------
class RosSpinThread : public QThread
{
public:
  explicit RosSpinThread(rclcpp::Node::SharedPtr node) : node_(node) {}

  void run() override
  {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node_);
    exec.spin();
  }

private:
  rclcpp::Node::SharedPtr node_;
};

// -------- Mat -> QImage --------
static QImage matToQImageBGR(const cv::Mat &bgr)
{
  if (bgr.empty()) return QImage();
  cv::Mat rgb;
  cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  return QImage(rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888).copy();
}

class FaceGui : public QWidget
{
public:
  FaceGui()
  {
    setWindowTitle("Face System GUI");
    resize(1200, 720);

    // UI layout
    auto *root = new QHBoxLayout(this);

    auto *leftBox = new QGroupBox("Afișare");
    auto *left = new QVBoxLayout(leftBox);

    btnShowRaw_    = new QPushButton("Show Camera RAW");
    btnShowKinect_ = new QPushButton("Show Kinect-like");
    btnShowFace_   = new QPushButton("Show Face Recognizer");
    btnHide_       = new QPushButton("Hide View");

    left->addWidget(btnShowRaw_);
    left->addWidget(btnShowKinect_);
    left->addWidget(btnShowFace_);
    left->addSpacing(12);
    left->addWidget(btnHide_);
    left->addStretch();

    auto *centerBox = new QGroupBox("Vizualizare");
    auto *center = new QVBoxLayout(centerBox);

    video_ = new QLabel("Apasă START SYSTEM, apoi Show ...");
    video_->setMinimumSize(900, 650);
    video_->setAlignment(Qt::AlignCenter);
    video_->setStyleSheet("background:#111; color:white;");
    center->addWidget(video_);

    auto *rightBox = new QGroupBox("Control");
    auto *right = new QVBoxLayout(rightBox);

    btnStart_ = new QPushButton("START SYSTEM");
    btnStop_  = new QPushButton("STOP SYSTEM");

    right->addWidget(btnStart_);
    right->addWidget(btnStop_);
    right->addStretch();

    root->addWidget(leftBox);
    root->addWidget(centerBox, 1);
    root->addWidget(rightBox);

    // ROS init
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);

    node_ = std::make_shared<rclcpp::Node>("gui_node");

    // Subs (mereu active) + afișăm doar când mode_ cere
    subRaw_ = node_->create_subscription<sensor_msgs::msg::Image>(
      "camera/image_raw", 10,
      [&](sensor_msgs::msg::Image::SharedPtr msg){
        if (mode_.load() != 1) return;
        showFromMsg(msg);
      });

    subKinect_ = node_->create_subscription<sensor_msgs::msg::Image>(
      "kinect/image_debug", 10,
      [&](sensor_msgs::msg::Image::SharedPtr msg){
        if (mode_.load() != 2) return;
        showFromMsg(msg);
      });

    subFace_ = node_->create_subscription<sensor_msgs::msg::Image>(
      "face_recognizer/image_debug", 10,
      [&](sensor_msgs::msg::Image::SharedPtr msg){
        if (mode_.load() != 3) return;
        showFromMsg(msg);
      });

    rosThread_ = new RosSpinThread(node_);
    rosThread_->start();

    launchProc_ = new QProcess(this);

    // Buttons
    connect(btnShowRaw_, &QPushButton::clicked, this, [&](){ mode_.store(1); });
    connect(btnShowKinect_, &QPushButton::clicked, this, [&](){ mode_.store(2); });
    connect(btnShowFace_, &QPushButton::clicked, this, [&](){ mode_.store(3); });

    connect(btnHide_, &QPushButton::clicked, this, [&](){
      mode_.store(0);
      video_->setPixmap(QPixmap());
      video_->setText("Afișare oprită (Hide View)");
    });

    connect(btnStart_, &QPushButton::clicked, this, &FaceGui::startSystem);
    connect(btnStop_,  &QPushButton::clicked, this, &FaceGui::stopSystem);
  }

  ~FaceGui()
  {
    stopSystem();

    if (rosThread_) {
      rosThread_->quit();
      rosThread_->wait();
      delete rosThread_;
    }
    rclcpp::shutdown();
  }

private:
  // UI
  QLabel *video_;
  QPushButton *btnShowRaw_;
  QPushButton *btnShowKinect_;
  QPushButton *btnShowFace_;
  QPushButton *btnHide_;
  QPushButton *btnStart_;
  QPushButton *btnStop_;

  // ROS
  rclcpp::Node::SharedPtr node_;
  RosSpinThread *rosThread_{nullptr};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subRaw_, subKinect_, subFace_;

  // Process
  QProcess *launchProc_{nullptr};

  // mode: 0 none, 1 raw, 2 kinect, 3 face
  std::atomic<int> mode_{0};

  void showFromMsg(const sensor_msgs::msg::Image::SharedPtr &msg)
  {
    cv::Mat bgr;
    try {
      bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (...) {
      return;
    }
    if (bgr.empty()) return;

    QImage img = matToQImageBGR(bgr);

    QMetaObject::invokeMethod(this, [this, img](){
      QPixmap px = QPixmap::fromImage(img).scaled(
        video_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation
      );
      video_->setPixmap(px);
    });
  }

  void startSystem()
  {
    if (launchProc_->state() == QProcess::Running) return;

    // IMPORTANT: folosim bash -lc ca să respecte "source"
    launchProc_->start("bash", QStringList()
      << "-lc"
      << "source /opt/ros/jazzy/setup.bash && "
         "source ~/ros2_ws/install/setup.bash && "
         "ros2 launch cpp_pubsub face_system.launch.py"
    );

    video_->setText("Sistem pornit. Apasă Show ...");
  }

  void stopSystem()
  {
    if (launchProc_ && launchProc_->state() == QProcess::Running) {
      launchProc_->terminate();
      launchProc_->waitForFinished(2000);
    }

    // Oprește orice procese cpp_pubsub rămase
    QProcess::execute("bash", {"-lc", "pkill -f cpp_pubsub || true"});

    mode_.store(0);
    video_->setPixmap(QPixmap());
    video_->setText("Sistem oprit.");
  }
};

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);
  FaceGui gui;
  gui.show(); // QWidget::show()
  return app.exec();
}
