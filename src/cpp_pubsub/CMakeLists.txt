cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

# Dacă nu e C++ 14 pe sistem, setează-l
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Include output warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# DEPENDINȚELE ROS2 ȘI OpenCV
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# EXECUTABIL PUBLISHER (camera)
add_executable(image_publisher src/minimal_publisher.cpp)
ament_target_dependencies(image_publisher
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

# EXECUTABIL SUBSCRIBER (afișare)
add_executable(image_subscriber src/minimal_subscriber.cpp)
ament_target_dependencies(image_subscriber
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

# INSTALARE EXECUTABILE
install(TARGETS
  image_publisher
  image_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
