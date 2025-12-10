cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Publisher (camera)
add_executable(image_publisher src/minimal_publisher.cpp)
ament_target_dependencies(image_publisher
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

# Subscriber (face detector)
add_executable(image_subscriber src/minimal_subscriber.cpp)
ament_target_dependencies(image_subscriber
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

install(TARGETS
  image_publisher
  image_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
