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

add_executable(image_publisher src/minimal_publisher.cpp)
ament_target_dependencies(image_publisher rclcpp sensor_msgs cv_bridge)
target_link_libraries(image_publisher ${OpenCV_LIBS})

add_executable(image_subscriber src/minimal_subscriber.cpp)
ament_target_dependencies(image_subscriber rclcpp sensor_msgs cv_bridge)
target_link_libraries(image_subscriber ${OpenCV_LIBS})

add_executable(face_recognizer src/face_recognizer.cpp)
ament_target_dependencies(face_recognizer rclcpp sensor_msgs cv_bridge)
target_link_libraries(face_recognizer ${OpenCV_LIBS})

install(TARGETS
  image_publisher
  image_subscriber
  face_recognizer
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
