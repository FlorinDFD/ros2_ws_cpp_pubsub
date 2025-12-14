cmake_minimum_required(VERSION 3.8)
project(gui_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

add_executable(gui_node src/gui_node.cpp)

target_link_libraries(gui_node
  Qt5::Widgets
  ${OpenCV_LIBS}
)

ament_target_dependencies(gui_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

install(TARGETS gui_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
