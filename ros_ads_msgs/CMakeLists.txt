cmake_minimum_required(VERSION 3.5)
project(ros_ads_msgs)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE debug)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/Ads.msg" 
  "msg/State.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files} DEPENDENCIES std_msgs)

ament_package()