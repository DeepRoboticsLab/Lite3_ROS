cmake_minimum_required(VERSION 3.5)
project(transfer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(transfer_interfaces REQUIRED)

add_executable(jetson2app src/Jetson2App.cpp)
ament_target_dependencies(jetson2app
    transfer_interfaces
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    tf2_msgs
    tf2_ros
)

add_executable(jetson2motion src/Jetson2Motion.cpp)
ament_target_dependencies(jetson2motion
    transfer_interfaces
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    tf2_msgs
    tf2_ros
)

add_executable(sensor_checker src/SensorChecker.cpp)
ament_target_dependencies(sensor_checker
    transfer_interfaces
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    tf2_msgs
    tf2_ros
)

install(TARGETS jetson2motion jetson2app sensor_checker
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
