cmake_minimum_required(VERSION 2.8.3)
project(message_transformer)

SET(CMAKE_CXX_FLAGS "-std=c++11 -g -O3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE Release)
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf
  message_generation
)
catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  ${PCL_OPENMP_PACKAGES}
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(qnx2ros
  src/qnx2ros.cpp
  src/moving_average.cpp
)
target_link_libraries(qnx2ros
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(nx2app
  src/nx2app.cpp
  src/moving_average.cpp
)
target_link_libraries(nx2app
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(ros2qnx
  src/ros2qnx.cpp
  src/input.cpp
)
target_link_libraries(ros2qnx
  ${catkin_LIBRARIES}
)

install(TARGETS qnx2ros ros2qnx
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
