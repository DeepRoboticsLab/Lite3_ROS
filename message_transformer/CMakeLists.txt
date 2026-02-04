cmake_minimum_required(VERSION 2.8.3)
project(message_transformer)

SET(CMAKE_CXX_FLAGS "-std=c++14 -g -O3 ${CMAKE_CXX_FLAGS}")
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

add_message_files(
  FILES
  SimpleCMD.msg
  ComplexCMD.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_runtime
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
)
add_dependencies(qnx2ros
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(qnx2ros
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(ros2qnx
  src/ros2qnx.cpp
)
add_dependencies(ros2qnx
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ros2qnx
  ${catkin_LIBRARIES}
)

add_executable(nx2app
  src/nx2app.cpp
)
add_dependencies(nx2app
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(nx2app
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(sensor_checker
  src/sensor_checker.cpp
)
add_dependencies(sensor_checker
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(sensor_checker
  ${catkin_LIBRARIES}
)

install(TARGETS nx2app qnx2ros ros2qnx sensor_checker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ./launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY ./script
  DESTINATION ${CMAKE_INSTALL_PREFIX}
)

  
