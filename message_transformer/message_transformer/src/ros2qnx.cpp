/*
 * @Author: zhanghan 1281311575@qq.com
 * @Date: 2023-02-17 17:22:16
 * @LastEditors: zhanghan 1281311575@qq.com
 * @LastEditTime: 2023-03-22 15:15:22
 * @FilePath: \message_transformer_cpp\src\ros2qnx.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <ctime>
#include "input.h"
using namespace std;

boost::shared_ptr<InputSocket> input_;

#pragma pack(4)
/// @brief cml_vel data structe implementation
struct DataSend {
  int32_t code;
  int32_t size;
  int32_t cons_code;
  double cmd_data;
};
DataSend data;    ///< cml_vel data

/**
 * @brief: Process the subscribed speed information, fill it in the udp instruction protocol, 
 * and forward it to the sports host
 * @agrs: geometry_msgs::TwistConstPtr 
 * @return None
 */  
void velCallback(geometry_msgs::TwistConstPtr msg) {
  data.code = 320;
  data.size = 8;
  data.cons_code = 1;
  data.cmd_data = msg->linear.x;                  ///< linear velocity
  input_->sendPacket((uint8_t*)&data, sizeof(data));

  data.code =325;
  data.size = 8;
  data.cons_code = 1;
  data.cmd_data = msg->linear.y;                 ///< Lateral velocity
  input_->sendPacket((uint8_t*)&data, sizeof(data));

  data.code = 321;
  data.size = 8;
  data.cons_code = 1;
  data.cmd_data = msg->angular.z;               ///< angular velocity
  input_->sendPacket((uint8_t*)&data, sizeof(data));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ros2qnx");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ros::Subscriber vel_sub = nh.subscribe("cmd_vel", 1, velCallback);
  ros::Subscriber vel_sub2 = nh.subscribe("cmd_vel_corrected", 1, velCallback);
  input_.reset(new InputSocket(private_nh));
  ros::spin();

  return 0;
}
