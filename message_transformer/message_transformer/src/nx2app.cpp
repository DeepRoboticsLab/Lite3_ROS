#include <errno.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <netinet/in.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <thread>
#include <arpa/inet.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include <geometry_msgs/Point.h>
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <fstream>
#include "../include/protocol.h"
#include "sensor_msgs/Joy.h"
#include "../include/sensor_logger.h"
#include <memory>

using namespace std;

class FunctionState
{
public:
  bool obstacle_avoidance = false;
  //TODO:
  bool people_tracking = false;
  bool slam = false;
  bool navigation = false;
};

using namespace NX2APPProtocol;

class NX2APP
{
public:
  NX2APP(ros::NodeHandle nh, int server_port){
    sensors_logger_ = std::make_unique<SensorsLogger>(nh);

    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    
    memset(&listen_addr_, 0, sizeof(struct sockaddr_in));
    listen_addr_.sin_family = AF_INET;
    listen_addr_.sin_port = htons(server_port);
    listen_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
    bind(socket_fd_, (struct sockaddr *)&listen_addr_, sizeof(listen_addr_)) < 0;

    logfile_.open("/home/ysc/lite_cog/transfer/nx2app_log.txt");
    logfile_ << "start logging" << endl;

    joy_pub_ = nh.advertise<sensor_msgs::Joy>("commands/joy_raw", 1);    
  }

  ~NX2APP(){
    close(socket_fd_);
    logfile_.close();
  }

  void ReceiveFrame(){
    int len = sizeof(listen_addr_);
    if((recv_num_ = recvfrom(socket_fd_, receive_buffer_, sizeof(receive_buffer_), 0,(struct sockaddr *)&send_addr_,(socklen_t *)&len)) < 0) 
    {
      perror("recvfrom error:");
      exit(1);
    }
  }

  void LogNewRound(){
    logfile_ << "-----new round-----" << endl;
    if(!sensors_logger_->imu_->isalive_){
      logfile_ << "imu is down"        << endl;
    }
    if(!sensors_logger_->odom_->isalive_){
      logfile_ << "odom is down"       << endl;
    }
    if(!sensors_logger_->odom2_->isalive_){
      logfile_ << "odom2 is down"      << endl;
    }
    if(!sensors_logger_->joint_->isalive_){
      logfile_ << "joint is down"        << endl;
    }
    if(!sensors_logger_->realsense_->isalive_){
      logfile_ << "realsense is down"  << endl;
    }
    if(!sensors_logger_->lidar_->isalive_){
      logfile_ << "lidar is down"      << endl;
    }
    if(!sensors_logger_->ultrasound_->isalive_){
      logfile_ << "ultrasound is down"        << endl;
    }
  }

  void ParseFrame(){
    switch (recv_num_)
    {
    case sizeof(SimpleCMD):
      ParseSimpleCMDFrame();
      break;
    
    case sizeof(JoystickChannelFrame):
      ParseJoystickChannelFrame();
      break;

    default:
      break;
    }
  }

  void ParseSimpleCMDFrame(){
    SimpleCMD *dr = (SimpleCMD *)(receive_buffer_);
    switch (dr->cmd_code)
    {
    case 0x21012109:{
      //start obstacle avoidance
      if (dr->cmd_value==0x40 && function_state_.obstacle_avoidance == false)
      {
        if(sensors_logger_->imu_->isalive_)
        {
          logfile_ << "start obstacle avoidance" << endl;
          int ret;
          ret = system("systemctl start realsense.service &");
          logfile_ << "systemctl start realsense.service & " << ret << endl;
          ret = system("systemctl start voa.service &");
          logfile_ << "systemctl start realsense.service & " << ret << endl;
          function_state_.obstacle_avoidance = true; 
        }
        else
        {
          logfile_ << "imu not ready, obstacle avoidance not started." << endl;
        }
      }
      //Turn off all ai functions
      if (dr->cmd_value==0x00)
      {
        logfile_ << "kill all functions" << endl;
        int ret;
        ret = system("systemctl stop realsense.service &");        
        logfile_ << "systemctl stop realsense.service & " << ret << endl;
        ret = system("systemctl stop voa.service &");
        logfile_ << "systemctl stop voa.service & " << ret << endl;
        function_state_.obstacle_avoidance = false;
      }
      break;
    }
    //Inquire obstacle avoidance state
    case 0x2101210D:{
      char outBuf[7]="";
      FILE *fp = popen("systemctl is-active voa.service", "r");
      if(fp){
          char *ret = fgets(outBuf, 7, fp);
          pclose(fp);
      }
      SimpleCMD message;
      if(memcmp(outBuf,"active",6)==0)  
      {
        logfile_ << "voa active" << endl;
        message.cmd_code = 0x2101210D;
        message.cmd_value = 0x11;
        message.type = 0;
        ssize_t nbytes = sendto(socket_fd_, &message, sizeof(message), 0,
              (struct sockaddr *)&send_addr_, sizeof(send_addr_));
        logfile_ << "bytes send" << nbytes << endl;
        logfile_ << "ip: " << inet_ntoa(send_addr_.sin_addr) << endl;
        logfile_ << "port: " << ntohs(send_addr_.sin_port) << endl;
      }
      else
      {
        logfile_ << "voa not active" << endl;
        message.cmd_code = 0x2101210D;
        message.cmd_value = 0x10;
        message.type = 0;
        ssize_t nbytes = sendto(socket_fd_, &message, sizeof(message), 0,
              (struct sockaddr *)&send_addr_, sizeof(send_addr_));
        logfile_ << "bytes send" << nbytes << endl;
        logfile_ << "ip: " << inet_ntoa(send_addr_.sin_addr) << endl;
        logfile_ << "port: " << ntohs(send_addr_.sin_port) << endl;
      }
      break;
    }

    default:
      break;
    }
  }

  void ParseJoystickChannelFrame(){
    ROS_WARN("ParseJoystickChannelFrame");
    JoystickChannelFrame *frame = (JoystickChannelFrame *)(receive_buffer_);
    if(frame->stx[0] != kHeader[0] || frame->stx[1] != kHeader[1]){
      ROS_WARN("Receiving a JoystickChannelFrame with incorrect stx!");
      return;
    }
    if (frame->id != (uint8_t)ControllerType::kRetroid){
      ROS_WARN("Receiving a JoystickChannelFrame with incorrect id!");
      return;
    }
    uint16_t checksum=0;
    uint8_t *pdata=frame->data;
    for(int i=0; i<frame->data_len; i++){
      checksum+=static_cast<uint8_t>(pdata[i]);
    }
    if(checksum!=frame->checksum){
      ROS_WARN("Receiving a JoystickChannelFrame with incorrect checksum!");
      return;
    }

    std::bitset<kChannlSize> value_bit(0);
    int16_t ch[kChannlSize];
    memcpy(ch, frame->data, sizeof(ch));
    for(int i = 0; i < kChannlSize; i++){
      value_bit[i] = ch[i];
    }
    joystick_state_.value = value_bit.to_ulong();

    joystick_state_.left  = (frame->left_axis_x == -kJoystickRange) ? (uint8_t)JoystickKeyStatus::kPressed 
                  : ((frame->left_axis_x ==  kJoystickRange) ? (uint8_t)JoystickKeyStatus::kReleased : (uint8_t)JoystickKeyStatus::kReleased);
    joystick_state_.right = (frame->left_axis_x ==  kJoystickRange) ? (uint8_t)JoystickKeyStatus::kPressed 
                  : ((frame->left_axis_x == -kJoystickRange) ? (uint8_t)JoystickKeyStatus::kReleased : (uint8_t)JoystickKeyStatus::kReleased);
    joystick_state_.up    = (frame->left_axis_y ==  kJoystickRange) ? (uint8_t)JoystickKeyStatus::kPressed 
                  : ((frame->left_axis_y == -kJoystickRange) ? (uint8_t)JoystickKeyStatus::kReleased : (uint8_t)JoystickKeyStatus::kReleased);
    joystick_state_.down  = (frame->left_axis_y == -kJoystickRange) ? (uint8_t)JoystickKeyStatus::kPressed 
                  : ((frame->left_axis_y ==  kJoystickRange) ? (uint8_t)JoystickKeyStatus::kReleased : (uint8_t)JoystickKeyStatus::kReleased);

    joystick_state_.left_axis_x = frame->left_axis_x/(float)kJoystickRange;
    joystick_state_.left_axis_y = frame->left_axis_y/(float)kJoystickRange;
    joystick_state_.right_axis_x = frame->right_axis_x/(float)kJoystickRange;
    joystick_state_.right_axis_y = frame->right_axis_y/(float)kJoystickRange;

    sensor_msgs::Joy msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "joy"; 
    msg.axes.push_back(-joystick_state_.left_axis_x);
    msg.axes.push_back(joystick_state_.left_axis_y);
    if(joystick_state_.L2){
      msg.axes.push_back(-1);
    }
    else{
      msg.axes.push_back(1);
    }
    msg.axes.push_back(-joystick_state_.right_axis_x);
    msg.axes.push_back(joystick_state_.right_axis_y);
    if(joystick_state_.R2){
      msg.axes.push_back(-1);
    }
    else{
      msg.axes.push_back(1);
    }
    if(joystick_state_.left){
      msg.axes.push_back(1);
    }
    else if(joystick_state_.right){
      msg.axes.push_back(-1);
    }
    else{
      msg.axes.push_back(0);
    }
    if(joystick_state_.up){
      msg.axes.push_back(1);
    } 
    else if(joystick_state_.down){
      msg.axes.push_back(-1);
    }
    else{
      msg.axes.push_back(0);
    }
    msg.buttons.push_back(joystick_state_.A);
    msg.buttons.push_back(joystick_state_.B);
    msg.buttons.push_back(joystick_state_.X);
    msg.buttons.push_back(joystick_state_.Y);
    msg.buttons.push_back(joystick_state_.L1);
    msg.buttons.push_back(joystick_state_.R1);
    msg.buttons.push_back(joystick_state_.select);
    msg.buttons.push_back(joystick_state_.select);
    msg.buttons.push_back(joystick_state_.start);
    msg.buttons.push_back(joystick_state_.left_axis_button);
    msg.buttons.push_back(joystick_state_.right_axis_button);
    joy_pub_.publish(msg);
  }

  void RunRoutine(){
    ReceiveFrame();
    ros::spinOnce();
    LogNewRound();
    ParseFrame();    
  }

private:
  int socket_fd_=-1;
  struct sockaddr_in listen_addr_;
  struct sockaddr_in send_addr_;

  int recv_num_;
  uint8_t receive_buffer_[512];
  ofstream logfile_;

  FunctionState function_state_;
  RetroidKeys joystick_state_;

  ros::Publisher joy_pub_;

  std::unique_ptr<SensorsLogger> sensors_logger_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "nx2app");
  ros::NodeHandle nh;
  int server_port;
  nh.param<int>("server_port", server_port, 43899);
  NX2APP nx2app(nh, server_port);
  ros::Rate loop_rate(100);
  ROS_INFO("nx2app node start");
  while(ros::ok()){    
    nx2app.RunRoutine();
    loop_rate.sleep();
  }
  return 0;
}
