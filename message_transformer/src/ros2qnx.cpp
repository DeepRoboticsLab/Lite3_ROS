#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <ctime>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <thread>
#include <arpa/inet.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include "message_transformer/SimpleCMD.h"
#include "message_transformer/ComplexCMD.h"
#include "../include/protocol.h"
#include "../include/sensor_logger.h"

using namespace std;

class ROS2QNX
{
public:
  ROS2QNX()
  {
    fd_ = socket(AF_INET, SOCK_DGRAM,0);
    if(fd_==-1){
      ROS_WARN("scoket create failed!");
    }

    addr_qnx_.sin_family = AF_INET;
    addr_qnx_.sin_port = htons(43893);
    addr_qnx_.sin_addr.s_addr = inet_addr("192.168.1.120");
  }

  void CmdVelCallback(geometry_msgs::TwistConstPtr msg){
    int nbytes;
    ComplexCMD complexcmd;
    complexcmd.cmd_code = 320;
    complexcmd.cmd_value = 8;
    complexcmd.type = 1;
    complexcmd.data = msg->linear.x;                  ///< linear x velocity
    nbytes = sendto(fd_, &complexcmd, sizeof(complexcmd), 0,
        (struct sockaddr *)&addr_qnx_, sizeof(addr_qnx_));

    complexcmd.cmd_code =325;
    complexcmd.cmd_value = 8;
    complexcmd.type = 1;
    complexcmd.data = msg->linear.y;                 ///< linear y velocity
    nbytes = sendto(fd_, &complexcmd, sizeof(complexcmd), 0,
        (struct sockaddr *)&addr_qnx_, sizeof(addr_qnx_));

    complexcmd.cmd_code = 321;
    complexcmd.cmd_value = 8;
    complexcmd.type = 1;
    complexcmd.data = -msg->angular.z;           ///< angular velocity
    nbytes = sendto(fd_, &complexcmd, sizeof(complexcmd), 0,
        (struct sockaddr *)&addr_qnx_, sizeof(addr_qnx_));
  }

  void KickBallCallback(std_msgs::Int32 msg){
    SimpleCMD cmd;
    cmd.cmd_code = 503;
    cmd.cmd_value = msg.data;
    cmd.type = 0;
    int nbytes = sendto(fd_, &cmd, sizeof(cmd), 0,
        (struct sockaddr *)&addr_qnx_, sizeof(addr_qnx_));
  }

  void SimpleCMDCallback(message_transformer::SimpleCMD msg){
    SimpleCMD cmd;
    cmd.cmd_code = msg.cmd_code;
    cmd.cmd_value = msg.cmd_value;
    cmd.type = msg.type;
    int nbytes = sendto(fd_, &cmd, sizeof(cmd), 0,
        (struct sockaddr *)&addr_qnx_, sizeof(addr_qnx_));
  }

  void ComplexCMDCallback(message_transformer::ComplexCMD msg){
    ComplexCMD cmd;
    cmd.cmd_code = msg.cmd_code;
    cmd.cmd_value = msg.cmd_value;
    cmd.type = msg.type;
    cmd.data = msg.data;
    int nbytes = sendto(fd_, &cmd, sizeof(cmd), 0,
        (struct sockaddr *)&addr_qnx_, sizeof(addr_qnx_));
  }

private:
  int fd_=-1;
  struct sockaddr_in addr_qnx_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ros2qnx");
  ros::NodeHandle nh;

  ROS2QNX ros2qnx;
  ROS_INFO("-----   ros2qnx node up   -----");
  ros::Subscriber vel_sub = nh.subscribe("cmd_vel", 1, &ROS2QNX::CmdVelCallback, &ros2qnx);
  ros::Subscriber vel_sub2 = nh.subscribe("cmd_vel_corrected", 1, &ROS2QNX::CmdVelCallback, &ros2qnx);
  ros::Subscriber kickball_sub = nh.subscribe("kick_ball", 1, &ROS2QNX::KickBallCallback, &ros2qnx);
  ros::Subscriber simplecmd_sub = nh.subscribe("simple_cmd", 1, &ROS2QNX::SimpleCMDCallback, &ros2qnx);
  ros::Subscriber complexcmd_sub = nh.subscribe("complex_cmd", 1, &ROS2QNX::ComplexCMDCallback, &ros2qnx);

  ros::spin();

  return 0;
}
