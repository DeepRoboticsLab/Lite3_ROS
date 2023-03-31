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
#include<arpa/inet.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include "std_msgs/Int32.h"
#include <geometry_msgs/Point.h>
#include "moving_average.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>

using namespace std;


#define PI 3.1415926
bool people_tracking = false;
bool obstacle_avoidance = false;
#pragma pack(4)


/// @brief A structure that receives app scripts to control the opening and closing of AI functions
struct AiSwitch
{
  int code;   ///< Instruction Code
  int size;   ///< Instruction Value
  int cons_code;  ///< Instruction type
};


/// @brief start_obstacle_avoidance
/// @todo Invoke system instruction to turn on stop-block function
void start_obstacle_avoidance(){
   system("sh /home/ysc/message_transformer_ws/src/obs_avoidence.sh");
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "nx2app");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  int SERV_PORT;       
  int default_port = 43899;       
  private_nh.param<int>("SERV_PORT", SERV_PORT, default_port);

  //Initialize the udp server to receive the datagram issued by the app
  int sock_fd_in = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_fd_in < 0)
  {
    perror("socket");
    exit(1);
  }
  struct sockaddr_in addr_serv;
  memset(&addr_serv, 0, sizeof(struct sockaddr_in));
  addr_serv.sin_family = AF_INET;
  addr_serv.sin_port = htons(SERV_PORT);
  addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);
  int len = sizeof(addr_serv);
  if (bind(sock_fd_in, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0) 
  {
    perror("bind error:");
    exit(1);
  }
  int recv_num = -1;
  char recv_buf[500];

  //Initialize udp customer service terminal to send information to app
  int sock_fd_out = socket(AF_INET, SOCK_DGRAM,0);
  struct sockaddr_in addr_client;
  addr_client.sin_family = AF_INET;
  addr_client.sin_addr.s_addr = inet_addr("192.168.1.255");
  addr_client.sin_port = htons(43897);

  ros::Rate loop_rate(100);
  while (ros::ok()) {
    if((recv_num = recvfrom(sock_fd_in, recv_buf, sizeof(recv_buf), 0,(struct sockaddr *)&addr_serv,(socklen_t *)&len)) < 0) 
    {
      perror("recvfrom error:");
      exit(1);
    }

    if(recv_num == sizeof(AiSwitch))
    {
        AiSwitch *dr = (AiSwitch *)(recv_buf);
        //Verify the datagram received by udp and call the corresponding thread execution program
        if (dr->code == 553722121)      ///< Command code for controlling AI function switches
        {
          if (dr->size==64 && obstacle_avoidance==false)   
          {   
              thread t1 (start_obstacle_avoidance);
              t1.detach();
              obstacle_avoidance = true; 
          }
          //Turn off all ai functions
          if (dr->size==0)
          {    
            //Grab the obstacle avoidance program process and use the kill command to kill the process
            system("ps aux | grep -e realsense-driver* | grep -v grep | awk '{print $2}' | xargs -i kill -9 {}");
            system("ps aux | grep -e height_map* | grep -v grep | awk '{print $2}' | xargs -i kill -9 {}");
            system("ps aux | grep -e height_map* | grep -v grep | awk '{print $2}' | xargs -i kill -9 {}");
            system("ps aux | grep -e height_map* | grep -v grep | awk '{print $2}' | xargs -i kill -9 {}");
            obstacle_avoidance=false;
          }
        }          
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  close(sock_fd_in);
  return 0;
}
