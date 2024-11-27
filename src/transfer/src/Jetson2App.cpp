#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "sensor_msgs/msg/joy.hpp"
#include "./SensorsLogger.hpp"
#include "../include/protocol.hpp"
#include <bitset>
#include <fstream>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#pragma pack(push, 4)
struct AppSimpleCMD{
  int32_t cmd_code;
  int32_t cmd_value;
  int32_t type;
};

struct AppComplexCMD : public AppSimpleCMD{
public:
  double data;
};
#pragma pack(pop)

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

class AppReceiver : public rclcpp::Node
{
public:
    AppReceiver()
    : Node("app_receiver"){
        int local_port;
        declare_parameter("local_port", 43899);
        get_parameter("local_port", local_port);

        sensors_logger_ = std::make_unique<SensorsLogger>
            (std::shared_ptr<rclcpp::Node>(this));

        socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        
        memset(&listen_addr_, 0, sizeof(struct sockaddr_in));
        listen_addr_.sin_family = AF_INET;
        listen_addr_.sin_port = htons(local_port);
        listen_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
        bind(socket_fd_, (struct sockaddr *)&listen_addr_, sizeof(listen_addr_));

        logfile_.open("/home/ysc/lite_cog_ros2/transfer/nx2app_log.txt");
        logfile_ << "start logging" << endl;

        joy_pub_ = create_publisher<sensor_msgs::msg::Joy>("commands/joy_raw", 1);    

        recv_thread_ = std::thread(&AppReceiver::RunRoutine, this);

        RCLCPP_INFO(get_logger(), "##### AppReceiver initialized! #####");
    }

    ~AppReceiver(){
        RCLCPP_INFO(get_logger(), "~AppReceiver()");
        if(recv_thread_.joinable()){
            recv_thread_.join();
        }
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
        case sizeof(AppSimpleCMD):
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
        AppSimpleCMD *dr = (AppSimpleCMD *)(receive_buffer_);
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
                    ret = system("systemctl start realsense_ros2.service &");
                    logfile_ << "systemctl start realsense_ros2.service & " << ret << endl;
                    ret = system("systemctl start voa_ros2.service &");
                    logfile_ << "systemctl start voa_ros2.service & " << ret << endl;
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
                ret = system("systemctl stop realsense_ros2.service &");        
                logfile_ << "systemctl stop realsense_ros2.service & " << ret << endl;
                ret = system("systemctl stop voa_ros2.service &");
                logfile_ << "systemctl stop voa_ros2.service & " << ret << endl;
                function_state_.obstacle_avoidance = false;
            }
            break;
        }
            //Inquire obstacle avoidance state
        case 0x2101210D:{
            AppSimpleCMD message;
            if(function_state_.obstacle_avoidance == true)
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
        RCLCPP_WARN(get_logger(), "ParseJoystickChannelFrame");
        JoystickChannelFrame *frame = (JoystickChannelFrame *)(receive_buffer_);
        if(frame->stx[0] != kHeader[0] || frame->stx[1] != kHeader[1]){
            RCLCPP_WARN(get_logger(), "Receiving a JoystickChannelFrame with incorrect stx!");
            return;
        }
        if (frame->id != (uint8_t)ControllerType::kRetroid){
            RCLCPP_WARN(get_logger(), "Receiving a JoystickChannelFrame with incorrect id!");
            return;
        }
        uint16_t checksum=0;
        uint8_t *pdata=frame->data;
        for(int i=0; i<frame->data_len; i++){
            checksum+=static_cast<uint8_t>(pdata[i]);
        }
        if(checksum!=frame->checksum){
            RCLCPP_WARN(get_logger(), "Receiving a JoystickChannelFrame with incorrect checksum!");
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

        sensor_msgs::msg::Joy msg;
        msg.header.stamp = rclcpp::Clock(RCL_STEADY_TIME).now();
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
        joy_pub_->publish(msg);
    }

    void RunRoutine(){
        while(rclcpp::ok()){
            ReceiveFrame();
            LogNewRound();
            ParseFrame();
        }
    }

private:
    std::thread recv_thread_;

    int socket_fd_=-1;
    struct sockaddr_in listen_addr_;
    struct sockaddr_in send_addr_;

    int recv_num_;
    uint8_t receive_buffer_[512];
    ofstream logfile_;

    FunctionState function_state_;
    RetroidKeys joystick_state_;

    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;

    std::unique_ptr<SensorsLogger> sensors_logger_;
};

//todo
class AppSender : public rclcpp::Node{

};

int main(int argc,char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto app_receiver = std::make_shared<AppReceiver>();
    executor.add_node(app_receiver);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
