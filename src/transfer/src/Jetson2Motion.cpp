#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "../include/protocol.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include "transfer_interfaces/msg/motion_simple_cmd.hpp"
#include "transfer_interfaces/msg/motion_complex_cmd.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#pragma pack(push, 4)
struct MotionSimpleCMD{
    int32_t cmd_code;
    int32_t size;
    int32_t type;
};

struct MotionComplexCMD : public MotionSimpleCMD{
public:
    double data;
};
#pragma pack(pop)

#define PI 3.1415926

class MotionReceiver : public rclcpp::Node{
public:
    MotionReceiver()
    : Node("motion_receiver"){
        int local_port;
        declare_parameter("local_port", 43897);
        get_parameter("local_port", local_port);
        RCLCPP_INFO(get_logger(),"local_port:  %d ",local_port);

        socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if(socket_fd_ < 0){
            perror("socket");
            exit(1);
        }
        memset(&listen_addr_, 0, sizeof(struct sockaddr_in));        ///< initialize to zeros
        listen_addr_.sin_family = AF_INET;                           ///< host byte order
        listen_addr_.sin_port = htons(local_port);                    ///< port in network byte order
        listen_addr_.sin_addr.s_addr = htonl(INADDR_ANY);            ///< automatically fill in my IP
        if (bind(socket_fd_, (struct sockaddr *)&listen_addr_, sizeof(listen_addr_)) < 0){
            perror("bind error:");
            exit(1);
        }

        leg_odom_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("leg_odom", 10);    
        leg_odom_pub2_ = create_publisher<nav_msgs::msg::Odometry>("leg_odom2", 10);
        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        // imu_pub_100_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
        imu_pub_200_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);        
        ultrasound_pub_ = create_publisher<std_msgs::msg::Float64>("/us_publisher/ultrasound_distance", 10);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        rclcpp::QoS best_effort_qos(1);
        best_effort_qos.best_effort();
        handle_pub_ = create_publisher<geometry_msgs::msg::Twist>("/handle_state", best_effort_qos);

        recv_thread_ = std::thread(&MotionReceiver::RunRoutine, this);

        RCLCPP_INFO(get_logger(), "##### MotionReceiver initialized! #####");
    }

    ~MotionReceiver(){
        RCLCPP_INFO(get_logger(), "~MotionReceiver()");
        break_flag = true;
        if(recv_thread_.joinable()){
            recv_thread_.join();
        }
        close(socket_fd_);
    }

    void ReceiveFrame(){
        int len = sizeof(listen_addr_);
        recv_num_ = recvfrom(socket_fd_, receive_buffer_, sizeof(receive_buffer_), 0, (struct sockaddr *)&listen_addr_, (socklen_t *)&len);
        if(recv_num_<0){
            perror("recvfrom error:");
            exit(1);
        }

        times_++;
        if(times_==3000)
        {
            RCLCPP_INFO(get_logger(), "qnx2ros received: [%d] bytes",recv_num_);
            times_=0;
        }
    }

    void ParseFrame(){
        switch(recv_num_)
        {
        case sizeof(QNX2ROSProtocol::RobotStateReceived):
            ParseRobotStateFrame();
            break;
        
        case sizeof(QNX2ROSProtocol::JointStateReceived):
            ParseJointStateFrame();
            break;

        case sizeof(QNX2ROSProtocol::HandleStateReceived):
            ParseHandleStateFrame();
            break;

        case sizeof(QNX2ROSProtocol::ImuDataReceived):
            ParseImuDataFrame();
            break;

        default:
            // ROS_WARN("udp pack length not fit into any");
            break;
        }
    }

    void ParseRobotStateFrame(){
        QNX2ROSProtocol::RobotStateReceived *dr = (QNX2ROSProtocol::RobotStateReceived *)(receive_buffer_);
        QNX2ROSProtocol::RobotState *robot_state = &dr->data;

        if (dr->code == 2305) 
        {
            tf2::Quaternion q;

            geometry_msgs::msg::PoseWithCovarianceStamped leg_odom_data;               
            leg_odom_data.header.frame_id = "odom";                    
            leg_odom_data.header.stamp = rclcpp::Clock(RCL_STEADY_TIME).now();
            q.setRPY(0, 0, robot_state->rpy[2] / 180 * PI);
            leg_odom_data.pose.pose.orientation = tf2::toMsg(q);
            leg_odom_data.pose.pose.position.x = robot_state->pos_world[0];
            leg_odom_data.pose.pose.position.y = robot_state->pos_world[1];
            leg_odom_data.pose.pose.position.z = robot_state->pos_world[2];
            leg_odom_pub_->publish(leg_odom_data);

            nav_msgs::msg::Odometry leg_odom_data2;     
            leg_odom_data2.header.stamp = rclcpp::Clock(RCL_STEADY_TIME).now();
            leg_odom_data2.pose = leg_odom_data.pose;
            leg_odom_data2.twist.twist.linear.x = robot_state->vel_body[0];
            leg_odom_data2.twist.twist.linear.y = robot_state->vel_body[1];
            leg_odom_data2.twist.twist.angular.z = robot_state->rpy_vel[2];
            leg_odom_pub2_->publish(leg_odom_data2);
            
            sensor_msgs::msg::Imu imu_msg;
            imu_msg.header.stamp = rclcpp::Clock(RCL_STEADY_TIME).now();
            imu_msg.header.frame_id = "base_link";
            q.setRPY(robot_state->rpy[0]/180*PI, robot_state->rpy[1]/180*PI, robot_state->rpy[2]/180*PI);
            imu_msg.orientation = tf2::toMsg(q);
            imu_msg.angular_velocity.x = robot_state->rpy_vel[0];
            imu_msg.angular_velocity.y = robot_state->rpy_vel[1];
            imu_msg.angular_velocity.z = robot_state->rpy_vel[2];
            imu_msg.linear_acceleration.x = robot_state->xyz_acc[0];
            imu_msg.linear_acceleration.y = robot_state->xyz_acc[1];
            imu_msg.linear_acceleration.z = robot_state->xyz_acc[2];
            // imu_pub_100_->publish(imu_msg);

            // geometry_msgs::msg::TransformStamped t;
            // t.header.stamp = rclcpp::Clock(RCL_STEADY_TIME).now();
            // t.header.frame_id = "robot_foot_print";
            // t.child_frame_id = "base_link";
            // t.transform.translation.x = 0.0;
            // t.transform.translation.y = 0.0;
            // t.transform.translation.z = 0.34751;
            // q.setRPY(robot_state->rpy[0]/180*PI, robot_state->rpy[1]/180*PI, 0.0);
            // t.transform.rotation.x = q.x();
            // t.transform.rotation.y = q.y();
            // t.transform.rotation.z = q.z();
            // t.transform.rotation.w = q.w();
            // tf_broadcaster_->sendTransform(t);

            std_msgs::msg::Float64 ultrasound_distance;
            ultrasound_distance.data = robot_state->ultrasound[1];
            ultrasound_pub_->publish(ultrasound_distance);
            
            geometry_msgs::msg::TransformStamped leg_odom_trans;       
            leg_odom_trans.header.stamp = rclcpp::Clock(RCL_STEADY_TIME).now();
            leg_odom_trans.header.frame_id = "odom";
            leg_odom_trans.child_frame_id = "base_link";
            leg_odom_trans.transform.translation.x = leg_odom_data.pose.pose.position.x;
            leg_odom_trans.transform.translation.y = leg_odom_data.pose.pose.position.y;
            leg_odom_trans.transform.translation.z = leg_odom_data.pose.pose.position.z;
            leg_odom_trans.transform.rotation = imu_msg.orientation;
            // tf_broadcaster_->sendTransform(leg_odom_trans);
        }
    }

    void ParseJointStateFrame(){
        QNX2ROSProtocol::JointStateReceived *dr = (QNX2ROSProtocol::JointStateReceived *)(receive_buffer_);
        QNX2ROSProtocol::JointState *joint_state = &dr->data;
        if (dr->code == 2306)
        {
            sensor_msgs::msg::JointState joint_state_data;
            joint_state_data.header.stamp = rclcpp::Clock(RCL_STEADY_TIME).now();
            joint_state_data.name.resize(12);
            joint_state_data.position.resize(12);

            joint_state_data.name[0] = "LF_Joint";
            joint_state_data.position[0] = -joint_state->LF_Joint;
            joint_state_data.name[1] = "LF_Joint_1";
            joint_state_data.position[1] = -joint_state->LF_Joint_1;
            joint_state_data.name[2] = "LF_Joint_2";
            joint_state_data.position[2] = -joint_state->LF_Joint_2;

            joint_state_data.name[3] = "RF_Joint";
            joint_state_data.position[3] = -joint_state->RF_Joint;
            joint_state_data.name[4] = "RF_Joint_1";
            joint_state_data.position[4] = -joint_state->RF_Joint_1;
            joint_state_data.name[5] = "RF_Joint_2";
            joint_state_data.position[5] = -joint_state->RF_Joint_2;

            joint_state_data.name[6] = "LB_Joint";
            joint_state_data.position[6] = -joint_state->LB_Joint;
            joint_state_data.name[7] = "LB_Joint_1";
            joint_state_data.position[7] = -joint_state->LB_Joint_1;
            joint_state_data.name[8] = "LB_Joint_2";
            joint_state_data.position[8] = -joint_state->LB_Joint_2;

            joint_state_data.name[9] = "RB_Joint";
            joint_state_data.position[9] = -joint_state->RB_Joint;
            joint_state_data.name[10] = "RB_Joint_1";
            joint_state_data.position[10] = -joint_state->RB_Joint_1;
            joint_state_data.name[11] = "RB_Joint_2";
            joint_state_data.position[11] = -joint_state->RB_Joint_2;
            joint_state_pub_->publish(joint_state_data);
        }
    }

    void ParseHandleStateFrame(){
        QNX2ROSProtocol::HandleStateReceived *dr = (QNX2ROSProtocol::HandleStateReceived *)(receive_buffer_);
        QNX2ROSProtocol::HandleState *handle_state = &dr->data;
        if (dr->code == 2309) 
        {
            geometry_msgs::msg::Twist handle_state_msg;
            handle_state_msg.linear.x = handle_state->left_axis_forward;
            handle_state_msg.linear.y = handle_state->left_axis_side;
            handle_state_msg.angular.z = - handle_state->right_axis_yaw;
            handle_pub_->publish(handle_state_msg);
        }
    }


    void ParseImuDataFrame(){
        QNX2ROSProtocol::ImuDataReceived *dr = (QNX2ROSProtocol::ImuDataReceived *)(receive_buffer_);
        QNX2ROSProtocol::ImuData *imu_data = &dr->data;
        if(dr->code == 0x010901) 
        {
            sensor_msgs::msg::Imu imu_msg;     
            imu_msg.header.frame_id = "base_link";
            imu_msg.header.stamp = rclcpp::Clock(RCL_STEADY_TIME).now();
            tf2::Quaternion q;
            q.setRPY(imu_data->angle_roll / 180 * PI,
                imu_data->angle_pitch / 180 * PI,
                imu_data->angle_yaw / 180 * PI);
            imu_msg.orientation = tf2::toMsg(q);
            imu_msg.angular_velocity.x = imu_data->angular_velocity_roll;
            imu_msg.angular_velocity.y = imu_data->angular_velocity_pitch;
            imu_msg.angular_velocity.z = imu_data->angular_velocity_yaw;
            imu_msg.linear_acceleration.x = imu_data->acc_x;
            imu_msg.linear_acceleration.y = imu_data->acc_y;
            imu_msg.linear_acceleration.z = imu_data->acc_z;
            imu_pub_200_->publish(imu_msg);
        }
    }

    void RunRoutine(){
        while(!break_flag){
            ReceiveFrame();
            ParseFrame();   
        }
    }

private:
    bool break_flag = false;
    int socket_fd_=-1;
    struct sockaddr_in listen_addr_;
    int recv_num_ = -1;
    uint8_t receive_buffer_[512];
    int times_=0;
    std::thread recv_thread_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr leg_odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr leg_odom_pub2_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_100_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_200_;    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr handle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ultrasound_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

class MotionSender : public rclcpp::Node{
public:
    MotionSender()
    : Node("motion_sender"){
        Init();
        RCLCPP_INFO(get_logger(), "##### MotionSender initialized! #####");
    };

    ~MotionSender()
    {
        RCLCPP_INFO(get_logger(), "~MotionSender()");
        close(socked_fd_);
    }

private:
    void Init(){
        RCLCPP_INFO(get_logger(), "init params");
        int target_port;
        declare_parameter("target_port", 43893);
        get_parameter("target_port", target_port);
        RCLCPP_INFO(get_logger(),"target_port:  %d ",target_port);
        declare_parameter("target_ip", "192.168.1.120");
        std::string target_ip;
        get_parameter("target_ip", target_ip);
        RCLCPP_INFO(get_logger(),"target_ip:  %s ",target_ip.c_str());

        RCLCPP_INFO(get_logger(), "init socket");
        socked_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socked_fd_ == -1){
            perror("Error creating socket");
            std::exit(1);
        }
        memset(&remote_addr_, 0, sizeof(remote_addr_));
        remote_addr_.sin_family = AF_INET;
        remote_addr_.sin_port = htons(target_port);
        remote_addr_.sin_addr.s_addr = inet_addr(target_ip.c_str());

        RCLCPP_INFO(get_logger(), "init sub");
        rclcpp::QoS best_effort_qos(1);
        best_effort_qos.best_effort();
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", best_effort_qos, 
            std::bind(&MotionSender::CmdVelCallback, this, std::placeholders::_1));
        cmd_vel_corrected_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_corrected", best_effort_qos,
            std::bind(&MotionSender::CmdVelCallback, this, std::placeholders::_1));
        simplecmd_sub = create_subscription<transfer_interfaces::msg::MotionSimpleCMD>(
            "simple_cmd", 10, 
            std::bind(&MotionSender::SimpleCMDCallback, this, std::placeholders::_1));
        complexcmd_sub = create_subscription<transfer_interfaces::msg::MotionComplexCMD>(
            "complex_cmd", 10,
            std::bind(&MotionSender::ComplexCMDCallback, this, std::placeholders::_1));
    };

    void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
        RCLCPP_INFO(get_logger(),"recevied cmd_vel #############");
        complex_cmd_.cmd_code = 320;
        complex_cmd_.size = 8;
        complex_cmd_.type = 1;
        complex_cmd_.data = msg->linear.x;                  ///< linear velocity
        ssize_t bytesSent = sendto(socked_fd_, &complex_cmd_, sizeof(complex_cmd_), 0, 
                                (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
        RCLCPP_INFO(get_logger(),"sender linear.x sucessful %d !!!",bytesSent);
        RCLCPP_INFO(get_logger(),"sender linear.x:  %f !!!",msg->linear.x);

        if (bytesSent == -1) {
            perror("Error sending message");
            RCLCPP_INFO(get_logger(),"sender linear.x error %d !!!",bytesSent);
            std::exit(1);
        }

        complex_cmd_.cmd_code =325;
        complex_cmd_.size = 8;
        complex_cmd_.type = 1;
        complex_cmd_.data = msg->linear.y;                 ///< Lateral velocity
        ssize_t bytesSent_1 = sendto(socked_fd_, &complex_cmd_, sizeof(complex_cmd_), 0, 
                                (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
        if (bytesSent_1 == -1) {
            perror("Error sending message");
            RCLCPP_INFO(get_logger(),"sender linear.y error %d !!!",bytesSent_1);
            std::exit(1);
        }

        complex_cmd_.cmd_code = 321;
        complex_cmd_.size = 8;
        complex_cmd_.type = 1;
        complex_cmd_.data = -1 * msg->angular.z;               ///< angular velocity
        ssize_t bytesSent_2 = sendto(socked_fd_, &complex_cmd_, sizeof(complex_cmd_), 0, 
                                (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
        if (bytesSent_2 == -1) {
            perror("Error sending message");
            RCLCPP_INFO(get_logger(),"sender angular.z error %d !!!",bytesSent_2);
            std::exit(1);
        }
    };

    void SimpleCMDCallback(transfer_interfaces::msg::MotionSimpleCMD::SharedPtr msg){
        MotionSimpleCMD cmd;
        cmd.cmd_code = msg->cmd_code;
        cmd.size = msg->size;
        cmd.type = msg->type;
        int nbytes = sendto(socked_fd_, &cmd, sizeof(cmd), 0,
            (struct sockaddr *)&remote_addr_, sizeof(remote_addr_));
    }

    void ComplexCMDCallback(transfer_interfaces::msg::MotionComplexCMD::SharedPtr msg){
        MotionComplexCMD cmd;
        cmd.cmd_code = msg->cmd_code;
        cmd.size = msg->size;
        cmd.type = msg->type;
        cmd.data = msg->data;
        int nbytes = sendto(socked_fd_, &cmd, sizeof(cmd), 0,
            (struct sockaddr *)&remote_addr_, sizeof(remote_addr_));
    }

    int socked_fd_;
    MotionComplexCMD complex_cmd_;
    struct sockaddr_in remote_addr_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_corrected_sub_;
    rclcpp::Subscription<transfer_interfaces::msg::MotionSimpleCMD>::SharedPtr simplecmd_sub;
    rclcpp::Subscription<transfer_interfaces::msg::MotionComplexCMD>::SharedPtr complexcmd_sub;    
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto motion_sender = std::make_shared<MotionSender>();
    auto motion_receiver = std::make_shared<MotionReceiver>();
    executor.add_node(motion_sender);
    executor.add_node(motion_receiver);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
