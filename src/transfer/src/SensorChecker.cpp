#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"

template <typename T>
class Sensor
{
public:
    Sensor(rclcpp::Node::SharedPtr nh, std::string sensor_name, std::string in_topic_name, std::string out_topic_name, bool isdebug) 
    : nh_(nh), sensor_name_(sensor_name), isdebug_(isdebug)
    {
        sub_ = nh->create_subscription<T>(in_topic_name, 10, std::bind(&Sensor::Callback, this, std::placeholders::_1));
        pub_ = nh->create_publisher<std_msgs::msg::Bool>(out_topic_name, 10);
        isalive.data=false;
    }

    void Check()
    {
        double t_diff_s = rclcpp::Clock(RCL_STEADY_TIME).now().seconds() - last_time_;
        if(last_time_ == 0)
        {
            isalive.data = false;
            if(report_times_++ < 10){
                RCLCPP_WARN(nh_->get_logger(), "no [%s] data received since setup", sensor_name_.c_str());
            }
        }
        else if(t_diff_s > silent_limit)
        {
            isalive.data = false;
            RCLCPP_WARN(nh_->get_logger(), "[%s] has been silient for: [%lf] second.", sensor_name_.c_str(), t_diff_s);
        }
        else
        {
            isalive.data = true;
        }
        pub_->publish(isalive);
    }

private:
    void Callback(const typename T::SharedPtr msg)
    {
        (void *)msg.get();  //to clean warning
        last_time_ = rclcpp::Clock(RCL_STEADY_TIME).now().seconds();
        if(isdebug_){
            RCLCPP_INFO(nh_->get_logger(), "[%s] refreshed: [%lf]", sensor_name_, last_time_);
        }
    }

    //for check
   rclcpp::Node::SharedPtr nh_;
    std::string sensor_name_; 
    bool isdebug_ = false;
    double last_time_ = 0.0;
    int silent_limit = 5;
    int report_times_ = 0;
    std_msgs::msg::Bool isalive;

    //for sub and pub
    typename rclcpp::Subscription<T>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
};

using namespace std::chrono_literals;
class SensorChecker : public rclcpp::Node
{
public:
    SensorChecker()
    : Node("sensor_checker"){
        declare_parameter("isdebug", false);
        get_parameter("isdebug", is_debug_);

        rclcpp::Node::SharedPtr nh = std::shared_ptr<rclcpp::Node>(this);
        imu_= std::make_shared<Sensor<sensor_msgs::msg::Imu>>(nh, "imu", "/imu/data", "/sensor_status/imu_isalive", is_debug_);
        odom_ = std::make_shared<Sensor<geometry_msgs::msg::PoseWithCovarianceStamped>>(nh, "odom", "/leg_odom", "/sensor_status/odom_isalive",is_debug_);
        odom2_ = std::make_shared<Sensor<nav_msgs::msg::Odometry>>(nh, "odom2", "/leg_odom2", "/sensor_status/odom2_isalive", is_debug_);
        joint_ = std::make_shared<Sensor<sensor_msgs::msg::JointState>>(nh, "joint", "/joint_states", "/sensor_status/joint_isalive", is_debug_);
        realsense_ = std::make_shared<Sensor<sensor_msgs::msg::PointCloud2>>(nh, "realsense", "/camera/depth/color/points", "/sensor_status/realsense_isalive", is_debug_);
        lidar_ = std::make_shared<Sensor<sensor_msgs::msg::PointCloud2>>(nh, "lidar", "/rslidar_points", "/sensor_status/lidar_isalive", is_debug_);
        ultrasound_ = std::make_shared<Sensor<std_msgs::msg::Float64>>(nh, "ultrasound", "/us_publisher/ultrasound_distance", "/sensor_status/ultrasound_isalive", is_debug_);

        timer_ = create_wall_timer(500ms, std::bind(&SensorChecker::TimerCallback, this));
    };

    void TimerCallback(){
        if(is_debug_){
            RCLCPP_INFO(nh_->get_logger(), "time now is: [%f]", rclcpp::Clock(RCL_STEADY_TIME).now().seconds());
        }
        imu_->Check();
        odom_->Check();
        odom2_->Check();
        joint_->Check();
        realsense_->Check();
        lidar_->Check();
        ultrasound_->Check();
    }

private:
    bool is_debug_;

    rclcpp::Node::SharedPtr nh_;
    std::shared_ptr<Sensor<sensor_msgs::msg::Imu>> imu_;
    std::shared_ptr<Sensor<geometry_msgs::msg::PoseWithCovarianceStamped>> odom_;
    std::shared_ptr<Sensor<nav_msgs::msg::Odometry>> odom2_;
    std::shared_ptr<Sensor<sensor_msgs::msg::JointState>> joint_;
    std::shared_ptr<Sensor<sensor_msgs::msg::PointCloud2>> realsense_;
    std::shared_ptr<Sensor<sensor_msgs::msg::PointCloud2>> lidar_;
    std::shared_ptr<Sensor<std_msgs::msg::Float64>> ultrasound_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    std::cout << "starting sensor_checker" << std::endl;
    //program init
    rclcpp::init(argc, argv);
    auto sensor_checker = std::make_shared<SensorChecker>();
    rclcpp::spin(sensor_checker);
    rclcpp::shutdown();
    return 0;
}
