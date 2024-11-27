#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class SensorLogger{
public:
    SensorLogger(rclcpp::Node::SharedPtr nh, std::string sensor_name) : sensor_name_(sensor_name){
        sub_ = nh->create_subscription<std_msgs::msg::Bool>("/sensor_status/" + sensor_name + "_isalive", 1, std::bind(&SensorLogger::sensor_stat_set, this, std::placeholders::_1));
    }

    void sensor_stat_set(std_msgs::msg::Bool::ConstSharedPtr msg)
    {
        isalive_ = msg->data;
    }

    bool isalive_=false;

private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
    std::string sensor_name_;
};

class SensorsLogger
{
public:
    std::unique_ptr<SensorLogger> imu_;
    std::unique_ptr<SensorLogger> odom_;
    std::unique_ptr<SensorLogger> odom2_;
    std::unique_ptr<SensorLogger> joint_;
    std::unique_ptr<SensorLogger> realsense_;
    std::unique_ptr<SensorLogger> lidar_;
    std::unique_ptr<SensorLogger> ultrasound_;

    SensorsLogger(rclcpp::Node::SharedPtr nh)
    : nh_(nh){
        imu_ = std::make_unique<SensorLogger>(nh, "imu");
        odom_ = std::make_unique<SensorLogger>(nh, "odom");
        odom2_ = std::make_unique<SensorLogger>(nh, "odom2");
        joint_ = std::make_unique<SensorLogger>(nh, "joint");
        realsense_ = std::make_unique<SensorLogger>(nh, "realsense");
        lidar_ = std::make_unique<SensorLogger>(nh, "lidar");
        ultrasound_ = std::make_unique<SensorLogger>(nh, "ultrasound");
    }
    ~SensorsLogger(){
        RCLCPP_INFO(nh_->get_logger(), "~SensorsLogger()");
    }

private:
    rclcpp::Node::SharedPtr nh_;
    bool isdebug_=false;
};