#include "ros/ros.h"
#include "std_msgs/Bool.h"

class SensorLogger{
public:
    SensorLogger(ros::NodeHandle nh, std::string sensor_name) : sensor_name_(sensor_name){
        sub_ = nh.subscribe("/sensor_status/" + sensor_name + "_isalive", 1, &SensorLogger::sensor_stat_set, this);
    }

    void sensor_stat_set(const std_msgs::Bool::ConstPtr& msg)
    {
        isalive_ = msg->data;
    }

    bool isalive_=false;

private:
    ros::Subscriber sub_;
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

    SensorsLogger(ros::NodeHandle &nh){
        imu_ = std::make_unique<SensorLogger>(nh, "imu");
        odom_ = std::make_unique<SensorLogger>(nh, "odom");
        odom2_ = std::make_unique<SensorLogger>(nh, "odom2");
        joint_ = std::make_unique<SensorLogger>(nh, "joint");
        realsense_ = std::make_unique<SensorLogger>(nh, "realsense");
        lidar_ = std::make_unique<SensorLogger>(nh, "lidar");
        ultrasound_ = std::make_unique<SensorLogger>(nh, "ultrasound");
    }
private:
    bool isdebug_=false;
};