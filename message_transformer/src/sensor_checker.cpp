#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

class Sensor
{
public:
    //for check
    bool isdebug=false;
    double last_time=0;
    int silent_limit=5;
    int report_times_=0;
    std_msgs::Bool isalive;

    //for sub and pub
    char name[20];
    ros::Subscriber sub;
    ros::Publisher pub;

    Sensor(bool isdebug_) : isdebug(isdebug_)
    {
        isalive.data=false;
    }

    void check()
    {
        if(last_time == 0)
        {
            isalive.data = false;
            if(report_times_++ < 10){
                ROS_WARN("no [%s] data received since setup", name);
            }
        }
        else if(ros::Time::now().toSec() - last_time > silent_limit)
        {
            isalive.data = false;
            ROS_WARN("[%s] has been silient for: [%f] second."
                ,name, ros::Time::now().toSec() - last_time);
        }
        else
        {
            isalive.data = true;
        }
        pub.publish(isalive);
    }
};

class Imu : public Sensor
{
public:
    Imu(ros::NodeHandle n,bool isdebug):Sensor(isdebug)
    {
        strcpy(name, "imu");
        sub = n.subscribe<sensor_msgs::Imu>("/imu/data", 10, &Imu::callback, this);
        pub = n.advertise<std_msgs::Bool>("/sensor_status/imu_isalive", 10);
    }
    void callback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        last_time = ros::Time::now().toSec();
        if(isdebug)    {ROS_INFO("[%s] refreshed: [%f]", name, last_time);}
    }
};

class Odom : public Sensor
{
public:
    Odom(ros::NodeHandle n,bool isdebug):Sensor(isdebug)
    {
        strcpy(name, "odom");
        sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/leg_odom", 10, &Odom::callback, this);
        pub = n.advertise<std_msgs::Bool>("/sensor_status/odom_isalive", 10);
    }
    void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        last_time = ros::Time::now().toSec();
        if(isdebug)    {ROS_INFO("[%s] refreshed: [%f]", name, last_time);}
    }
};

class Odom2 : public Sensor
{
public:
    Odom2(ros::NodeHandle n,bool isdebug):Sensor(isdebug)
    {
        strcpy(name, "odom2");
        sub = n.subscribe<nav_msgs::Odometry>("/leg_odom2", 10, &Odom2::callback, this);
        pub = n.advertise<std_msgs::Bool>("/sensor_status/odom2_isalive", 10);
    }
    void callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        last_time = ros::Time::now().toSec();
        if(isdebug)    {ROS_INFO("[%s] refreshed: [%f]", name, last_time);}
    }
};

class Joint : public Sensor
{
public:
    Joint(ros::NodeHandle n,bool isdebug):Sensor(isdebug)
    {
        strcpy(name, "joint");
        sub = n.subscribe<sensor_msgs::JointState>("/joint_states", 10, &Joint::callback, this);
        pub = n.advertise<std_msgs::Bool>("/sensor_status/joint_isalive", 10);

    }
    void callback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        last_time = ros::Time::now().toSec();
        if(isdebug)    {ROS_INFO("[%s] refreshed: [%f]", name, last_time);}
    }
};

class Realsense : public Sensor
{
public:
    Realsense(ros::NodeHandle n,bool isdebug):Sensor(isdebug)
    {
        strcpy(name, "realsense");
        sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 10, &Realsense::callback, this);
        pub = n.advertise<std_msgs::Bool>("/sensor_status/realsense_isalive", 10);
    }
    void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        last_time = ros::Time::now().toSec();
        if(isdebug)    {ROS_INFO("[%s] refreshed: [%f]", name, last_time);}
    }
};

class Lidar : public Sensor
{
public:
    Lidar(ros::NodeHandle n,bool isdebug):Sensor(isdebug)
    {
        strcpy(name, "lidar");
        sub = n.subscribe<sensor_msgs::PointCloud2>("/rslidar_points", 10, &Lidar::callback, this);
        pub = n.advertise<std_msgs::Bool>("/sensor_status/lidar_isalive", 10);
    }
    void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        last_time = ros::Time::now().toSec();
        if(isdebug)    {ROS_INFO("[%s] refreshed: [%f]", name, last_time);}
    }
};

class Ultrasound : public Sensor
{
public:
    Ultrasound(ros::NodeHandle n,bool isdebug):Sensor(isdebug)
    {
        strcpy(name, "ultrasound");
        sub = n.subscribe<std_msgs::Float64>("/us_publisher/ultrasound_distance", 10, &Ultrasound::callback, this);
        pub = n.advertise<std_msgs::Bool>("/sensor_status/ultrasound_isalive", 10);
    }
    void callback(const std_msgs::Float64::ConstPtr& msg)
    {
        last_time = ros::Time::now().toSec();
        if(isdebug)    {ROS_INFO("[%s] refreshed: [%f]", name, last_time);}
    }
};

class SensorChecker
{
public:
    int check_freq_hz;
    Imu imu;
    Odom odom;
    Odom2 odom2;
    Joint joint;
    Realsense realsense;
    Lidar lidar;
    Ultrasound ultrasound;
    SensorChecker(ros::NodeHandle n, bool isdebug) : 
        imu(n,isdebug), odom(n,isdebug), odom2(n,isdebug), joint(n,isdebug),
        realsense(n,isdebug), lidar(n,isdebug), ultrasound(n,isdebug){};

    void check(){
        imu.check();
        odom.check();
        odom2.check();
        joint.check();
        realsense.check();
        lidar.check();
        ultrasound.check();
    }
};

int main(int argc, char **argv)
{
    std::cout << "starting sensor_checker" << std::endl;
    //program init
    ros::init(argc, argv, "sensor_checker");  
    ros::NodeHandle n;
    bool isdebug;
    bool isdebug_param_ok = n.param<bool>("/sensor_checker/isdebug", isdebug, false);
    
    //SensorChecker init
    SensorChecker sensor_checker(n,isdebug);
    bool imu_param_ok = n.param<int>("/sensor_checker/imu_silent_limit", sensor_checker.imu.silent_limit, 10);
    bool odom_param_ok = n.param<int>("/sensor_checker/odom_silent_limit", sensor_checker.odom.silent_limit, 10);
    bool odom2_param_ok = n.param<int>("/sensor_checker/odom2_silent_limit", sensor_checker.odom2.silent_limit, 10);
    bool joint_param_ok = n.param<int>("/sensor_checker/joint_silent_limit", sensor_checker.joint.silent_limit, 10);
    bool realsense_param_ok = n.param<int>("/sensor_checker/realsense_silent_time", sensor_checker.realsense.silent_limit, 10);
    bool lidar_param_ok = n.param<int>("/sensor_checker/lidar_silent_time", sensor_checker.lidar.silent_limit, 10);
    bool ultrasound_param_ok = n.param<int>("/sensor_checker/ultrasound_silent_time", sensor_checker.ultrasound.silent_limit, 10);
    bool freq_param_ok = n.param<int>("/sensor_checker/check_freq_hz", sensor_checker.check_freq_hz, 10);
    if(!imu_param_ok)    {ROS_WARN("get param /sensor_checker/imu_silent_limit failed");}
    if(!odom_param_ok)    {ROS_WARN("get param /sensor_checker/odom_silent_limit failed");}
    if(!odom2_param_ok)    {ROS_WARN("get param /sensor_checker/odom2_silent_limit failed");}
    if(!joint_param_ok)    {ROS_WARN("get param /sensor_checker/joint_silent_limit failed");}
    if(!realsense_param_ok)    {ROS_WARN("get param /sensor_checker/realsense_silent_time failed");}
    if(!lidar_param_ok)    {ROS_WARN("get param /sensor_checker/lidar_silent_time failed");}
    if(!ultrasound_param_ok)    {ROS_WARN("get param /sensor_checker/ultrasound_silent_time failed");}
    if(!freq_param_ok)    {ROS_WARN("get param /sensor_checker/check_freq_hz failed");}
    if(!isdebug_param_ok)    {ROS_WARN("get param /sensor_checker/isdebug failed");}

    //loop
    ros::Rate loop_rate(sensor_checker.check_freq_hz);
    while (ros::ok())
    {
        if(isdebug)  {ROS_INFO("time now is: [%f]",ros::Time::now().toSec());}
        sensor_checker.check();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
