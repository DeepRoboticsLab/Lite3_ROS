#introduction
    The software package implements the mutual conversion of ROS and UDP messages: 
    1.Receive UDP datagrams from motion hosts and publish them as ROS topic messages;
    2.Send ROS topic messages from sensory hosts to motion hosts using UDP;
    3.Receive instruction code from app to control AI function on and off.

#使用方法
```mkdir ~p message_transformer_ws/src```
```cd message_transformer_ws/src```
```git clone .....```
```catkin_make```
```cd message_transformer_ws/```
```source devel/setup.bash```
```roslaunch message_transformer_cpp message_transformer_cpp.launch```

---
message_transformer
|---include
|---src

    |---input.cc
    |---moving_average.cpp
    |---nx2app.cpp
    |---qnx2ros.cpp
    |---ros2qnx.cpp
|---launch
|---CMakeLists.txt
|---package.xml

##ros2qnx.cpp: 
Send ROS topic messages from sensory hosts to motion hosts using UDP;
发布的话题：/cmd_vel  /geometry_msgs/Twist
其中 linear.x为前向速度(m/s)，向前为正，linear.y为侧向速度，向左为正，angular.z为转向角速度(弧度/s)，向左转为正。传输程序会订阅该话题，发送速度指令消息给运动主机。

##nx2app.cpp
接收app指令码控制管理ai功能开启关闭状态，并向app反馈ai功能当前状态。

##qnx2ros.cpp
运动主机通过UDP上报机器人状态信息，接收到状态信息后，将其转为ros话题发布，供感知主机上其他ai功能使用。
该节点发布的话题：
    机器人里程计：    /leg_odom       (nav_msgs::Odometry)
    IMU数据：        /imu/data      (sensor_msgs::Imu)
    机器人关节角度：   /joint_states   (sensor_msgs::JointState)