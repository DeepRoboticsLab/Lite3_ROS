# 绝影Lite3感知开发

[English](./README.md)

## 运动通信功能包`transfer`

### 功能介绍

本案例实现了ROS2与UDP消息的功能转换。

绝影Lite3的运动主机与感知主机之间、感知主机与手柄App之间的数据传输均采用UDP协议，通过本案例提供的***fransfer*** 软件包，可实现：

- 将运动主机上报的UDP消息转换为ROS2话题进行发布
- 将感知主机下发的运动控制指令使用UDP发送给运动主机
- 接收APP发送的控制指令,以开启和关闭感知主机上的AI功能
- 接收APP的按键和摇杆状态信息并转为ROS话题发布

其提供的ROS通信接口：

**发布话题:**  运动主机向感知主机传输数据

```html
足式里程计(仅位姿):     /leg_odom       (geometry_msgs::msg::PoseWithCovarianceStamped)
足式里程计(位姿和速度):     /leg_odom2      (nav_msgs::msg::Odometry)
IMU数据:       /imu/data       (sensor_msgs::msg::Imu)
关节数据:       /joint_states   (sensor_msgs::msg::JointState)
```

**订阅话题：**  感知主机向运动主机传输数据

```html
速度指令:   /cmd_vel        (geometry_msg::msg::Twist)
```


### 使用方法

1. 打开一个新的终端依次执行以下命令，以**启动通信功能包节点** `jetson2motion`, `jetson2app`, `sensor_checker`:

	```bash
	source ~/lite_cog_ros2/transfer/devel/setup.bash            #添加transfer程序包工作空间环境变量
	roslaunch transfer transfer.launch    #启动通信功能包节点
	```

2. 打开一个新的终端，使用ROS中的`ros2 topic`命令**查看机器狗状态信息**：

	```bash
	ros2 topic info xxxxxx
	ros2 topic echo xxxxxx     # xxxxxx指的是具体话题名称，可在自己的代码中订阅话题进行二次开发
	```

3. 使用`/cmd_vel`话题**向运动主机下发速度指令**，话题消息类型`geometry_msgs/msg/Twist`定义如下：

	```bash
	geometry_msgs/msg/Vector3 linear				# 线速度(m/s)
		float64 x					# 前向速度，向前为正
		float64 y					# 侧向速度，向左为正
		float64 z					# 无效参数
	geometry_msgs/msg/Vector3 angular				# 角速度(rad/s)
		float64 x					# 无效参数
		float64 y					# 无效参数
		float64 z					# 转向角速度，左转为正
	```
	- 用户可在基于ROS编译的C++和Python程序中发布该话题(需要用户具有ROS基础，ROS基础的学习请参考 https://docs.ros.org/en/foxy/Tutorials.html )，也可以打开一个终端，输入以下命令发布进行调试： 

    ```bash
    ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear:{x: 0.2, y: 0.1, z: 0.0}, angular:{x: 0.0, y: 0.0, z: 0.3}}"
    ```

	- 运行命令，即可发布话题。

	- 传输程序会订阅该话题，并将其转为UDP指令消息发给运动主机。

	- 在传输程序已正常开启的情况下，用APP切入自动模式，机器狗即可按照如上速度行动，为防止在调试过程中对人或物品造成损伤，请在空旷处调试，并随时准备将机器狗切回手动模式进行接管。  

### 程序结构

```bash
~/lite_cog/transfer/
├── LICENSE
├── README.md
├── README_ZH.md
└── src
    ├── transfer
    │   ├── CMakeLists.txt
    │   ├── include
    │   │   └── protocol.hpp
    │   ├── launch
    │   │   └── transfer_launch.py
    │   ├── package.xml
    │   └── src
    │       ├── Jetson2App.cpp
    │       ├── Jetson2Motion.cpp
    │       ├── SensorChecker.cpp
    │       └── SensorsLogger.hpp
    └── transfer_interfaces
        ├── CMakeLists.txt
        ├── msg
        │   ├── MotionComplexCMD.msg
        │   └── MotionSimpleCMD.msg
        └── package.xml
```

- ***Jetson2App.cpp***
包含AppReceiver类
AppReceiver类接收手柄App指令、执行对应操作并回包。除此之外，AppReceiver还会接收手柄状态并转换为ROS话题发布。

- ***Jetson2Motion.cpp***
包含MotionReceiver和MotionSender类。
MotionReceiver类用于接收运动主机上报的数据，并将其转化为ROS话题，供其他功能包调用。
MotionSender类用于订阅其他功能包节点发布的话题，转化为UDP数据报下发给运动主机。

- ***SensorChecker.cpp***
用于订阅机器狗传感器数据话题，并发布传感器状态话题。

- ***SensorLogger.cpp***
用于订阅机器狗传感器状态话题并保存结果
