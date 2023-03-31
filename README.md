# 绝影Lite3感知开发

## 运动通信功能包`message_transformer`

### 功能介绍

本案例实现了ROS与UDP消息的功能转换。

绝影Lite3的运动主机与感知主机之间、感知主机与手柄App之间的数据传输均采用UDP协议，通过本案例提供的***message_transformer_cpp*** 软件包，可实现：

- 将运动主机上报的UDP消息转换为ROS话题进行发布，将感知主机下发的运动控制指令使用UDP发送给运动主机；
- 接收App发送的控制指令，以开启和关闭感知主机上的AI功能。
	
其提供的ROS通信接口：

**发布话题:**  运动主机向感知主机传输数据

```html
腿部里程计：      /leg_odom       (nav_msgs::Odometry)
IMU数据：         /imu/data       (sensor_msgs::Imu)
关节数据：        /joint_states   (sensor_msgs::JointState)
```

**订阅话题：**  感知主机向运动主机传输数据

```html
速度指令：         /cmd_vel        (geometry_msgs::Twist)
```


### 使用方法

1. 打开一个新的终端依次执行以下命令，以**启动通信功能包节点** `nx2app`、`rk2ros`、`ros2rk`：

	```bash
	cd message_transformer_ws/                                    #进入功能包工作空间（/home/ysc/message_transformer_ws）
	source devel/setup.bash                                       #添加工作空间环境变量
	roslaunch message_transformer message_transformer.launch      #启动通信功能包节点
	```
				
2. 打开一个新的终端，使用ROS中的rostopic命令**查看机器狗状态信息**：
	
	```bash
	rostopic info xxxxxx
	rostopic echo xxxxxx     # xxxxxx指的是具体话题名称，可在自己的代码中订阅话题进行二次开发
	```
		
3. 使用`/cmd_vel`话题**向运动主机下发速度指令**，话题消息类型`geometry_msgs/Twist`定义如下：
	
		```bash
		geometry_msgs/Vector3 linear   # 线速度(m/s)
			float64 x              # 前向速度，向前为正
			float64 y              # 侧向速度，向左为正
			float64 z	       # 无效参数
		geometry_msgs/Vector3 angular  # 角速度(rad/s)
			float64 x	       # 无效参数
			float64 y	       # 无效参数
			float64 z              # 转向角速度，左转为正
		```
	- 用户可在基于ROS编译的C++和Python程序中发布该话题(需要用户具有ROS基础，ROS基础的学习请参考 http://wiki.ros.org/ROS/Tutorials )，也可以打开一个终端，输入命令发布进行调试，请先输入： 

		```bash
		rostopic pub /cmd_vel geometry_msgs/Twist
		```

	- 输入完先不运行，在语句后面加一个空格，再按Tab键，就会自动补充当前消息类型的内容，具体如下：  

		```bash
		rostopic pub /cmd_vel geometry_msgs/Twist "linear:
		x: 0.0
		y: 0.0
		z: 0.0
		angular:
		x: 0.0
		y: 0.0
		z: 0.0
		"
		```
	- 使用键盘上的向左/向右方向键移动光标，对速度值进行修改，然后在`geometry_msgs/Twist`之后加入` -r 10` 规定发布频率（即每秒发布10次），输入完毕后终端内容如下所示：

		```bash
		rostopic pub /cmd_vel geometry_msgs/Twist -r 10 "linear:
		x: 0.2
		y: 0.1
		z: 0.0
		angular:
		x: 0.0
		y: 0.0
		z: 0.3
		"
		```

	- 运行命令，即可发布话题。

	- 传输程序会订阅该话题，并将其转为UDP指令消息发给运动主机。

	- 在传输程序已正常开启的情况下，用App切入自动模式，机器狗即可按照如上速度行动，为防止在调试过程中对人或物品造成损伤，请在空旷处调试，并随时准备将机器狗切回手动模式进行接管。  

### 二次开发说明

#### 程序结构

```bash
~/message_transformer_ws/src/message_transformer
├── CMakeLists.txt 
└── message_transformer
    ├── CMakeLists.txt
    ├── include
    │   ├── input.h
    │   └── moving_average.h
    ├── launch
    │   └── message_transformer.launch
    ├── package.xml
    └── src
        ├── input.cpp
        ├── moving_average.cpp
        ├── nx2app.cpp
        ├── qnx2ros.cpp
        └── ros2qnx.cpp
```

- ***nx2app.cpp***主要用于感知主机与手柄App之间进行UDP通信，App下发指令码到感知主机，该程序根据收到的命令执行相应的操作。手柄下发指令码结构体如下：

	```c
	//AI switch control command
	struct AiSwitch
	{
		int code;				  //Instruction code
		int size;				  //Command value
		int cons_code;				  //Instruction Type
	};
	```

- ***qnx2ros.cpp***用于接收运动主机上报的数据，并将其转化为ROS话题，供其他功能包调用。

- ***ros2qnx.cpp***订阅其他功能包节点发布的话题，转化为UDP数据报下发给运动主机。

