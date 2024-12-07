# Jueying Lite3 Perception Development

[简体中文](./README_ZH.md)

## transfer

### Introduction

With this package, ROS2 and UDP messages can be converted to each other.

The data transmission between the perception host and the motion host or app is based on the UDP protocol. The package ***tarnsfer*** can realize that:

- Transform UDP messages sent by motion host into ROS2 topic messages and publish
- Send motion control commands issued by perception host to motion host using UDP
- Receive control commands from the App to turn on and off some functions on perception host
- Receive the button and joystick status information from the APP and convert it to ROS2 topics for publication.

**ROS2 topics:**

The package will receive the UDP messages from motion host and publish them to the following topics: 

```html
Leg Odometry Data(pose only):		/leg_odom		(geometry_msgs::msg::PoseWithCovarianceStamped)
Leg Odometry Data(pose and velocity):	/leg_odom2		(nav_msgs::msg::Odometry)
IMU Data:				/imu/data		(sensor_msgs::msg::Imu)
Joint Data:				/joint_states		(sensor_msgs::msg::JointState)
```

The package will subscribe to the following topics and send the topic messages to motion host. 

```html
Velocity Command:			/cmd_vel		(geometry_msgs::msg::Twist)
```


### How to Use

1. Open a new terminal and enter the following codes to **start the nodes** `jetson2motion`, `jetson2app`, `sensor_checker`:

	```bash
	source ~/lite_cog_ros2/transfer/devel/setup.bash	# Add workspace environment variables
	roslaunch transfer transfer.launch			# Launch the related nodes
	```

2. Open a new terminal and use `ros2 topic` command to **check the robot status information**:

	```bash
	ros2 topic info xxxxxx
	ros2 topic echo xxxxxx     # xxxxxx refers to the topic name, and users can subscribe to the topic for secondary development
	```
	
3. Use the topic `/cmd_vel` to send velocity commands to motion host, in the format of `geometry_msgs/msg/Twist` :
	
	```bash
	geometry_msgs/msg/Vector3 linear			# Linear velocity (m/s)
		float64 x					# Longitudinal velocity: positive value when going forward
		float64 y					# Lateral velocity: positive value when going left
		float64 z					# Invalid parameter
	geometry_msgs/msg/Vector3 angular			# Angular velocity (rad/s)
		float64 x					# Invalid parameter
		float64 y					# Invalid parameter
		float64 z					# Angular velocity: positive value when turning left
	```
	- Users can publish to this topic in C++ or Python programs compiled based on ROS2 (refer to https://docs.ros.org/en/foxy/Tutorials.html for learning about ROS2 ). Users can also publish messages to the topic for debugging in terminal. Please first type the following codes in terminal: 

    ```bash
    ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear:{x: 0.2, y: 0.1, z: 0.0}, angular:{x: 0.0, y: 0.0, z: 0.3}}"
    ```

	- Press Enter key to run it, and the topic messages will be published.

	- This package can subscribe to this topic, transform the topic messages into UDP messages and send them to motion host.

	- After `transfer` starts, use the App to switch to auto mode, the robot can act according to the velocity as published. In order to prevent damage to people or objects, please debug in the open space and switch back to the manual mode or press emergency stop in emergency.



### Package Structure

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
Contains the AppReceiver class. 
The AppReceiver class receives commands from the handle App, performs corresponding operations, and sends back packets. 
In addition, the AppReceiver also receives the handle status and converts it into ROS topics for publication.

- ***Jetson2Motion.cpp***
Contains the MotionReceiver and MotionSender classes. 
The MotionReceiver class is used to receive data reported by the motion host and convert it into ROS topics for other functional packages to call. 
The MotionSender class is used to subscribe to topics published by other functional package nodes, convert them into UDP datagrams, and send them to the motion host.

- ***SensorChecker.cpp***
Used to subscribe to the sensor data topics of the robot dog and publish sensor status topics.

- ***SensorLogger.cpp***
Used to subscribe to the robot dog’s sensor status topics and save the results.
