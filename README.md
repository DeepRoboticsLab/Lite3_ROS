# Jueying Lite3 Perception Development

## message_transformer

### Introduction

With this package, ROS and UDP messages can be converted to each other.

The data transmission between the perception host and the motion host or app is based on the UDP protocol. The package ***message_transformer_cpp*** can realize that:

- Transform UDP messages sent by motion host into ROS topic messages and publish, and send motion control commands issued by perception host to motion host using UDP;
- Receive control commands from the App to turn on and off some functions on perception host.
	

**ROS topics:**

The package will receive the UDP messages from motion host and publish them to the following topics: 

```html
Leg Odometry Data:		/leg_odom       (nav_msgs::Odometry)
IMU Data:				/imu/data       (sensor_msgs::Imu)
Joint Data:				/joint_states   (sensor_msgs::JointState)
```

The package will subscribe to the following topics and send the topic messages to motion host. 

```html
Velocity Command:		/cmd_vel        (geometry_msgs::Twist)
```


### How to Use

1. Open a new terminal and enter the following codes to **start the nodes** `nx2app`, `rk2ros`, `ros2rk`:

	```bash
	cd message_transformer_ws/                                    # go to the package workspace (/home/ysc/message_transformer_ws)
	source devel/setup.bash                                       # Add workspace environment variables
	roslaunch message_transformer message_transformer.launch      # Launch the related nodes
	```
	
2. Open a new terminal and use `rostopic` command to **check the robot status information**:
	
	```bash
	rostopic info xxxxxx
	rostopic echo xxxxxx     # xxxxxx refers to the topic name, and users can subscribe to the topic for secondary development
	```
	
3. Use the topic `/cmd_vel` to send velocity commands to motion host, in the format of `geometry_msgs/Twist` :
	
	```bash
	geometry_msgs/Vector3 linear	# Linear velocity (m/s)
		float64 x					# Longitudinal velocity: positive value when going forward
		float64 y					# Lateral velocity: positive value when going left
		float64 z					# Invalid parameter
	geometry_msgs/Vector3 angular	# Angular velocity (rad/s)
		float64 x					# Invalid parameter
		float64 y					# Invalid parameter
		float64 z					# Angular velocity: positive value when turning left
	```
	- Users can publish to this topic in C++ or Python programs compiled based on ROS (refer to http://wiki.ros.org/ROS/Tutorials for learning about ROS ). Users can also publish messages to the topic for debugging in terminal. Please first type the following codes in terminal: 

		```bash
		rostopic pub /cmd_vel geometry_msgs/Twist
		```

	- Add a space after the codes and press Tab key to complement the message type as follows:  

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
	- Use the left/right arrow keys on the keyboard to move the cursor, modify the velocity values, and then add ` -r 10` after `geometry_msgs/Twist` to specify the posting frequency (10 times per second) as follows:

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

	- Press Enter key to run it, and the topic messages will be published.

	- This package can subscribe to this topic, transform the topic messages into UDP messages and send them to motion host.

	- After `message_transformer` starts, use the App to switch to auto mode, the robot can act according to the velocity as published. In order to prevent damage to people or objects, please debug in the open space and switch back to the manual mode or press emergency stop in emergency.



### Package Structure

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

- ***nx2app.cpp*** is mainly used for UDP communication between perception host and App. App sends command code to perception host and ***nx2app.cpp*** will execute tasks according to the received command. Structure definition of the command sent by App is as follows:

	```c
	//AI switch control command
	struct AiSwitch
	{
		int code;					//Instruction code
		int size;					//Command value
		int cons_code;				//Instruction Type
	};
	```

- ***qnx2ros.cpp*** is used for receiving the data sent by motion host and transform it into ROS topic messages.

- ***ros2qnx.cpp*** can subscirbe to the topic published by other nodes, transform the messages into UDP data and send them to motion host.
