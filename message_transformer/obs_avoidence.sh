gnome-terminal -t "elevation1" -x bash -c "cd /home/ysc/realsense-driver-jetson; source devel/setup.bash; roslaunch realsense2_camera rs_camera_424_240.launch; exec bash;"
sleep 5
gnome-terminal -t "elevation2" -x bash -c "cd /home/ysc/height_map_v1_ws;source devel/setup.bash; roslaunch deeprobotics_local_height_map height_map_safety_v1.launch; exec bash;"


