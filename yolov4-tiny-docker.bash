#!/bin/bash
# wget -q https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.weights -P /home/ros2_ws/src/darknet_ros/darknet_ros/yolo_network_config/weights/
source /opt/ros/foxy/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch darknet_ros yolov4-tiny.launch.py