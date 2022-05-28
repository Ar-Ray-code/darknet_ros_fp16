#!/bin/bash
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch darknet_ros yolov4-tiny.launch.py