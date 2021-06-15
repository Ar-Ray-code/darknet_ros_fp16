# darknet_ros_yolov4
darknet_ros + ROS2 Foxy + OpenCV4 + CUDA 11.2



I've been wanting to make the ROS2 + YOLO v4 implementation happen for a long time, and I'm happy to report that I was able to implement it.

## Main changes
- __Support for YOLO v4__ : Switched the submodule to the master branch of [AlexeyAB/darknet.](https://github.com/AlexeyAB/darknet)
- __Removed IPL__ : Switched from IPL to CV::Mat for OpenCV4 support.

## Requirements
- ROS2 Foxy
- OpenCV4 ($ sudo apt install ros-foxy-vision-opencv)
- CUDA 10 or 11 (tested with CUDA 11.2)

## Installation
```
$ source /opt/ros/foxy/setup.bash
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
$ git clone --recursive https://github.com/Ar-Ray-code/darknet_ros_yolov4.git
$ darknet_ros_yolov4/darknet_ros/rm_darknet_CMakeLists.sh
$ cd ~/ros2_ws
$ colcon build --symlink-install
```
## Demo

Connect your webcam to your PC.

### Terminal 1
```
$ source /opt/ros/foxy/setup.bash
$ source ~/ros2_ws/install/local_setup.bash
$ ros2 run v4l2_camera v4l2_camera_node --ros-args -r __ns:=/camera/rgb
```
### Terminal 2
```
$ source /opt/ros/foxy/setup.bash
$ source ~/ros2_ws/install/local_setup.bash
$ ros2 run darknet_ros yolov4.launch.py
```
![example](https://user-images.githubusercontent.com/67567093/117596596-a2c8db00-b17e-11eb-90f9-146212e64567.png)

## Performance

Using YOLO v4 consumes a lot of GPU memory and lowers the frame rate, so you need to pay attention to your PC specs.

### Test Machine

| Topics | Spec                                    |
| ------ | --------------------------------------- |
| CPU    | Ryzen7 2700X (@3.7GHz x 16)             |
| RAM    | 16GB DDR4                               |
| GPU    | NVIDIA GeForce RTX 2080 Ti (GDDR6 11GB) |
| Driver | 460.32.03                               |

### Performance

YOLO v3 : 67 fps , uses 1781MB of VRAM

YOLO v4 : 29 fps , uses 3963MB of VRAM

Scaled YOLO v4 : 51 fps , uses 2831MB of VRAM

![E2tRQvnUcAQqn8O](https://user-images.githubusercontent.com/67567093/121984014-35d3e100-cdcd-11eb-9959-b1063a9a0b2b.jpeg)


Please give it a try. Thank you.



## Acknowledgment
 I am not a good programmer, but I was able to implement it with the help of many repositories. Thank you to [AlexeyAB](https://github.com/AlexeyAB)'s [darknet](https://github.com/AlexeyAB/darknet) , [legged robotics](https://github.com/leggedrobotics)'s [darknet_ros](https://github.com/leggedrobotics/darknet_ros), and [Tossy0423](https://github.com/Tossy0423/)'s [darknet_ros](https://github.com/Tossy0423/yolov4-for-darknet_ros/) !
