# darknet-ros-fp16

darknet_ros + ROS2 Humble + OpenCV4 + CUDA 11 + __CUDNN (FP16)__ :fire::fire::fire:

- [English (GitHub Wiki)](https://github.com/Ar-Ray-code/darknet_ros_fp16/wiki/Darknet_ros_FP16-Report-(1.3x-faster)-%F0%9F%94%A5)
- [Japanese (zenn)](https://zenn.dev/array/articles/4c82fc8382e62d)

<br>

## Update

- May 1st 2022 Update
  - Support Ampere arch (including Jetson Orin)
- July 23th 2022 Update
  - Support YOLOv7-tiny

<br>

## Main changes

- __Support for YOLO v7__ : Switched the submodule to the master branch of [AlexeyAB/darknet.](https://github.com/AlexeyAB/darknet)
- __Removed IPL__ : Switched from IPL to CV::Mat for OpenCV4 support.
- __Support cuDNN + FP16__

<!-- ## Try on Docker :whale:

[DockerHub](https://hub.docker.com/r/ray255ar/darknet-ros-fp16) -->

<br>

## Requirements

- ROS2 (tested on Humble)
- CUDA10 or later
  - If not, it will automatically turn off
- OpenCV
- v4l2-camera (Connect to `/dev/video*`)
- NVIDIA Graphics Card (Volta , Turing , Ampere)
<!-- - Docker + [NVIDIA-Docker](https://github.com/NVIDIA/nvidia-docker)
  - This docker image is using `cuda:11.7` . -->
- xhost (To install xhost , run `$ sudo apt install xorg` .)
- cuDNN (Ubuntu 20.04)

<br>


## Installation 🐢

### Installation

```bash
$ sudo apt install ros-humble-desktop ros-humble-v4l2-camera
$ source /opt/ros/humble/setup.bash
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
$ git clone --recursive https://github.com/Ar-Ray-code/darknet_ros_yolov4.git
$ darknet_ros_yolov4/darknet_ros/rm_darknet_CMakeLists.sh
$ cd ~/ros2_ws
$ colcon build --symlink-install
```

### NVIDIA-Docker

- Driver version : 515.65.01
- NVIDIA Docker2
- NVIDIA Graphics card (Tested : RTX3060Ti)

```bash
git clone https://github.com/Ar-Ray-code/darknet_ros_fp16.git
docker build -t darknet_ros_fp16 ./darknet_ros_fp16/.

# connect webcamera
docker run --rm -it --device /dev/video0:/dev/video0:mwr -e DISPLAY=$DISPLAY --gpus all -v /tmp/.X11-unix:/tmp/.X11-unix darknet_ros_fp16 /bin/bash
```

### Edit CMakeLists.txt

#### Options

When each option is turned off, the respective compile option will be disabled. This item is for benchmarking purposes, as it will be automatically disabled if the required libraries are not installed.

```cmake
set(CUDA_ENABLE ON)
set(CUDNN_ENABLE ON)
set(FP16_ENABLE ON)
```

#### cuDNN (FP16)

Darknet can be made even faster by enabling CUDNN_HALF(FP16), but you need to specify the appropriate architecture.

FP16 is automatically enabled for GPUs of the Turing or Ampere architecture if the appropriate cuDNN is installed. To disable it, change line 12 to `set(FP16_ENABLE OFF)`.

The Jetson AGX Xavier and TITAN V support FP16, but due to their Volta architecture, auto-detection is not possible. (Sorry... :( )

In that case, please comment out line 17 `set(CMAKE_CUDA_ARCHITECTURES 72)`

#### Download darknet weights

Since the weights to be downloaded are large, you can select the weights to be downloaded by the options.

```cmake
set(DOWNLOAD_YOLOV2_TINY ON)　 # default : on
set(DOWNLOAD_YOLOV3 OFF)       # default : off
set(DOWNLOAD_YOLOV4 ON)      　# default : on
set(DOWNLOAD_YOLOV4_CSP OFF) 　# default : off
set(DOWNLOAD_YOLOV4_TINY OFF)  # default : on
set(DOWNLOAD_YOLOV4_MISH OFF)　# default : off
set(DOWNLOAD_YOLOV7_TINY ON)　 # default : on
```



### Demo

Connect your webcam to your PC.

```bash
$ source /opt/ros/humble/setup.bash
$ source ~/ros2_ws/install/local_setup.bash
$ ros2 launch darknet_ros demo-v4-tiny.launch.py
```

![example](https://user-images.githubusercontent.com/67567093/117596596-a2c8db00-b17e-11eb-90f9-146212e64567.png)



## Performance

Using YOLO v4 consumes a lot of GPU memory and lowers the frame rate, so you need to pay attention to your PC specs.

### Test Machine

| Topics | Spec                                    |
| ------ | --------------------------------------- |
| CPU    | Intel Core i9 12900KF                   |
| RAM    | 64GB DDR4                               |
| GPU    | NVIDIA GeForce RTX 2080 Ti (GDDR6 11GB) |
| Driver | 495.29.05                               |

### Performance (using cuDNN FP16)

YOLO v4 : 48fps

Scaled YOLO v4 : 80fps

YOLO v4-tiny : 215fps

YOLO v4x-mish : 32fps

YOLO v2-tiny : 205fps (Min : 24fps)

YOLOv7-tiny : 160fps (cudnn_half = 0)

> Note : YOLOv2-tiny is deprecated.

![E2tRQvnUcAQqn8O](https://user-images.githubusercontent.com/67567093/121984014-35d3e100-cdcd-11eb-9959-b1063a9a0b2b.jpeg)


## YOLOv7 🚀

```bash
git clone https://github.com/Ar-Ray-code/darknet_ros_fp16 --recursive ~/darknet_ws/src/darknet_ros_fp16
darknet_ws/src/darknet_ros_fp16/darknet_ros/rm_darknet_CMakeLists.sh

source /opt/ros/humble/setup.bash
cd ~/darknet_ws/
colcon build --symlink-install
source install/setup.bash

ros2 launch darknet_ros yolov7.launch.py
```


## Acknowledgment
 I am not a good programmer, but I was able to implement it with the help of many repositories. Thank you to [AlexeyAB](https://github.com/AlexeyAB)'s [darknet](https://github.com/AlexeyAB/darknet) , [legged robotics](https://github.com/leggedrobotics)'s [darknet_ros](https://github.com/leggedrobotics/darknet_ros), and [Tossy0423](https://github.com/Tossy0423/)'s [darknet_ros](https://github.com/Tossy0423/yolov4-for-darknet_ros/) !
