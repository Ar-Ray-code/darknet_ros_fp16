# darknet_ros_yolov4 (with cuDNN)

FP16 performance report has been released.

- [English (GitHub Wiki)](https://github.com/Ar-Ray-code/darknet_ros_fp16/wiki/Darknet_ros_FP16-Report-(1.3x-faster)-%F0%9F%94%A5)

- [Japanese (zenn)](https://zenn.dev/array/articles/4c82fc8382e62d)

---

darknet_ros + ROS1 Noetic + OpenCV4 + CUDA 11.2 + __CUDNN (FP16)__ :fire::fire::fire:

## Main changes
- __Support for YOLO v4__ : Switched the submodule to the master branch of [AlexeyAB/darknet.](https://github.com/AlexeyAB/darknet)
- __Removed IPL__ : Switched from IPL to CV::Mat for OpenCV4 support.
- __Support cuDNN__

## Requirements
- ROS1 Noetic
- OpenCV 4.2
- CUDA 10 or 11 (tested with CUDA 11.2)
- cuDNN 8.1 ([Installation tutorial](https://docs.nvidia.com/deeplearning/cudnn/install-guide/index.html))

## Installation
```bash
sudo apt install ros-noetic-desktop ros-noetic-usb-cam
source /opt/ros/noetic/setup.bash
mkdir -p ~/ros1_ws/src
cd ~/ros1_ws/src
git clone --recursive https://github.com/Ar-Ray-code/darknet_ros_yolov4.git
darknet_ros_yolov4/darknet_ros/rm_darknet_CMakeLists.sh
cd ~/ros1_ws
catkin_make
```
## Edit CMakeLists.txt

### Options

When each option is turned off, the respective compile option will be disabled. This item is for benchmarking purposes, as it will be automatically disabled if the required libraries are not installed.

```
set(CUDA_ENABLE ON)
set(CUDNN_ENABLE ON)
set(FP16_ENABLE ON)
```

### cuDNN (FP16)

Darknet can be made even faster by enabling CUDNN_HALF(FP16), but you need to specify the appropriate architecture. 

FP16 is automatically enabled for GPUs of the Turing or Ampere architecture if the appropriate cuDNN is installed. To disable it, change line 12 to `set(FP16_ENABLE OFF)`.

The Jetson AGX Xavier and TITAN V support FP16, but due to their Volta architecture, auto-detection is not possible. (Sorry... :( )

In that case, please comment out line 17 `set(CMAKE_CUDA_ARCHITECTURES 72)`

## Demo

Connect your webcam to your PC.

```bash
source /opt/ros/noetic/setup.bash
source ~/ros1_ws/devel/setup.bash
roslaunch darknet_ros yolov4-tiny.launch
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

### Performance (Not using cuDNN FP16)

YOLO v3 : 67 fps , uses 1781MB of VRAM

YOLO v4 : 29 fps , uses 3963MB of VRAM

Scaled YOLO v4 : 51 fps , uses 2831MB of VRAM

### Performance (using cuDNN FP16)

YOLO v4 : 40 fps (+10fps)

Scaled YOLO v4 : 60fps (+9fps)

YOLO v4-tiny : 140fps+ (MAX 150fps:fire:)

YOLO v2-tiny : 135fps+



![E2tRQvnUcAQqn8O](https://user-images.githubusercontent.com/67567093/121984014-35d3e100-cdcd-11eb-9959-b1063a9a0b2b.jpeg)


Please give it a try. Thank you.



## Acknowledgment
 I am not a good programmer, but I was able to implement it with the help of many repositories. Thank you to [AlexeyAB](https://github.com/AlexeyAB)'s [darknet](https://github.com/AlexeyAB/darknet) , [legged robotics](https://github.com/leggedrobotics)'s [darknet_ros](https://github.com/leggedrobotics/darknet_ros), and [Tossy0423](https://github.com/Tossy0423/)'s [darknet_ros](https://github.com/Tossy0423/yolov4-for-darknet_ros/) !
