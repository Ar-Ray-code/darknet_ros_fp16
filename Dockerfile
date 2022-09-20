FROM nvidia/cuda:11.7.0-cudnn8-devel-ubuntu22.04

ENV TZ=Asia/Tokyo
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    apt -y clean && \
    rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

RUN apt update && \
    apt install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt update && \
    apt install -y  -o Dpkg::Options::="--force-confdef" -o Dpkg::Options::="--force-confold" keyboard-configuration && \
    apt install -y ros-humble-desktop && \
    apt install -y python3-colcon-common-extensions && \
    apt install -y ros-humble-v4l2-camera && \
    apt install -y git && \
    apt install -y xterm && \
    apt install -y wget && \
    apt install -y pciutils && \
    apt -y clean && \
    rm -rf /var/lib/apt/lists/*

# mkdir ros2_ws/src
WORKDIR /home/ros2_ws/src
RUN git clone https://github.com/Ar-Ray-code/darknet_ros -b master --recursive
RUN bash darknet_ros/rm_darknet_CMakeLists.sh
WORKDIR /home/ros2_ws/
COPY ./yolov4-tiny-docker.bash /home/ros2_ws/yolov4-tiny-docker.bash

# RUN ln -s /usr/local/cuda/lib64/stubs/libcuda.so /usr/local/cuda/lib64/stubs/libcuda.so.1
RUN LD_LIBRARY_PATH=/usr/local/cuda/lib64/stubs/:$LD_LIBRARY_PATH

# USE Usb Camera
CMD ["bash"]
# ================================
# git clone https://github.com/Ar-Ray-code/darknet_ros_fp16.git
# docker build -t darknet_ros_fp16 ./darknet_ros_fp16/.
# # (connect webcamera)
# docker run --rm -it --device /dev/video0:/dev/video0:mwr -e DISPLAY=$DISPLAY --gpus all -v /tmp/.X11-unix:/tmp/.X11-unix darknet_ros_fp16 /bin/bash