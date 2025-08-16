#!/bin/bash


docker run -it --rm \
  -v /dev:/dev \
  -v /dev/shm:/dev/shm \
  --net=host \
  microros/micro-ros-agent:$ROS_DISTRO udp4 --port 8888 -v6
