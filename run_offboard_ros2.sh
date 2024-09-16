#!/bin/bash
xhost +
docker run --rm -it --privileged --ipc host \
    --net host\
	--runtime nvidia --gpus all \
    -v ./scripts:/root/scripts \
    -v ./offboard_companion/:/root/ws_offboard/src/offboard_companion \
	-v /tmp/.X11-unix/:/tmp/.X11-unix \
	-v ~/.Xauthority:/root/.Xauthority \
    -e XAUTHORITY=/root/.Xauthority \
    -e DISPLAY=$DISPLAY \
    -w /root \
    --name offboard_companion \
    offboard_ros2:v1.14 bash 

