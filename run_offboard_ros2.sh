#!/bin/bash
xhost +
docker run --rm -it --privileged --ipc host \
    --net host\
	--runtime nvidia --gpus all \
    -v ./scripts:/root/scripts \
    -v ./offboard_companion/:/root/ws_offboard/src/offboard_companion \
    -v ./ros-interfaces/polibax_interfaces:/root/ws_offboard/src/polibax_interfaces \
    -v ./cache/build/offboard_companion:/root/ws_offboard/build/offboard_companion \
    -v ./cache/install/offboard_companion:/root/ws_offboard/install/offboard_companion \
    -v ./cache/build/polibax_interfaces:/root/ws_offboard/build/polibax_interfaces \
    -v ./cache/install/polibax_interfaces:/root/ws_offboard/install/polibax_interfaces \
	-v /tmp/.X11-unix/:/tmp/.X11-unix \
	-v ~/.Xauthority:/root/.Xauthority \
    -e XAUTHORITY=/root/.Xauthority \
    -e DISPLAY=$DISPLAY \
    -w /root/ws_offboard \
    --name offboard_companion \
    offboard_ros2:v1.14_jazzy_cached bash # colcon build --packages-select offboard_companion

