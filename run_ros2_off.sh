docker run --rm -it --net host --ipc host \
	-v ./px4_ros_com/src/examples:/root/ws_sensor_combined/src/px4_ros_com/src/examples \
	-v ./src:/root/ros2_ws/src \
	--name px4_offboard \
	px4_offboard:ros2 /bin/bash
