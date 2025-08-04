ROS_DISTRO = jazzy

CONTAINER_IMAGE := polibax/ros-offboard-common:$(ROS_DISTRO)
CONTAINER_NAME := offboard_companion
PERCENT := %
ROOT_DIR := $(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
WORK_DIR := /root/ws_offboard
ROS_DOMAIN_ID := 33

default: run


build: ## Build release container
	@echo "Please build all Docker Images from https://github.com/PolibaX/docker.git"

run-dev: ## Run a disposable development container
	@docker run --rm -it --privileged --ipc host \
		--net host\
		--runtime nvidia --gpus all \
		-v ./scripts:/root/scripts \
		-v ./offboard_companion/:$(WORK_DIR)/src/offboard_companion \
		-v ./ros-interfaces/polibax_interfaces:$(WORK_DIR)/src/polibax_interfaces \
		-v ./cache/build/offboard_companion:$(WORK_DIR)/build/offboard_companion \
		-v ./cache/install/offboard_companion:$(WORK_DIR)/install/offboard_companion \
		-v ./cache/build/polibax_interfaces:$(WORK_DIR)/build/polibax_interfaces \
		-v ./cache/install/polibax_interfaces:$(WORK_DIR)/install/polibax_interfaces \
		-v /tmp/.X11-unix/:/tmp/.X11-unix \
		-v ~/.Xauthority:/root/.Xauthority \
		-e XAUTHORITY=/root/.Xauthority \
		-e ROS_DOMAIN_ID=$(ROS_DOMAIN_ID) \
		-e DISPLAY=$(DISPLAY) \
		-w $(WORK_DIR) \
		--name $(CONTAINER_NAME) \
		$(CONTAINER_IMAGE) \
		bash

run-sim: ## Run a disposable development container
	@docker run --rm -it --privileged --ipc host \
		--net host\
		--runtime nvidia --gpus all \
		-v ./scripts:/root/scripts \
		-v ./offboard_companion/:$(WORK_DIR)/src/offboard_companion \
		-v ./ros-interfaces/polibax_interfaces:$(WORK_DIR)/src/polibax_interfaces \
		-v ./cache/build/offboard_companion:$(WORK_DIR)/build/offboard_companion \
		-v ./cache/install/offboard_companion:$(WORK_DIR)/install/offboard_companion \
		-v ./cache/build/polibax_interfaces:$(WORK_DIR)/build/polibax_interfaces \
		-v ./cache/install/polibax_interfaces:$(WORK_DIR)/install/polibax_interfaces \
		-v /tmp/.X11-unix/:/tmp/.X11-unix \
		-v ~/.Xauthority:/root/.Xauthority \
		-e XAUTHORITY=/root/.Xauthority \
		-e DISPLAY=$(DISPLAY) \
		-e ROS_DOMAIN_ID=$(ROS_DOMAIN_ID) \
		-w $(WORK_DIR) \
		--volume ${ROOT_DIR}/entrypoint.sh:/workspace/entrypoint.sh \
		--name $(CONTAINER_NAME) \
		$(CONTAINER_IMAGE) \
		bash -ci "colcon build && source install/local_setup.bash && ros2 launch offboard_companion offboard_relay_sim.launch.py"

run-irl: ## Run a disposable development container
	@docker run --rm -it --privileged --ipc host \
		--net host\
		--runtime nvidia --gpus all \
		-v ./scripts:/root/scripts \
		-v ./offboard_companion/:$(WORK_DIR)/src/offboard_companion \
		-v ./ros-interfaces/polibax_interfaces:$(WORK_DIR)/src/polibax_interfaces \
		-v ./cache/build/offboard_companion:$(WORK_DIR)/build/offboard_companion \
		-v ./cache/install/offboard_companion:$(WORK_DIR)/install/offboard_companion \
		-v ./cache/build/polibax_interfaces:$(WORK_DIR)/build/polibax_interfaces \
		-v ./cache/install/polibax_interfaces:$(WORK_DIR)/install/polibax_interfaces \
		-v /tmp/.X11-unix/:/tmp/.X11-unix \
		-v ~/.Xauthority:/root/.Xauthority \
		-e XAUTHORITY=/root/.Xauthority \
		-e DISPLAY=$(DISPLAY) \
		-e ROS_DOMAIN_ID=$(ROS_DOMAIN_ID) \
		-w $(WORK_DIR) \
		--volume ${ROOT_DIR}/entrypoint.sh:/workspace/entrypoint.sh \
		--name $(CONTAINER_NAME) \
		$(CONTAINER_IMAGE) \
		bash -ci "colcon build && source install/local_setup.bash && ros2 launch offboard_companion offboard_relay.launch.py"

exec : ## Execute a command in the running container
	@docker exec -it $(CONTAINER_NAME) bash

clean: ## Clean image artifacts
	-docker rmi $(CONTAINER_IMAGE)

help:
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
		| sort \
		| awk 'BEGIN {FS = ":.*?## "}; {printf "%-10s %s\n", $$1, $$2}'

.PHONY: default run run-dev build exec clean help

