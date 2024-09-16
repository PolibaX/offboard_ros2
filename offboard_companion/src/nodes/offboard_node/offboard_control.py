#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from geometry_msgs.msg import PoseStamped

from std_srvs.srv import SetBool
from polibax_interfaces.action import TakeOff

from scipy.spatial.transform import Rotation as R

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/chotto/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/chotto/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/chotto/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/chotto/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/chotto/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = -0.
        self.target_yaw = 0.0

        # define subscriber for target position
        self.target_pose_subscriber = self.create_subscription(
            PoseStamped, '/chotto/offboard/pose_target', self.target_pose_callback, qos_profile)

        self.target_error_publisher = self.create_publisher(
            PoseStamped, '/chotto/offboard/pose_error', qos_profile)

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.timer_status = self.create_timer(0.5, self.timer_status_callback)

        self.srv_ = self.create_service(SetBool, 'offboard_ready', self.am_ready_callback)

    def am_ready_callback(self, request, response):
        response.success = True
        # response.message = 'I am ready'
        return response

    def publish_target_error(self):
        """Publish the error between the target and the current position."""
        msg = PoseStamped()
        msg.pose.position.x = self.target_x - self.vehicle_local_position.x
        msg.pose.position.y = self.target_y - self.vehicle_local_position.y
        msg.pose.position.z = self.target_z - self.vehicle_local_position.z
        self.target_error_publisher.publish(msg)

    def target_pose_callback(self, msg):
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.target_z = msg.pose.position.z
        self.target_yaw = R.from_quat([
                msg.pose.orientation.x, 
                msg.pose.orientation.y, 
                msg.pose.orientation.z, 
                msg.pose.orientation.w
            ]).as_euler('xyz')[2]
        # q_ = R.from_euler('xyz', [0, 0, yaw]).as_quat()
        # self.target_qx = q_[0]
        # self.target_qy = q_[1]
        # self.target_qz = q_[2]
        # self.target_qw = q_[3]

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def timer_status_callback(self):
        """Callback function for the timer."""
        self.get_logger().info(f"Veichle State: {self.vehicle_status.arming_state}")

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().infovehicle_command_publisher("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [
            self.target_x, self.target_y, self.target_z
        ]
        # msg.yaw = 1.57079  # (90 degree)
        msg.yaw = self.target_yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        do_arm = self.get_parameter('/chotto_puppet/arm').get_parameter_value().bool_value
        if do_arm:
            self.publish_offboard_control_heartbeat_signal()

            if self.offboard_setpoint_counter == 10:
                self.engage_offboard_mode()
                self.arm()

            # if self.vehicle_local_position.z > self.takeoff_height and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.publish_position_setpoint()
                self.publish_target_error()


            if self.offboard_setpoint_counter < 11:
                self.offboard_setpoint_counter += 1
        elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND
            self.land()


def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
