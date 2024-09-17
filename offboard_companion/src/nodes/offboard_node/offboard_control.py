#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleOdometry
from geometry_msgs.msg import PoseStamped

from std_srvs.srv import SetBool
from polibax_interfaces.srv import Takeoff, Land, MoveTo

from scipy.spatial.transform import Rotation as R
import numpy as np

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control')

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
            # OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/chotto/fmu/in/trajectory_setpoint', qos_profile)
            # TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/chotto/fmu/in/vehicle_command', qos_profile)
            # VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleLocalPosition, '/chotto/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
            # VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/chotto/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
            # VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_odometry = VehicleOdometry()
        self.vehicle_status = VehicleStatus()
        self.target_x = None
        self.target_y = None
        self.target_z = -0.
        self.target_yaw = 0.0

        self.do_takeoff = False # False sets the vehicle to land | True sets the vehicle to arm-offboard-takeoff to target_z

        # define subscriber for target position
        # self.target_pose_subscriber = self.create_subscription(
        #     PoseStamped, '/chotto/offboard/pose_target', self.target_pose_callback, qos_profile)

        self.target_error_publisher = self.create_publisher(
            PoseStamped, '/chotto/offboard/pose_error', qos_profile)

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.timer_status = self.create_timer(1., self.timer_status_callback)

        self.takeoff_srv = self.create_service(Takeoff, '/chotto/offboard/takeoff', self.takeoff_callback)
        self.land_srv = self.create_service(Land, '/chotto/offboard/land', self.land_callback)
        self.land_srv = self.create_service(MoveTo, '/chotto/offboard/moveto', self.moveto_callback)

        self.min_x = -2.
        self.max_x = 18.
        self.min_y = -2.
        self.max_y = 18.
        self.min_z = -3.
        self.max_z = 0.

    def moveto_callback(self, request, response):
        """ accept call only if in offboard mode """
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and \
           self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED and \
            self.check_valid_target(request.x, request.y, -request.z):
            self.target_x = request.x
            self.target_y = request.y
            self.target_z = -request.z
            self.target_yaw = request.yaw
            response.success = True
        else:
            if self.check_valid_target(request.x, request.y, -request.z):
                self.get_logger().info(f"Invalid target for moveto: {request.x, request.y, -request.z}")
            response.success = False
        return response

    def takeoff_callback(self, request, response):
        """Callback function for the takeoff service."""
        valid_target = self.check_valid_target(self.vehicle_odometry.position[0], self.vehicle_odometry.position[1], -request.height)
        if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_STANDBY and \
            valid_target:

            self.target_x = float( self.vehicle_odometry.position[0] )
            self.target_y = float( self.vehicle_odometry.position[1] )
            self.target_z = -request.height
            yaw = R.from_quat([
                self.vehicle_odometry.q[0], 
                self.vehicle_odometry.q[1], 
                self.vehicle_odometry.q[2], 
                self.vehicle_odometry.q[3]]).as_euler('xyz')[2]
            self.target_yaw = yaw
            print(f"Takeoff to: {self.target_x, self.target_y, self.target_z, self.target_yaw}")
            self.do_takeoff = True
            response.success = True
            self.offboard_setpoint_counter = 0
        else:
            if not valid_target:
                self.get_logger().info(f"Invalid target for takeoff: {self.vehicle_odometry.position[0], self.vehicle_odometry.position[1], -request.height}")
            response.success = False
        return response

    def land_callback(self, request, response):
        """Callback function for the takeoff service."""
        valid_target = self.check_valid_target(self.vehicle_odometry.position[0], self.vehicle_odometry.position[1], 0.)
        if valid_target:
            self.target_x = self.vehicle_odometry.position[0]
            self.target_y = self.vehicle_odometry.position[1]
            self.target_z = 0.
            self.do_takeoff = False
            print(f"Land to: {self.target_x, self.target_y, self.target_z}")
            response.success = True
            self.land()
        else:
            self.get_logger().info(f"Invalid target for landing: {self.vehicle_odometry.position[0], self.vehicle_odometry.position[1], 0.}")
            response.success = False
        return response

    def publish_target_error(self):
        """Publish the error between the target and the current position."""
        msg = PoseStamped()
        msg.pose.position.x = float( self.target_x - self.vehicle_odometry.position[0] )
        msg.pose.position.y = float( self.target_y - self.vehicle_odometry.position[1] )
        msg.pose.position.z = float( self.target_z - self.vehicle_odometry.position[2] )
        self.target_error_publisher.publish(msg)

    # def target_pose_callback(self, msg):
    #     self.target_x = msg.pose.position.x
    #     self.target_y = msg.pose.position.y
    #     self.target_z = msg.pose.position.z
    #     self.target_yaw = R.from_quat([
    #             msg.pose.orientation.x, 
    #             msg.pose.orientation.y, 
    #             msg.pose.orientation.z, 
    #             msg.pose.orientation.w
    #         ]).as_euler('xyz')[2]
    #     # q_ = R.from_euler('xyz', [0, 0, yaw]).as_quat()
    #     # self.target_qx = q_[0]
    #     # self.target_qy = q_[1]
    #     # self.target_qz = q_[2]
    #     # self.target_qw = q_[3]

    def vehicle_odometry_callback(self, vehicle_odometry):
        """Callback function for vehicle_odometry topic subscriber."""
        self.vehicle_odometry = vehicle_odometry

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def timer_status_callback(self):
        """Callback function for the timer."""
        self.get_logger().info(f"Vehicle State: {self.vehicle_status.arming_state}, Preflight Checks: {self.vehicle_status.pre_flight_checks_pass}, Nav State: {self.vehicle_status.nav_state}")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

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
        self.get_logger().info("Switching to land mode")

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
        # self.get_logger().info(f"Publishing position setpoints {[self.target_x, self.target_y, self.target_z]}")

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()
        if self.do_takeoff:

            if self.offboard_setpoint_counter == 10:
                self.engage_offboard_mode()
                self.arm()

            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.publish_position_setpoint()
                self.publish_target_error()


            if self.offboard_setpoint_counter < 11:
                self.offboard_setpoint_counter += 1
        elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            self.offboard_setpoint_counter = 0
        
    def check_valid_target(self, x, y, z):
        return self.min_x <= float( x ) <= self.max_x and \
               self.min_y <= float( y ) <= self.max_y and \
               self.min_z <= float( z ) <= self.max_z


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
