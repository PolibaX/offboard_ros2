#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleOdometry
from geometry_msgs.msg import PoseStamped, Pose, Point

from std_srvs.srv import SetBool
from polibax_interfaces.srv import Takeoff, Land, MoveTo

from scipy.spatial.transform import Rotation as R
import numpy as np
from tf2_ros import TransformListener, Buffer, StaticTransformBroadcaster, TransformStamped

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control')

        self.declare_parameter("namespace", "matte")
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('px4_odom_frame', 'FRD_px4_odom')
        self.declare_parameter('baselink_frame', 'x500_depth_0')
        self.declare_parameter('map_frame', 'map')

        # Initialize variables
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        self.odom_frame = f'{self.namespace}/odom'
        self.FRD_px4_odom_frame = f'{self.namespace}/FRD_px4_odom'
        self.baselink_frame = self.get_parameter('baselink_frame').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value

        """
            The baselink frame in simulation cannot be "x500_depth_0/base_link" 
            because the when doing the transform, the distance from footprint
            to the pixhawk would be considered. This means that a takeoff at
            1m would be a takeoff at 1m+<pixhawk-to-footprint-distance> (in this case +~0.23m).
        """
        self.offboard_setpoint_counter = 0
        self.vehicle_odometry = VehicleOdometry()
        self.vehicle_status = VehicleStatus()
        self.setpoint_x = 0.
        self.setpoint_y = 0.
        self.setpoint_z = 0.
        self.setpoint_yaw = 0.0
        
        self.min_x = 0.
        self.max_x = 20.
        self.min_y = 0.
        self.max_y = 10.
        self.min_z = -0.01
        self.max_z = 3.

        self.do_takeoff = False # False sets the vehicle to land | True sets the vehicle to arm-offboard-takeoff to setpoint_z
        self.armed = False
        self.flying = False

        """
            State machine flags:
            - do_takeoff: If True, the vehicle will be able to arm and take off to the setpoint

        """
        
        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, f'/{self.namespace}/fmu/in/offboard_control_mode', qos_profile)
            # OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, f'/{self.namespace}/fmu/in/trajectory_setpoint', qos_profile)
            # TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, f'/{self.namespace}/fmu/in/vehicle_command', qos_profile)
            # VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, f'/{self.namespace}/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
            # VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, f'/{self.namespace}/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
            # VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # define subscriber for target position
        # self.target_pose_subscriber = self.create_subscription(
        #     PoseStamped, '/chotto/offboard/pose_target', self.target_pose_callback, qos_profile)

        self.target_error_publisher = self.create_publisher(
            PoseStamped, f'/{self.namespace}/offboard/pose_error', qos_profile)

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.timer_status = self.create_timer(1., self.timer_status_callback)

        self.takeoff_srv = self.create_service(Takeoff, f'/{self.namespace}/offboard/takeoff', self.takeoff_callback)
        self.land_srv = self.create_service(Land, f'/{self.namespace}/offboard/land', self.land_callback)
        self.land_srv = self.create_service(MoveTo, f'/{self.namespace}/offboard/moveto', self.moveto_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.enable_debug_topics = False
        self.goal_publisher = self.create_publisher(
            PoseStamped, f'/{self.namespace}/offboard/goal', qos_profile)

    def moveto_callback(self, request, response):
        if self.flying:
            valid_target = self.check_valid_target(request.x, request.y, request.z, request.yaw, request.frame_id)
            """ request contains x, y, z, yaw and the frame_id of the setpoint to be reached """
            if (self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and \
                self.armed) and valid_target:
                # Transform the target position to the FRD_px4_odom_frame
                try:
                    self.setpoint_x, \
                    self.setpoint_y, \
                    self.setpoint_z, \
                    self.setpoint_yaw = self.transform_setpoint(
                        request.x, 
                        request.y, 
                        request.z, 
                        request.yaw, 
                        request.frame_id, 
                        target_frame_id=self.FRD_px4_odom_frame)
                except Exception as e:
                    self.get_logger().error(f"Error in TF lookup: {e}")
                    response.success = False
                    return response
                self.setpoint_frame_id = request.frame_id
                response.success = True
                self.get_logger().info(f"Moving to: {self.setpoint_x, self.setpoint_y, self.setpoint_z, self.setpoint_yaw}")
            else:
                if not valid_target:
                    self.get_logger().info(f"Invalid target for moveto: {request.x, request.y, request.z, request.yaw, request.frame_id}")
                else:
                    self.get_logger().warn(f"Vehicle not in offboard mode or not armed: {self.vehicle_status.nav_state}, {self.vehicle_status.arming_state}")
                response.success = False
        else:
            self.get_logger().warn(f"Vehicle not in offboard mode or not armed: {self.vehicle_status.nav_state}, {self.vehicle_status.arming_state}")
            response.success = False
        return response

    def takeoff_callback(self, request, response):
        """Callback function for the takeoff service."""
        if (self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD) and \
            not self.flying:
            vehicle_yaw = R.from_quat([
                    self.vehicle_odometry.q[0], 
                    self.vehicle_odometry.q[1], 
                    self.vehicle_odometry.q[2],
                    self.vehicle_odometry.q[3]
                    ]).as_euler('xyz')[2]
            
            valid_target = self.check_valid_target(
                0., 
                0., 
                request.height,
                vehicle_yaw,
                frame_id=self.baselink_frame)
            if valid_target:
                
                self.get_logger().warn("Arming and taking off")
                self.arm()
                
                if not self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED and \
                    self.vehicle_status.pre_flight_checks_pass and \
                    self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:

                    self.setpoint_x, \
                    self.setpoint_y, \
                    self.setpoint_z, \
                    self.setpoint_yaw = self.transform_setpoint(
                        0., 
                        0., 
                        request.height, 
                        vehicle_yaw,
                        self.baselink_frame, 
                        target_frame_id=self.FRD_px4_odom_frame)
                    self.get_logger().warn(f"Takeoff to: {self.setpoint_x, self.setpoint_y, self.setpoint_z, self.setpoint_yaw}")
                    self.flying = True
                    response.success = True
                else:
                    self.get_logger().warn(f"Vehicle not armed or not in offboard mode: {self.vehicle_status.arming_state}, {self.vehicle_status.nav_state}")
                    response.success = False
            else:
                if not valid_target:
                    self.get_logger().warn(f"Invalid target for takeoff: {self.vehicle_odometry.position[0], self.vehicle_odometry.position[1], request.height}")
                else:
                    self.get_logger().warn(f"Vehicle not in offboard mode or already armed: {self.vehicle_status.nav_state}, {self.vehicle_status.arming_state}")
                response.success = False
        else:
            if self.flying:
                self.get_logger().warn(f"Vehicle already flying: {self.vehicle_status.nav_state}, {self.flying}")
            else:
                self.get_logger().warn(f"Vehicle not in offboard mode. Nav state: {self.vehicle_status.nav_state}")
            response.success = False
        return response

    def land_callback(self, request, response):
        """Callback function for the takeoff service."""
        if (self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD) and \
            self.flying:
            vehicle_yaw = R.from_quat([
                    self.vehicle_odometry.q[0], 
                    self.vehicle_odometry.q[1], 
                    self.vehicle_odometry.q[2],
                    self.vehicle_odometry.q[3]
                    ]).as_euler('xyz')[2]
            valid_target = self.check_valid_target(
                self.vehicle_odometry.position[0], 
                self.vehicle_odometry.position[1], 
                0.,
                vehicle_yaw,
                frame_id=self.baselink_frame)
            if valid_target:
                self.setpoint_x = self.vehicle_odometry.position[0]
                self.setpoint_y = self.vehicle_odometry.position[1]
                self.setpoint_z = 0.
                self.setpoint_yaw = vehicle_yaw
                self.do_takeoff = False
                self.get_logger().warn(f"Land to: {self.setpoint_x, self.setpoint_y, self.setpoint_z}")
                response.success = True
                self.land()
            else:
                self.get_logger().info(f"Invalid target for landing: {self.vehicle_odometry.position[0], self.vehicle_odometry.position[1], 0.}")
                response.success = False
        else:
            self.get_logger().warn(f"Vehicle not in offboard mode or not flying: {self.vehicle_status.nav_state}, {self.flying}")
            response.success = False
        return response

    def publish_target_error(self):
        """Publish the error between the target and the current position."""
        msg = PoseStamped()
        msg.pose.position.x = float( self.setpoint_x - self.vehicle_odometry.position[0] )
        msg.pose.position.y = float( self.setpoint_y - self.vehicle_odometry.position[1] )
        msg.pose.position.z = float( self.setpoint_z - self.vehicle_odometry.position[2] )
        self.target_error_publisher.publish(msg)

    # def target_pose_callback(self, msg):
    #     self.setpoint_x = msg.pose.position.x
    #     self.setpoint_y = msg.pose.position.y
    #     self.setpoint_z = msg.pose.position.z
    #     self.setpoint_yaw = R.from_quat([
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
        # detect change from armed to disarmed state
        if self.vehicle_status.arming_state != vehicle_status.arming_state:
            if vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.get_logger().info("Vehicle armed")
                self.set_armed(True)
            elif vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.get_logger().info("Vehicle disarmed")
                self.set_armed(False)
                self.flying = False
                self.do_takeoff = False
        self.vehicle_status = vehicle_status

    def timer_status_callback(self):
        """Callback function for the timer."""
        self.get_logger().info(f"Arming State: {self.vehicle_status.arming_state}, Preflight Checks: {self.vehicle_status.pre_flight_checks_pass}, Nav State: {self.vehicle_status.nav_state}")

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
        # self.get_logger().info("Switching to offboard mode")

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
        msg.position = [ self.setpoint_x, self.setpoint_y, self.setpoint_z ]
        msg.yaw = self.setpoint_yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        tmp = PoseStamped()
        orient = R.from_euler('xyz', [0, 0, self.setpoint_yaw]).as_quat()
        tmp.pose.position.x = float(self.setpoint_x)
        tmp.pose.position.y = float(self.setpoint_y)
        tmp.pose.position.z = float(self.setpoint_z)
        tmp.pose.orientation.x = float(orient[0])
        tmp.pose.orientation.y = float(orient[1])
        tmp.pose.orientation.z = float(orient[2])
        tmp.pose.orientation.w = float(orient[3])
        tmp.header.stamp = self.get_clock().now().to_msg()
        tmp.header.frame_id = self.FRD_px4_odom_frame
        self.goal_publisher.publish(tmp)
        # self.get_logger().info(f"Publishing position setpoints {[self.setpoint_x, self.setpoint_y, self.setpoint_z]}")
        
    def check_valid_target(self, x, y, z, yaw, frame_id):
        # try:
        x, y, z, _ = self.transform_setpoint(x, y, z, yaw, frame_id, target_frame_id=self.map_frame)
        # except:
        #     self.get_logger().error(f"Transform setpoint failed.")
        #     return False
        # Transform the target position to the map frame
        return self.min_x <= float( x ) <= self.max_x and \
               self.min_y <= float( y ) <= self.max_y and \
               self.min_z <= float( z ) <= self.max_z

    def transform_setpoint(self, x, y, z, yaw, frame_id, target_frame_id):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame_id, 
                frame_id, 
                rclpy.time.Time(), 
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            # Transform the setpoint to the target frame
            setpoint = [x, y, z, *R.from_euler('xyz', [0, 0, yaw]).as_quat()]
            transformed_pose = self.apply_transform(setpoint, transform)
            x, y, z = transformed_pose[:3]
            yaw = R.from_quat(transformed_pose[3:]).as_euler('xyz')[2]
        except Exception as e:
            self.get_logger().error(f"Error in TF lookup: {e}")
            # raise e
        return x, y, z, yaw
    
    def apply_transform(self, setpoint, transform):
        """Apply a transform to a setpoint."""
        # Extract translation and rotation from the transform
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        # Convert quaternion to rotation matrix
        rotation_matrix = R.from_quat([rotation.x, rotation.y, rotation.z, rotation.w]).as_matrix()
        # Apply the rotation and translation to the setpoint
        new_rot = rotation_matrix @ R.from_quat(setpoint[3:]).as_matrix()
        new_trans = np.dot(rotation_matrix, setpoint[:3]) + np.array([
            translation.x, 
            translation.y, 
            translation.z
        ])
        transformed_setpoint = [*new_trans, *R.from_matrix(new_rot).as_quat()]
        return transformed_setpoint

    def set_armed(self, armed):
        self.get_logger().warn(f"Setting armed state to {armed}")
        self.armed = armed
        
    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            # self.arm()
            self.get_logger().warn("Engaging offboard mode")
            self.do_takeoff = True
            self.get_logger().info("Takeoff is now available")
        
            
            
        self.publish_position_setpoint()
        self.publish_target_error()
            

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1
        if  (self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND) and \
            self.offboard_setpoint_counter > 0:
            self.offboard_setpoint_counter = 0
            self.get_logger().warn("Resetting offboard setpoint counter")

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
