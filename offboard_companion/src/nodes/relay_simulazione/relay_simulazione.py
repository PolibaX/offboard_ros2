#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleOdometry
from geometry_msgs.msg import PoseArray
from scipy.spatial.transform import Rotation as R
import numpy as np

class OffboardControl(Node):
    """Node to send VIO data to the FMU."""

    def __init__(self) -> None:
        super().__init__('VIO_relay')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=2
        )

        # Create publishers
        self.VIO_publisher = self.create_publisher(
            VehicleOdometry, '/chotto/fmu/in/vehicle_visual_odometry', qos_profile)
         

        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/chotto/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        self.pose_subscriber = self.create_subscription(PoseArray, '/gazebo/model/state', self.pose_callback, 10)
        self.vehicle_pose = VehicleOdometry()
        self.vehicle_pose.pose_frame = 2

        # Create a timer to publish VIO data (VIO=Visual Inertial Odometry)
        self.timer = self.create_timer(1/50, self.timer_callback)
    
    def pose_callback(self, pose_array):
        """Callback function for pose_array topic subscriber."""
        # Get the position of the drone
        x = pose_array.poses[1].position.x
        y = pose_array.poses[1].position.y
        z = pose_array.poses[1].position.z
        qx = pose_array.poses[1].orientation.x
        qy = pose_array.poses[1].orientation.y
        qz = pose_array.poses[1].orientation.z
        qw = pose_array.poses[1].orientation.w
        roll, pitch, yaw = R.from_quat([qx,qy,qz,qw]).as_euler('xyz')
        # yaw = yaw - np.pi # subtract 90 degrees offset to yaw        # convert euler angles to quaternion
        qx, qy, qz, qw = R.from_euler('xyz', [roll, pitch, -yaw]).as_quat()
        self.vehicle_pose.position = [x, -y, -z]
        self.vehicle_pose.q = [qw, qx, qy, qz]
        

    def publish_VIO_data(self, x, y, z):
        """Publish VIO data to the FMU."""
        # msg = VehicleOdometry()
        # msg.position = [x, y, z]
        # msg.q = [1.0, -0.0, 0.0, 0.0]
        # #frame FRD
        # msg.pose_frame = 2
        self.VIO_publisher.publish(self.vehicle_pose)
        self.get_logger().info('VIO data published')

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def timer_callback(self) -> None:
        """Callback function for the timer."""

        # Publish VIO data
        self.publish_VIO_data(9.0, 7.0, 0.0)
        


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
