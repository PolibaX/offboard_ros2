#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleOdometry
from geometry_msgs.msg import PoseStamped
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
            depth=1
        )

        # Create publishers
        self.VIO_publisher = self.create_publisher(
            VehicleOdometry, '/chotto/fmu/in/vehicle_visual_odometry', qos_profile)
         

        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/chotto/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        self.zed_sub = self.create_subscription(PoseStamped, '/chotto/zed_node/pose', self.zed_callback, 10)
        # self.zed_sub = self.create_subscription(PoseStamped, '/mini_zed_wrapper/pose', self.zed_callback, 10)
        self.FRD_pose = VehicleOdometry()
        self.FRD_pose.pose_frame = 2
        """
            DEPRECATED with new message (and version/pkgs)
            self.FRD_pose.header.frame_id = 'map'
        """

        # Initialize variables
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        # Create a timer to publish VIO data (VIO=Visual Inertial Odometry)
        self.timer = self.create_timer(1/50, self.timer_callback)

    def zed_callback(self, msg):
        # self.FRD_pose.timestamp = msg.header.stamp.sec*100
        # convert the vicon data to FRD frame

        # self.FRD_pose.position = [msg.pose.position.x, -msg.pose.position.y, -msg.pose.position.z] # from FLU to FRD
        self.FRD_pose.position = [msg.pose.position.y, msg.pose.position.x, -msg.pose.position.z] # from RFU to FRD
        # convert vicon quaternion to euler angles
        roll, pitch, yaw = R.from_quat([msg.pose.orientation.x, \
                                        msg.pose.orientation.y, \
                                        msg.pose.orientation.z, \
                                        msg.pose.orientation.w]).as_euler('xyz')
        yaw_FRD = -yaw # from * to FRD
        # convert euler angles to quaternion
        qx, qy, qz, qw = R.from_euler('xyz', [roll, pitch, yaw_FRD]).as_quat()
        self.FRD_pose.q = [qw, qx, qy, qz]
        # self.FRD_pose.pose.covariance = np.eye(6, dtype=np.float32).reshape((1,36)).tolist()[0]
    
    def publish_VIO_data(self):
        """Publish VIO data to the FMU."""
        # msg = VehicleOdometry()
        # msg.position = [x, y, z]
        # msg.q = [1.0, 0.0, 0.0, 0.0]
        #frame FRD
        self.VIO_publisher.publish(self.FRD_pose)
        # self.get_logger().info('VIO data published')

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        # Publish VIO data
        self.publish_VIO_data()
        


def main(args=None) -> None:
    print('Starting relay_pose node...')
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
