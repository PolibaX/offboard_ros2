#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleOdometry
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
import numpy as np
from tf2_ros import TransformListener, Buffer, StaticTransformBroadcaster, TransformStamped


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

        self.FRD_px4_odom_frame = 'chotto/odom_px4_FRD' 
        self.odom_frame = 'chotto/odom'  # Define the global frame for the vehicle odometry
        self.base_frame = 'chotto/base_link'  # Define the base frame for the vehicle odometry
        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)
        # self.tf_timer = self.create_timer(1/51, self.tf_timer_callback)

        self.zed_sub = self.create_subscription(PoseStamped, '/chotto/pose', self.zed_callback, 3)
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

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        # Publish a static transform from the baselink FLU to baselink FRD
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = self.odom_frame
        static_transform.child_frame_id = self.FRD_px4_odom_frame
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        # Convert the rotation from FLU to FRD
        tmp = R.from_euler('xyz', [np.pi, 0., 0.]).as_quat()
        static_transform.transform.rotation.x = tmp[0]
        static_transform.transform.rotation.y = tmp[1]
        static_transform.transform.rotation.z = tmp[2]
        static_transform.transform.rotation.w = tmp[3]
        self.tf_static_broadcaster.sendTransform(static_transform)
    
    def tf_timer_callback(self):
        """Callback function for the TF timer."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.FRD_px4_odom_frame, self.base_frame, rclpy.time.Time())
            if transform is not None:
                self.FRD_pose.position = [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                ]
                q = transform.transform.rotation
                self.FRD_pose.q = [q.w, q.x, q.y, q.z]
                # self.FRD_pose.pose.covariance = np.eye(6, dtype=np.float32).reshape((1, 36)).tolist()[0]
            else:
                self.get_logger().warn("Transform not found, using last known pose.")
        except Exception as e:
            self.get_logger().error(f"Error in TF lookup: {e}")

    def zed_callback(self, msg):
        # self.FRD_pose.timestamp = msg.header.stamp.sec*100
        # convert the vicon data to FRD frame

        # Choose your conversion (depending on the zed camera convention)
        # self.FRD_pose.position = [msg.pose.position.x, -msg.pose.position.y, -msg.pose.position.z] # from FLU to FRD
        self.FRD_pose.position = [msg.pose.position.x, -msg.pose.position.y, -msg.pose.position.z] # (VERGOGNA)
        # self.FRD_pose.position = [msg.pose.position.y, msg.pose.position.x, -msg.pose.position.z] # from RFU to FRD
        # convert vicon quaternion to euler angles
        roll, pitch, yaw = R.from_quat([msg.pose.orientation.x, \
                                        msg.pose.orientation.y, \
                                        msg.pose.orientation.z, \
                                        msg.pose.orientation.w]).as_euler('xyz')
        yaw_FRD = -yaw #- np.pi/2# from RFU to FRD
        # convert euler angles to quaternion
        qx, qy, qz, qw = R.from_euler('xyz', [roll, pitch, yaw_FRD]).as_quat()
        self.FRD_pose.q = [qw, qx, qy, qz]
        # self.FRD_pose.pose.covariance = np.eye(6, dtype=np.float32).reshape((1,36)).tolist()[0]
        # self.FRD_pose.position_variance = [0.01,0.01,0.01]
        # self.FRD_pose.orientation_variance = [0.01,0.01,0.01]
    
    def publish_VIO_data(self):
        """Publish VIO data to the FMU."""
        # msg = VehicleOdometry()
        # msg.position = [x, y, z]
        # msg.q = [1.0, 0.0, 0.0, 0.0]
        #frame FRD
        self.VIO_publisher.publish(self.FRD_pose)

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
