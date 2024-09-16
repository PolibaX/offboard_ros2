import rclpy
from rclpy.node import Node
#from vicon_bridge.msg import SafeVicon
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
# from tf2_py import Quaternion
import time
from scipy.spatial.transform import Rotation as R
import numpy as np

class ViconPx4Relay(Node):

    def __init__(self):
        super().__init__('vicon_px4_relay_gara')
        self.vision_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/mavros/vision_pose/pose_cov', 10)
        # self.vision_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/mavros/vision_pose/pose_cov', 10)
        # self.vicon_sub = self.create_subscription(SafeVicon, '/vicon/uav1/uav1', self.vicon_callback, 10)
        #self.vicon_sub = self.create_subscription(SafeVicon, '/vicon/Leonardo1/Leonardo1', self.vicon_callback, 10)
        self.zed_sub = self.create_subscription(PoseStamped, '/chotto/zed_node/pose', self.vicon_callback, 10)
        # create pose stamped message without quaternions
        self.NED_pose = PoseWithCovarianceStamped()
        # set timer at 200 Hz
        self.pose_timer = self.create_timer(0.005, self.publish_vision_pose)
        # self.target_setpoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/global', 10)
        self.time = time.time()

    def vicon_callback(self, msg):
        #print("CALLBACK DI VICON PX4 RELAY")
        #print("[Vicon2PX4]: Vicon Callback")
        self.NED_pose.header.stamp = msg.header.stamp
        self.NED_pose.header.frame_id = 'map'
        # convert the vicon data to NED frame
        self.NED_pose.pose.pose.position.x = msg.pose.position.y
        self.NED_pose.pose.pose.position.y = -msg.pose.position.x
        self.NED_pose.pose.pose.position.z = msg.pose.position.z
        #self.NED_pose.pose.pose.position.x = msg.transform.translation.x
        #self.NED_pose.pose.pose.position.y = msg.transform.translation.y
        #self.NED_pose.pose.pose.position.z = msg.transform.translation.z
        # print(msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w)
        # convert vicon quaternion to euler angles
        roll, pitch, yaw = R.from_quat([msg.pose.orientation.x, \
                                        msg.pose.orientation.y, \
                                        msg.pose.orientation.z, \
                                        msg.pose.orientation.w]).as_euler('xyz')
        # roll, pitch, yaw = Quaternion([msg.transform.rotation.x, \
        #                                 msg.transform.rotation.y, \
        #                                 msg.transform.rotation.z, \
        #                                 msg.transform.rotation.w]).to_euler()
        #yaw = yaw #- np.pi/2 # add 90 degrees offset to yaw
        yaw = yaw - np.pi/2 # add 90 degrees offset to yaw
        # yaw = +np.pi/2 # add 90 degrees offset to yaw
        # convert euler angles to quaternion
        self.NED_pose.pose.pose.orientation.x, \
            self.NED_pose.pose.pose.orientation.y, \
            self.NED_pose.pose.pose.orientation.z, \
            self.NED_pose.pose.pose.orientation.w = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
        # self.NED_pose.pose.pose.orientation.x, \
        #     self.NED_pose.pose.pose.orientation.y, \
        #     self.NED_pose.pose.pose.orientation.z, \
        #     self.NED_pose.pose.pose.orientation.w = Quaternion.from_euler([roll, pitch, yaw])

        # set covariance to trust the vicon data
        # self.NED_pose.pose.covariance[:] = 0
        # self.NED_pose.pose.covariance[0] = 1
        # self.NED_pose.pose.covariance[7] = 1
        # self.NED_pose.pose.covariance[13] = 1
        # self.NED_pose.pose.covariance[18] = 1
        # self.NED_pose.pose.covariance[22] = 1
        # self.NED_pose.pose.covariance[25] = 1
        # self.NED_pose.pose.covariance[27] = 1
        self.NED_pose.pose.covariance = np.eye(6, dtype=np.float32).reshape((1,36)).tolist()[0]


    def publish_vision_pose(self):
        #print("[Vicon2PX4]: Publishing")
        self.vision_pose_pub.publish(self.NED_pose)



def main():
    print('Hi from px4_nodes_test plus.')
    # create a ros2 node and subscribe to the vicon topic
    rclpy.init()
    vicon_px4_relay = ViconPx4Relay()
    # node = rclpy.create_node('vicon_px4_relay')
    # subscribe to the vicon topic with QoS profile 10
    # pub = node.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 10)
    # node.create_subscription(SafeVicon, '/vicon/uav1/uav1', vicon_callback, 10)
    # spin the node
    try:
        rclpy.spin(vicon_px4_relay)
        # vicon_px4_relay.run()
    except KeyboardInterrupt:
        print("Interrupting gracefully.")
    vicon_px4_relay.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()