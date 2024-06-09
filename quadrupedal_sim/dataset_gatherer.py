import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
from cv_bridge import CvBridge
import cv2

JOINT_NAMES = [
    'panda_joint1',
    'panda_joint2',
    'panda_joint3',
    'panda_joint4',
    'panda_joint5',
    'panda_joint6',
    'panda_joint7',
    'joint_world_to_body']

class DatasetGatherer(Node):
    def __init__(self):
        super().__init__('quadrupedal_sim_controller')

        # timer
        sampling_frequency = 30.0
        self.timer = self.create_timer(1.0/sampling_frequency, self.timer_callback)

        # joint state subscriber
        self.joint_state_subscriber = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 1)

        # image subscribers
        self.image_body_subscriber = self.create_subscription(Image, 'camera_body/image_raw', self.image_body_callback, 1)
        self.image_body_subscriber = self.create_subscription(Image, 'camera_hand/image_raw', self.image_hand_callback, 1)

        # buffered data
        self.joint_state_msg_buffered = None
        self.image_body_buffered = None
        self.image_hand_buffered = None

        # cv bridge
        self.cv_bridge = CvBridge()

    def joint_state_callback(self, msg):
        self.joint_state_msg_buffered = msg

    def image_body_callback(self, msg):
        self.image_body_buffered = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def image_hand_callback(self, msg):
        self.image_hand_buffered = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def timer_callback(self):
        # check all data buffered
        if self.joint_state_msg_buffered is None or self.image_hand_buffered is None or self.image_body_buffered is None:
            return

        # extract joint angles of interest
        joint_angles = {}
        for i, name in enumerate(self.joint_state_msg_buffered.name):
            if name in JOINT_NAMES:
                joint_angles[name] = self.joint_state_msg_buffered.position[i]
        print(joint_angles)

        # save test images
        cv2.imwrite("body.png", self.image_body_buffered)
        cv2.imwrite("hand.png", self.image_hand_buffered)

def main(args=None):
    rclpy.init(args=args)
    node = DatasetGatherer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
