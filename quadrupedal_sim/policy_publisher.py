import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
from cv_bridge import CvBridge
import cv2
from functools import partial
import h5py
import os

# constants
JOINT_NAMES = [
    'panda_joint1',
    'panda_joint2',
    'panda_joint3',
    'panda_joint4',
    'panda_joint5',
    'panda_joint6',
    'panda_joint7',
    'joint_world_to_body']
CAMERA_NAMES = ['camera_body', 'camera_hand', 'camera_overview', 'camera_table']


def main(args=None):
    rclpy.init(args=args)
    node = DatasetGatherer()
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger('quadrupedal_sim_policy_publisher').info('Done')
    node.destroy_node()
    rclpy.shutdown()


class DatasetGatherer(Node):
    def __init__(self):
        super().__init__('quadrupedal_sim_policy_publisher')

        # params
        self.declare_parameter('dir', '.')
        self.declare_parameter('fs', 30.0)
        self.declare_parameter('ns', 300)
        self.dataset_dir = os.path.expanduser(self.get_parameter('dir').get_parameter_value().string_value)
        self.sampling_frequency = self.get_parameter('fs').get_parameter_value().double_value
        self.max_timestep = self.get_parameter('ns').get_parameter_value().integer_value

        # timer
        self.timer = self.create_timer(1.0 / self.sampling_frequency, self.timer_callback)

        # joint state subscriber
        self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)

        # image subscribers
        for camera_name in CAMERA_NAMES:
            self.create_subscription(Image, camera_name + '/image_raw', partial(self.image_callback, camera_name), 10)

        # publishers
        self.arm_joint_command_publisher = self.create_publisher(JointTrajectory,
                                                                 'panda_joint_trajectory_controller/joint_trajectory',
                                                                 10)
        self.base_joint_command_publisher = self.create_publisher(Float64MultiArray,
                                                                  'base_joint_position_controller/commands', 10)

        # buffered data
        self.joint_angles_buffered = []
        self.joint_velocities_buffered = []
        self.images_buffered = {}

        # cv bridge
        self.cv_bridge = CvBridge()

        # data records
        self.data_dict = {
            '/observations/qpos': [],
            '/observations/qvel': [],
            '/action': []}
        for camera_name in CAMERA_NAMES:
            self.data_dict[f'/observations/images/{camera_name}'] = []

    @staticmethod
    def make_save_file_name(dir):
        i = 0
        while True:
            episode_name = f'episode_{i}'
            file_path = os.path.join(dir, episode_name + '.hdf5')
            dump_dir = os.path.join(dir, episode_name + '_dump')
            if not os.path.isfile(file_path):
                return file_path, dump_dir
            i += 1

    @staticmethod
    def dump_list_of_lists(path, data):
        with open(path, 'w') as f:
            for list in data:
                f.write(','.join([str(x) for x in list]) + '\n')

    def joint_state_callback(self, msg):
        joint_angles_buffered = []
        joint_velocities_buffered = []
        for i, name in enumerate(msg.name):
            if name in JOINT_NAMES:
                joint_angles_buffered.append(msg.position[i])
                joint_velocities_buffered.append(msg.velocity[i])

        self.joint_angles_buffered = joint_angles_buffered
        self.joint_velocities_buffered = joint_velocities_buffered

    def image_callback(self, image_name, msg):
        self.images_buffered[image_name] = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def timer_callback(self):
        # check all data buffered
        if not np.all(self.joint_angles_buffered) or len(self.images_buffered) != len(CAMERA_NAMES):
            return

        # ml inference
        # TODO: replace placeholder
        joint_angles = [0.0]*8  # in order of 'panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7','joint_world_to_body'

        # publish commands
        self.get_logger().info("publishing")
        # publish elevation command
        msg = Float64MultiArray()
        msg.data.append(joint_angles[7])
        self.base_joint_command_publisher.publish(msg)

        # publish arm command
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = JOINT_NAMES[:7]
        t = 0.000
        msg.points.append(
            JointTrajectoryPoint(
                positions=joint_angles[:7],
                velocities=[0] * 7,
                time_from_start=Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))))
        t = 0.001
        msg.points.append(
            JointTrajectoryPoint(
                positions=joint_angles[:7],
                velocities=[0] * 7,
                time_from_start=Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))))
        self.arm_joint_command_publisher.publish(msg)





if __name__ == '__main__':
    main()
