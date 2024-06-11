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
        rclpy.logging.get_logger('quadrupedal_sim_dataset_gatherer').info('Done')
    node.destroy_node()
    rclpy.shutdown()


class DatasetGatherer(Node):
    def __init__(self):
        super().__init__('quadrupedal_sim_dataset_gatherer')

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

        # joint command subscribers
        self.create_subscription(JointTrajectory, 'panda_joint_trajectory_controller/joint_trajectory',
                                 self.panda_joint_command_callback, 10)
        self.create_subscription(Float64MultiArray, 'base_joint_position_controller/commands',
                                 self.base_joint_command_callback, 10)

        # buffered data
        self.joint_angles_buffered = []
        self.joint_velocities_buffered = []
        self.images_buffered = {}
        self.panda_joint_command_buffered = None
        self.base_joint_command_buffered = None

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
                f.write(','.join([str(x) for x in list]))
                f.write('\n')

    def joint_state_callback(self, msg):
        self.joint_angles_buffered.clear()
        self.joint_velocities_buffered.clear()
        for i, name in enumerate(msg.name):
            if name in JOINT_NAMES:
                self.joint_angles_buffered.append(msg.position[i])
                self.joint_velocities_buffered.append(msg.velocity[i])

    def panda_joint_command_callback(self, msg):
        self.panda_joint_command_buffered = [x for x in msg.points[-1].positions]

    def base_joint_command_callback(self, msg):
        self.base_joint_command_buffered = msg.data[0]

    def image_callback(self, image_name, msg):
        self.images_buffered[image_name] = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def timer_callback(self):
        # check all data buffered
        if not np.all(self.joint_angles_buffered) or len(self.images_buffered) != len(
                CAMERA_NAMES) or not self.panda_joint_command_buffered or self.base_joint_command_buffered == None:
            return

        # record data
        self.data_dict['/observations/qpos'].append(self.joint_angles_buffered)
        self.data_dict['/observations/qvel'].append(self.joint_velocities_buffered)
        self.data_dict['/action'].append(
            np.array(self.panda_joint_command_buffered + [self.base_joint_command_buffered]))
        for camera_name in CAMERA_NAMES:
            self.data_dict[f'/observations/images/{camera_name}'].append(self.images_buffered[camera_name])

        # check recording complete
        n_recorded = len(self.data_dict['/action'])
        self.get_logger().info(f'logging: {n_recorded}/{self.max_timestep}')
        if n_recorded < self.max_timestep:
            return

        # generate .hdf5 dataset file
        file_path, dump_dir = self.make_save_file_name(self.dataset_dir)
        if not os.path.exists(dump_dir):
            os.makedirs(dump_dir)
        self.get_logger().info(f'saving as file: {file_path}')
        with h5py.File(file_path, 'w', rdcc_nbytes=1024 ** 2 * 2) as root:
            root.attrs['sim'] = True
            root.attrs['fs'] = self.sampling_frequency
            obs = root.create_group('observations')
            image = obs.create_group('images')
            for camera_name in CAMERA_NAMES:
                image.create_dataset(camera_name, (self.max_timestep, 480, 640, 3), dtype='uint8',
                                     chunks=(1, 480, 640, 3))
            obs.create_dataset('qpos', (self.max_timestep, len(JOINT_NAMES)))
            obs.create_dataset('qvel', (self.max_timestep, len(JOINT_NAMES)))
            root.create_dataset('action', (self.max_timestep, len(JOINT_NAMES)))

            for name, array in self.data_dict.items():
                self.get_logger().info(f'saving: {name}')
                root[name][...] = array

        # dump
        for camera_name in CAMERA_NAMES:
            os.mkdir(os.path.join(dump_dir, camera_name))
            for i, image in enumerate(self.data_dict[f'/observations/images/{camera_name}']):
                cv2.imwrite(os.path.join(dump_dir, camera_name, f'{i:04d}.png'), image)
        self.dump_list_of_lists(os.path.join(dump_dir, 'qpos.csv'), self.data_dict['/observations/qpos'])
        self.dump_list_of_lists(os.path.join(dump_dir, 'qvel.csv'), self.data_dict['/observations/qvel'])
        self.dump_list_of_lists(os.path.join(dump_dir, 'action.csv'), self.data_dict['/action'])

        # raise exception to quit spin
        raise SystemExit


if __name__ == '__main__':
    main()
