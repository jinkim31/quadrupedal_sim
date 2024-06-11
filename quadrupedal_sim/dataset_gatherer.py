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

# configs
DATASET_DIR = '/home/jin/Desktop'
SAMPLING_FREQUENCY = 30.0   # Hz
MAX_TIMESTEPS = 300         # number of timesteps to record

class DatasetGatherer(Node):
    def __init__(self):
        super().__init__('quadrupedal_sim_dataset_gatherer')

        # timer
        self.timer = self.create_timer(1.0/SAMPLING_FREQUENCY, self.timer_callback)

        # joint state subscriber
        self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)

        # image subscribers
        for camera_name in CAMERA_NAMES:
            self.create_subscription(Image, camera_name + '/image_raw', partial(self.image_callback, camera_name), 10)

        # joint command subscribers
        self.create_subscription(JointTrajectory, 'panda_joint_trajectory_controller/joint_trajectory', self.panda_joint_command_callback, 10)
        self.create_subscription(Float64MultiArray, 'base_joint_position_controller/commands', self.base_joint_command_callback, 10)

        # buffered data
        self.joint_angles_buffered = None
        self.joint_velocities_buffered = None
        self.images_buffered = {}
        self.panda_joint_command_buffered = None
        self.base_joint_command_buffered = None

        # cv bridge
        self.cv_bridge = CvBridge()

        # data records
        self.data_dict = {
            '/observations/qpos': [],
            '/observations/qvel': [],
            '/action': [],
        }
        for cam_name in CAMERA_NAMES:
            self.data_dict[f'/observations/images/{cam_name}'] = []

    def joint_state_callback(self, msg):
        angles = []
        velocities = []

        self.joint_angles_buffered = []
        for i, name in enumerate(msg.name):
            if name in JOINT_NAMES:
                angles.append(msg.position[i])
                velocities.append(msg.velocity[i])

        self.joint_angles_buffered = angles
        self.joint_velocities_buffered = velocities

    def panda_joint_command_callback(self, msg):
        self.panda_joint_command_buffered = [x for x in msg.points[-1].positions]

    def base_joint_command_callback(self, msg):
        self.base_joint_command_buffered = msg.data[0]

    def image_callback(self, image_name, msg):
        self.images_buffered[image_name] = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def timer_callback(self):
        # check all data buffered
        if not np.all(self.joint_angles_buffered) or len(self.images_buffered) != len(CAMERA_NAMES) or not self.panda_joint_command_buffered or self.base_joint_command_buffered == None:
            return

        # record data
        self.data_dict['/observations/qpos'].append(self.joint_angles_buffered)
        self.data_dict['/observations/qvel'].append(self.joint_velocities_buffered)
        self.data_dict['/action'].append(np.array(self.panda_joint_command_buffered + [self.base_joint_command_buffered]))
        for camera_name in CAMERA_NAMES:
            self.data_dict[f'/observations/images/{camera_name}'].append(self.images_buffered[camera_name])

        # check record complete
        n_recorded = len(self.data_dict['/action'])
        self.get_logger().info(f'logging: {n_recorded}/{MAX_TIMESTEPS}')
        if n_recorded < MAX_TIMESTEPS:
            return

        # generate .hdf5 dataset file
        dataset_path = os.path.join(DATASET_DIR, f'episode_{0}' + '.hdf5')
        with h5py.File(dataset_path, 'w', rdcc_nbytes=1024 ** 2 * 2) as root:
            root.attrs['sim'] = True
            obs = root.create_group('observations')
            image = obs.create_group('images')
            for camera_name in CAMERA_NAMES:
                _ = image.create_dataset(camera_name, (MAX_TIMESTEPS, 480, 640, 3), dtype='uint8', chunks=(1, 480, 640, 3))
            qpos = obs.create_dataset('qpos', (MAX_TIMESTEPS, len(JOINT_NAMES)))
            qvel = obs.create_dataset('qvel', (MAX_TIMESTEPS, len(JOINT_NAMES)))
            action = root.create_dataset('action', (MAX_TIMESTEPS, len(JOINT_NAMES)))

            for name, array in self.data_dict.items():
                self.get_logger().info(f'saving: {name}')
                root[name][...] = array

        # raise exception to quit spin
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = DatasetGatherer()
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger('quadrupedal_sim_dataset_gatherer').info('Done')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
