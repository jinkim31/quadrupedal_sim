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
import argparse
import torch
import pickle

from constants import SIM_TASK_CONFIGS
from policy import ACTPolicy, CNNMLPPolicy

# constants
JOINT_NAMES = [
    'panda_joint1',
    'panda_joint2',
    'panda_joint3',
    'panda_joint4',
    'panda_joint5',
    'panda_joint6',
    'panda_joint7',
    'joint_world_to_body',
    'panda_finger_joint1']
CAMERA_NAMES = ['camera_body', 'camera_hand', 'camera_overview', 'camera_table']

def make_policy(policy_class, policy_config):
    if policy_class == 'ACT':
        policy = ACTPolicy(policy_config)
    elif policy_class == 'CNNMLP':
        policy = CNNMLPPolicy(policy_config)
    else:
        raise NotImplementedError
    return policy

def main(args=None):
    rclpy.init(args=args)
    node = DatasetGatherer()

    ### from act's imitate_episodes.py
    # ckpt_dir = args['ckpt_dir']
    # policy_class = args['policy_class']
    # command line parameters
    ckpt_dir = args['ckpt_dir']
    policy_class = args['policy_class']
    onscreen_render = args['onscreen_render']
    task_name = args['task_name']
    num_epochs = args['num_epochs']
    # num_epochs = args['num_epochs']
    # get task parameters
    is_sim = task_name[:4] == 'sim_'
    if is_sim:
        from constants import SIM_TASK_CONFIGS
        task_config = SIM_TASK_CONFIGS[task_name]
    else:
        from aloha_scripts.constants import TASK_CONFIGS
        task_config = TASK_CONFIGS[task_name]

    episode_len = task_config['episode_len']
    camera_names = task_config['camera_names']

    # fixed parameters
    state_dim = 8
    lr_backbone = 1e-5
    backbone = 'resnet18'
    if policy_class == 'ACT':
        enc_layers = 4
        dec_layers = 7
        nheads = 8
        policy_config = {'lr': args['lr'],
                         'num_queries': args['chunk_size'],
                         'kl_weight': args['kl_weight'],
                         'hidden_dim': args['hidden_dim'],
                         'dim_feedforward': args['dim_feedforward'],
                         'lr_backbone': lr_backbone,
                         'backbone': backbone,
                         'enc_layers': enc_layers,
                         'dec_layers': dec_layers,
                         'nheads': nheads,
                         'camera_names': camera_names,
                         }
    elif policy_class == 'CNNMLP':
        policy_config = {'lr': args['lr'], 
                         'lr_backbone': lr_backbone, 
                         'backbone' : backbone, 
                         'num_queries': 1,
                         'camera_names': camera_names,}
    else:
        raise NotImplementedError

    config = {
        'num_epochs': num_epochs,
        'ckpt_dir': ckpt_dir,
        'episode_len': episode_len,
        'state_dim': state_dim,
        'lr': args['lr'],
        'policy_class': policy_class,
        'onscreen_render': onscreen_render,
        'policy_config': policy_config,
        'task_name': task_name,
        'seed': args['seed'],
        'temporal_agg': args['temporal_agg'],
        'camera_names': camera_names,
        'real_robot': not is_sim
    }

    ckpt_name = 'policy_epoch_1000_seed_0.ckpt'
    ckpt_path = os.path.join(ckpt_dir, ckpt_name)
    node.config = config
    node.policy = make_policy(policy_class, policy_config)
    loading_status = node.policy.load_state_dict(torch.load(ckpt_path))
    print(loading_status)
    node.policy.cuda()
    node.policy.eval()
    print(f'Loaded: {ckpt_path}')
    stats_path = os.path.join(ckpt_dir, f'dataset_stats.pkl')
    with open(stats_path, 'rb') as f:
        stats = pickle.load(f)

    node.pre_process = lambda s_qpos: (s_qpos - stats['qpos_mean']) / stats['qpos_std']
    node.post_process = lambda a: a * stats['action_std'] + stats['action_mean']


    max_timesteps = config['episode_len']
    max_timesteps = int(max_timesteps * 1) # may increase for real-world tasks
    if config['temporal_agg']:
        num_queries = policy_config['num_queries']
        node.all_time_actions = torch.zeros([max_timesteps, max_timesteps+num_queries, state_dim]).cuda()
    
    node.max_timestep = max_timesteps

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

        self.config = None
        self.policy = None
        self.pre_process = None
        self.post_process = None
        self.all_time_actions = None
        self.steps = 0
        self.max_timestep = None
        self.all_actions = None

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

    def get_image(self, camera_names):
        curr_images = []
        for name in camera_names:
            np_image = np.array(self.images_buffered[name], dtype=np.float32)
            np_image /= 255.0
            torch_image = torch.from_numpy(np_image).permute(2, 0, 1)
            curr_images.append(torch_image)
        curr_image = torch.stack(curr_images, dim=0).cuda().unsqueeze(0)
        return curr_image
        
    def timer_callback(self):
        # check all data buffered
        if not np.all(self.joint_angles_buffered) or len(self.images_buffered) != len(CAMERA_NAMES):
            return


        # observations :
        self.joint_angles_buffered # current joint angles in order of 'panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7','joint_world_to_body'
        self.images_buffered # image dictionary with keys 'camera_body', 'camera_hand', 'camera_overview', 'camera_table'
        # action :
        # ml inference
        # TODO: replace placeholder


        ### from act's imitate_episodes.py's eval_bc
    
        policy_config = self.config['policy_config']
        camera_names = self.config['camera_names']
        temporal_agg = self.config['temporal_agg']
        query_frequency = policy_config['num_queries']
        if temporal_agg:
            query_frequency = 1
            num_queries = policy_config['num_queries']

        with torch.inference_mode():

            qpos_numpy = np.array(self.joint_angles_buffered)
            qpos = self.pre_process(qpos_numpy)
            qpos = torch.from_numpy(qpos).float().cuda().unsqueeze(0)
            curr_image = self.get_image(camera_names)
            t = self.steps
            if self.config['policy_class'] == "ACT":
                if t % query_frequency == 0:
                    self.all_actions = self.policy(qpos, curr_image)
                if temporal_agg:
                    self.all_time_actions[[t], t:t+num_queries] = self.all_actions
                    actions_for_curr_step = self.all_time_actions[:, t]
                    actions_populated = torch.all(actions_for_curr_step != 0, axis=1)
                    actions_for_curr_step = actions_for_curr_step[actions_populated]
                    k = 0.01
                    exp_weights = np.exp(-k * np.arange(len(actions_for_curr_step)))
                    exp_weights = exp_weights / exp_weights.sum()
                    exp_weights = torch.from_numpy(exp_weights).cuda().unsqueeze(dim=1)
                    raw_action = (actions_for_curr_step * exp_weights).sum(dim=0, keepdim=True)
                else:
                    raw_action = self.all_actions[:, t % query_frequency]
            self.steps += 1


        ### post-process actions
        raw_action = raw_action.squeeze(0).cpu().numpy()
        joint_angles = self.post_process(raw_action) # in order of 'panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7','joint_world_to_body'

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
        duration = 0.1
        msg.points.append(
            JointTrajectoryPoint(
                positions=joint_angles[:7],
                velocities=[0] * 7,
                time_from_start=Duration(sec=int(duration), nanosec=int((duration - int(duration)) * 1e9))))
        self.arm_joint_command_publisher.publish(msg)

        if self.steps >= self.max_timestep:
            self.get_logger().info("Reached max_timestep. Stop Policy Publisher.")
            self.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--eval', action='store_true')
    parser.add_argument('--onscreen_render', action='store_true')
    parser.add_argument('--ckpt_dir', action='store', type=str, help='ckpt_dir', required=True)
    parser.add_argument('--policy_class', action='store', type=str, help='policy_class, capitalize', required=True)
    parser.add_argument('--task_name', action='store', type=str, help='task_name', required=True)
    parser.add_argument('--batch_size', action='store', type=int, help='batch_size', required=True)
    parser.add_argument('--seed', action='store', type=int, help='seed', required=True)
    parser.add_argument('--num_epochs', action='store', type=int, help='num_epochs', required=True)
    parser.add_argument('--lr', action='store', type=float, help='lr', required=True)

    # for ACT
    parser.add_argument('--kl_weight', action='store', type=int, help='KL Weight', required=False)
    parser.add_argument('--chunk_size', action='store', type=int, help='chunk_size', required=False)
    parser.add_argument('--hidden_dim', action='store', type=int, help='hidden_dim', required=False)
    parser.add_argument('--dim_feedforward', action='store', type=int, help='dim_feedforward', required=False)
    parser.add_argument('--temporal_agg', action='store_true')
    
    main(vars(parser.parse_args()))

# --task_name sim_fixed_glue_position --ckpt_dir ~/act/assets --policy_class ACT --kl_weight 10 --chunk_size 100 --hidden_dim 512 --batch_size 8 --dim_feedforward 3200 --num_epochs 2000 --lr 1e-5 --seed 0