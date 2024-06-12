import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
import h5py

JOINT_NAMES = [
            'panda_joint1', 
            'panda_joint2', 
            'panda_joint3', 
            'panda_joint4', 
            'panda_joint5', 
            'panda_joint6', 
            'panda_joint7'
        ]

CAMERA_NAMES = ['camera_body', 'camera_hand', 'camera_overview', 'camera_table']

class SimController(Node):
    def __init__(self, file_path):
        super().__init__('quadrupedal_sim_controller')
        self.arm_joint_command_publisher = self.create_publisher(JointTrajectory, '/panda_joint_trajectory_controller/joint_trajectory', 10)
        self.base_joint_command_publisher = self.create_publisher(Float64MultiArray, '/base_joint_position_controller/commands', 10)
        self.camera_names = CAMERA_NAMES
        self.arm_joint_names = JOINT_NAMES
        print(self.camera_names)
        print(self.arm_joint_names)

        self.image_pubs = {camera_name: self.create_publisher(Image, f'replay_{camera_name}/image_raw', 10) for camera_name in self.camera_names}

        self.timer = self.create_timer(1/30.0, self.timer_callback)


        # file_path = "/home/minseok/cs477_final_ws/demo_data/episode_4.hdf5"
        self.h5_file = h5py.File(file_path, 'r')
        self.action_data = self.h5_file['action']
        self.image_data = {camera_name: self.h5_file['observations/images'][camera_name] for camera_name in self.camera_names}

        self.current_step = 0
        self.bridge = CvBridge()

    def timer_callback(self):
        if self.current_step >= len(self.action_data):
            self.get_logger().info('Finished publishing all joint trajectory points.')
            self.h5_file.close()
            self.destroy_node()
            return
        
        # publish elevation command
        msg = Float64MultiArray()
        msg.data = [self.action_data[self.current_step, -1].astype('float64')]
        self.base_joint_command_publisher.publish(msg)

        # publish arm command
        duration = 0.1
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.arm_joint_names
        msg.points.append(
            JointTrajectoryPoint(
                positions=self.action_data[self.current_step, :-1].astype('float64').tolist(),
                velocities=[0]*7,
                time_from_start=Duration(sec=int(duration), nanosec=int((duration - int(duration)) * 1e9)))
                )
        self.arm_joint_command_publisher.publish(msg)

        for camera_name in self.camera_names:
            image_np = self.image_data[camera_name][self.current_step]
            image_msg = self.bridge.cv2_to_imgmsg(image_np, encoding="rgb8")
            self.image_pubs[camera_name].publish(image_msg)
            # self.get_logger().info(f'Published step {self.current_step} for {camera_name}')

        self.get_logger().info(f'Publishing recorded data : {self.current_step+1}/{len(self.action_data)}')
        self.current_step += 1

def main(args=None):
    rclpy.init(args=args)

    file_path = "/home/minseok/cs477_final_ws/demo_data/fixed_glue_pose_body_cam_z_030_with_20240612_005549/episode_49.hdf5"

    minimal_publisher = SimController(file_path)
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
