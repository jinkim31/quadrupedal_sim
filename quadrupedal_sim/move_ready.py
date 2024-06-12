import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

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

JOINT_POS = [0,-3.141592/4, 0.0, -3*3.141592/4, 0.0, 3.141592/2, 3.141592/4]

class ControllerStatePublisher(Node):
    def __init__(self):
        super().__init__('controller_state_publisher')
        ## publish joint trajectory, elevation of base.
        self.traj_publisher = self.create_publisher(JointTrajectory, '/panda_joint_trajectory_controller/joint_trajectory', 10)
        self.gripper_publisher = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self.height_publisher = self.create_publisher(Float64MultiArray, '/base_joint_position_controller/commands', 10)
        self.height = 0.0

    def move_joint(self, theta, duration=0.1):
        # construct a goal message
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        traj.points.append(
            JointTrajectoryPoint(positions=theta, velocities=[0]*7,
                                    time_from_start=Duration(sec=int(duration),
                nanosec=int((duration-int(duration))*1e9)))
            )
        self.traj_publisher.publish(traj)

    def move_base(self, dh = 0.0):
        cmd = Float64MultiArray()
        self.height += dh
        cmd.data = [self.height]
        self.height_publisher.publish(cmd)
        self.get_logger().info('Published base height command : {}'.format(cmd.data[0]))

    def gripperOpen(self, duration=0.02):
        # construct a goal message
        gripper_traj = JointTrajectory()
        gripper_traj.joint_names = ['panda_finger_joint1', 'panda_finger_joint2']
        pos = 0.08
        gripper_traj.points.append(
            JointTrajectoryPoint(positions=[pos]*2, velocities=[0]*2,
                                    time_from_start=Duration(sec=int(duration),
                nanosec=int((duration-int(duration))*1e9)))
            )
        self.gripper_publisher.publish(gripper_traj)
        # gripper_open_pos = 0.04
        # return self.gripperGotoPos(self, gripper_open_pos, timeout=duration)


    def gripperClose(self, duration=3.0):
        # construct a goal message
        gripper_traj = JointTrajectory()
        gripper_traj.joint_names = ['panda_finger_joint1', 'panda_finger_joint2']
        pos = 0.01
        gripper_traj.points.append(
            JointTrajectoryPoint(positions=[pos]*2, velocities=[0]*2,
                                    time_from_start=Duration(sec=int(duration),
                nanosec=int((duration-int(duration))*1e9)))
            )
        self.gripper_publisher.publish(gripper_traj)


def main(args=None):
    rclpy.init(args=args)
    controller_state_publisher = ControllerStatePublisher()
    controller_state_publisher.move_joint(JOINT_POS, duration=1.0)
    controller_state_publisher.move_base(0.0)
    controller_state_publisher.gripperOpen()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
