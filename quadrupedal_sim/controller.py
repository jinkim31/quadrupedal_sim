import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np

class SimController(Node):
    def __init__(self):
        super().__init__('quadrupedal_sim_controller')
        self.arm_joint_command_publisher = self.create_publisher(JointTrajectory, 'panda_joint_trajectory_controller/joint_trajectory', 10)
        self.gripper_joint_command_publisher = self.create_publisher(JointTrajectory, 'gripper_controller/joint_trajectory', 10)
        self.base_joint_command_publisher = self.create_publisher(Float64MultiArray, 'base_joint_position_controller/commands', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.04]
        self.arm_joint_names = ['panda_joint'+str(i+1) for i in range(7)]
        print(self.arm_joint_names)

    def timer_callback(self):
        # publish elevation command
        msg = Float64MultiArray()
        msg.data.append(self.joint_positions[0])
        self.base_joint_command_publisher.publish(msg)

        # publish arm command
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.arm_joint_names
        t = 0.000
        msg.points.append(
            JointTrajectoryPoint(
                positions=self.joint_positions[1:8],
                velocities=[0]*7,
                time_from_start=Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))))
        t = 0.001
        msg.points.append(
            JointTrajectoryPoint(
                positions=self.joint_positions[1:8],
                velocities=[0]*7,
                time_from_start=Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))))
        self.arm_joint_command_publisher.publish(msg)

        # publish gripper command
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ['panda_finger_joint1', 'panda_finger_joint2']
        t = 0.000
        msg.points.append(
            JointTrajectoryPoint(
                positions=[self.joint_positions[8]]*2,
                velocities=[0]*2,
                time_from_start=Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))))
        t = 0.001
        msg.points.append(
            JointTrajectoryPoint(
                positions=[self.joint_positions[8]]*2,
                velocities=[0]*2,
                time_from_start=Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))))
        self.gripper_joint_command_publisher.publish(msg)

        #self.joint_positions += np.ones(9) * 0.002

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = SimController()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
