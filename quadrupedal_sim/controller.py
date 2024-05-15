import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np

class SimController(Node):
    def __init__(self):
        super().__init__('quadrupedal_sim_controller')
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            'joint_trajectory_controller/joint_trajectory', 10)
        timer_period = 5.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    def timer_callback(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names.append('world_to_follower_x')
        msg.joint_names.append('follower_x_to_follower_y')
        msg.joint_names.append('follower_y_to_follower_z')

        for i in range(3000):
            t = i/1000.0
            msg.points.append(
                JointTrajectoryPoint(
                    positions=[np.sin(1*np.pi*t/3.0)]*3,
                    velocities=[0]*3,
                    time_from_start=Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))))

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing')

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = SimController()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
