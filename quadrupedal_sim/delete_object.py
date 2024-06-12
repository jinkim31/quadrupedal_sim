import os
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import DeleteEntity
from ament_index_python.packages import get_package_share_directory
import quadrupedal_sim.misc as misc
import time

# sdf_path = os.path.join(get_package_share_directory('manip_challenge'), 'models')



def spawn_sdf_object(node, client, object_name, name=None):
    """ Spawn an object """
    
    # Set data for request
    sdf_file_path = os.path.join(
    get_package_share_directory("quadrupedal_sim"), "data", "models",
    object_name, "model.sdf")
    request = DeleteEntity.Request()
    request.name = name or object_name
    node.get_logger().info("Sending service request to `/delete_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

def main():
    # Start node
    rclpy.init()
    
    node = rclpy.create_node("entity_deleter")
    time.sleep(5)

    node.get_logger().info(
        'Creating Service client to connect to `/delete_entity`')
    client = node.create_client(DeleteEntity, "/delete_entity")

    node.get_logger().info("Connecting to `/delete_entity` service...")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...connected!")
    
    # Spawn object
    spawn_sdf_object(node, client, 'cafe_table')
    spawn_sdf_object(node, client, 'book')
    spawn_sdf_object(node, client, 'eraser')
    spawn_sdf_object(node, client, 'snacks')
    spawn_sdf_object(node, client, 'soap2')
    spawn_sdf_object(node, client, 'biscuits')
    spawn_sdf_object(node, client, 'glue')
    spawn_sdf_object(node, client, 'soap')

    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
