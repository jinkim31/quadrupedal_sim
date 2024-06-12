import os
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory
import quadrupedal_sim.misc as misc
import time

# sdf_path = os.path.join(get_package_share_directory('manip_challenge'), 'models')



def spawn_sdf_object(node, client, object_name, xyzrpy, name=None):
    """ Spawn an object """
    
    # Set data for request
    sdf_file_path = os.path.join(
    get_package_share_directory("quadrupedal_sim"), "data", "models",
    object_name, "model.sdf")
    request = SpawnEntity.Request()
    request.name = name or object_name
    request.xml = open(sdf_file_path, 'r').read()
    request.robot_namespace = object_name
    request.initial_pose = misc.list2Pose(xyzrpy)

    node.get_logger().info("Sending service request to `/spawn_entity`")
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
    
    node = rclpy.create_node("entity_spawner")
    time.sleep(5)

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")

    node.get_logger().info("Connecting to `/spawn_entity` service...")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...connected!")
    
    object_pos_dir = "/home/minseok/cs477_final_ws/src/quadrupedal_sim/quadrupedal_sim/save_rand_object_setting"
    data = np.load(os.path.join(object_pos_dir, "20240612_005549.npy"), allow_pickle=True)

    table_x = 1.0
    table_y = 0.0

    # Extract the variables
    table_random_height = data['table_random_height'][0]
    object_random_place = data['object_random_place'][0]
    object_random_yaw = data['object_random_yaw'][0]
    object_list_np = data['object_list'][0]

    # Convert the NumPy array of strings back to a Python list
    object_list = list(object_list_np)

    print("table_random_height:", table_random_height)
    print("object_random_place:", object_random_place)
    print("object_random_yaw:", object_random_yaw)
    print("object_list:", object_list)

    spawn_sdf_object(node, client, 'cafe_table', [table_x, table_y, table_random_height, 0.0, 0.0, 0.0])

    i = 0
    for object in object_list:
        spawn_sdf_object(node, client, object, [object_random_place[i,0], object_random_place[i,1], table_random_height + 0.8, 0.0, 0.0, object_random_yaw[i]])
        i += 1

    # Spawn object
    # spawn_sdf_object(node, client, 'cafe_table', [1.2, 0.0, -0.2, 0.0, 0.0, 0.0])
    # spawn_sdf_object(node, client, 'book', [0.52,0.2,0.6, 0, 0, np.pi/4.])
    # spawn_sdf_object(node, client, 'eraser', [0.52,-0.2,0.6, 0, 0, np.pi/2.])
    # spawn_sdf_object(node, client, 'snacks', [0.72,0.3,0.6, 0, 0, np.pi/4.])
    # spawn_sdf_object(node, client, 'soap2', [0.72,-0.1,0.6, 0, 0, np.pi/4.])
    # spawn_sdf_object(node, client, 'biscuits', [0.72,-0.3,0.6, 0, 0, np.pi/4.])
    # spawn_sdf_object(node, client, 'glue', [0.52, 0.,0.6, 0, 0, np.pi/4.])
    # spawn_sdf_object(node, client, 'soap', [0.72,0.1,0.6, 0, 0, -np.pi/4.])

    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
