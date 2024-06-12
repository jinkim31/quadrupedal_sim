import os
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory
import quadrupedal_sim.misc as misc
import time
from datetime import datetime

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

    table_x = 1.0
    table_y = 0.0
    table_random_height =  0.5*np.random.random_sample() - 0.3 # Random height between -0.3 and 0.2
    # Random Object place on the table


    # object_list = ["book", "soap2", "glue"]
    object_list = ["glue"]
    object_num = len(object_list)

    object_random_place = np.random.random_sample((object_num, 2))
    # table size : <size>0.913 0.913 0.04</size>
    # because of workspace of the robot, limit x area to -0.20 ~ 0.0
    object_random_place[:, 0] = 0.20 * object_random_place[:, 0] - 0.20
    object_random_place[:, 0] += table_x
    object_random_place[:, 1] = 0.5 * object_random_place[:, 1] - 0.25
    object_random_place[:, 1] += table_y
    object_random_yaw = 2*np.pi * np.random.random_sample(object_num) - np.pi

    spawn_sdf_object(node, client, 'cafe_table', [table_x, table_y, table_random_height, 0.0, 0.0, 0.0])

    i = 0
    for object in object_list:
        spawn_sdf_object(node, client, object, [object_random_place[i,0], object_random_place[i,1], table_random_height + 0.8, 0, 0, object_random_yaw[i]])
        i += 1

    # Spawn object
    # spawn_sdf_object(node, client, 'cafe_table', [table_x, table_y, table_random_height, 0.0, 0.0, 0.0])
    # spawn_sdf_object(node, client, 'book', [object_random_place[0,0], object_random_place[0,1], table_random_height + 0.8, 0, 0, object_random_yaw[0]])
    # spawn_sdf_object(node, client, 'eraser', [object_random_place[1,0], object_random_place[1,1], table_random_height + 0.8, 0, 0, object_random_yaw[1]])
    # spawn_sdf_object(node, client, 'snacks', [object_random_place[2,0], object_random_place[2,1], table_random_height + 0.8, 0, 0, object_random_yaw[2]])
    # spawn_sdf_object(node, client, 'soap2', [object_random_place[3,0], object_random_place[3,1], table_random_height + 0.8, 0, 0, object_random_yaw[3]])
    # spawn_sdf_object(node, client, 'biscuits', [object_random_place[4,0], object_random_place[4,1], table_random_height + 0.8, 0, 0, object_random_yaw[4]])
    # spawn_sdf_object(node, client, 'glue', [object_random_place[5,0], object_random_place[5,1], table_random_height + 0.8, 0, 0, object_random_yaw[5]])
    # spawn_sdf_object(node, client, 'soap', [object_random_place[6,0], object_random_place[6,1], table_random_height + 0.8, 0, 0, object_random_yaw[6]])
    
    object_list_np = np.array(object_list, dtype='U10')  # 'U10' means up to 10-character strings
    data = np.array([(table_random_height, object_random_place, object_random_yaw, object_list_np)],
                    dtype=[('table_random_height', 'f8'),
                        ('object_random_place', 'f8', (object_num, 2)),
                        ('object_random_yaw', 'f8', (object_num,)),
                        ('object_list', 'U10', (object_num,))])
    
    current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
    np.save(f"/home/minseok/cs477_final_ws/src/quadrupedal_sim/quadrupedal_sim/save_rand_object_setting/{current_time}.npy", data)
    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
