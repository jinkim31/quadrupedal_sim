#!/usr/bin/python3


# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable,
                            LogInfo, RegisterEventHandler, TimerAction,
                            ExecuteProcess)
from launch.substitutions import TextSubstitution, LaunchConfiguration

import xacro
import yaml


# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():

    # add .sdf model path
    model_path = os.path.join(get_package_share_directory('quadrupedal_sim'))
    if 'GAZEBO_MODEL_PATH' in os.environ:
        print("GAZEBO_MODEL_PATH already exists")
    else:
        print("GAZEBO_MODEL_PATH does not exist")
        os.environ['GAZEBO_MODEL_PATH'] = model_path
        print("GAZEBO_MODEL_PATH set to", os.environ['GAZEBO_MODEL_PATH'])

    # launch args
    launch_args = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="use sim time"
    )

    # robot state publisher
    pkg_path = os.path.join(get_package_share_directory("quadrupedal_sim"))
    xacro_file = os.path.join(pkg_path, "urdf", "06-flexible.urdf")
    robot_description = xacro.process_file(xacro_file)
    print(robot_description.toxml())
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description.toxml(),
            "use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']))

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "my_robot"],
        output="screen",
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_trajectory_controller'],
        output='screen'
    )

    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory_controller],
            )
        ),

        launch_args,
        state_publisher,
        gazebo,
        spawn_entity
    ])