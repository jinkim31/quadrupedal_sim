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
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

import xacro
import yaml


# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():    
    # add .sdf model path
    model_path = os.path.join(get_package_share_directory('quadrupedal_sim'),'data','models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        print("GAZEBO_MODEL_PATH already exists")
    else:
        print("GAZEBO_MODEL_PATH does not exist")
        os.environ['GAZEBO_MODEL_PATH'] = model_path
        print("GAZEBO_MODEL_PATH set to", os.environ['GAZEBO_MODEL_PATH'])

    # pos_x_parameter_name = 'pos_x'
    # pos_y_parameter_name = 'pos_y'
    # pos_z_parameter_name = 'pos_z'

    # pos_x = LaunchConfiguration(pos_x_parameter_name)
    # pos_y = LaunchConfiguration(pos_y_parameter_name)
    # pos_z = LaunchConfiguration(pos_z_parameter_name)

    # world_xacro_file = os.path.join(get_package_share_directory('quadrupedal_sim'), 'data',
    #                                  'worlds', 'ur5_picking_challenge.world.xacro')
    world_file = os.path.join(get_package_share_directory('quadrupedal_sim'), 'data',
                                     'worlds', 'demo_world.world')
    # convert_xacro = ExecuteProcess(
    #     cmd=[
    #         'xacro', world_xacro_file,
    #         'pos_x:=', pos_x,
    #         'pos_y:=', pos_y,
    #         'pos_z:=', pos_z,
    #         '-o', world_file
    #     ],
    #     output='screen'
    # )
    # convert_xacro = Command(
    #     [FindExecutable(name='xacro'), ' ', world_xacro_file, ' pos_x:=', pos_x, ' pos_y:=', pos_y, ' pos_z:=', pos_z,
    #      ' -o ', world_file]
    #     )
    # convert_xacro = ExecuteProcess(
    #     cmd=[FindExecutable(name='xacro'), world_xacro_file, 'pos_x:=' + pos_x, 'pos_y:=' + pos_y, 'pos_z:=' + pos_z,
    #          '-o', world_file],
    #     output='screen'
    # )

    pkg_path = os.path.join(get_package_share_directory("quadrupedal_sim"))

    # robot state publisher
    robot_description = xacro.process_file(os.path.join(pkg_path, "description", "quadrupedal.urdf.xacro"))
    # print(robot_description.toxml())
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description.toxml(),
            "use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # gazebo
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    #         # launch_arguments={'world': os.path.join(pkg_path, "world", "world.world")}.items())
    #         launch_arguments={'world': os.path.join(pkg_path, "data", "worlds", "ur5_picking_challenge.world")}.items())
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
            # launch_arguments={'world': os.path.join(pkg_path, "world", "world.world")}.items())
            launch_arguments={'world': world_file,
                              "verbose": "true",}.items()
        )


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

    load_base_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'base_joint_position_controller'],
        output='screen'
    )

    load_panda_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'panda_joint_trajectory_controller'],
        output='screen'
    )

    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'gripper_controller'],
        output='screen'
    )

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
                on_exit=[load_base_joint_trajectory_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_panda_joint_trajectory_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_gripper_controller],
            )
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="use sim time"),
        state_publisher,
        gazebo,
        spawn_entity,
    ])