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


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="quadrupedal_sim",
            executable="dataset_gatherer.py",
            arguments=["-topic", "robot_description", "-entity", "my_robot"],
            output="screen",
            parameters=[{"use_sim_time": True}],
        )
    ])
