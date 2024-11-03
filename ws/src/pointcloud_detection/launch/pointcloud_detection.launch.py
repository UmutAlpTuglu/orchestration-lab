#!/usr/bin/env python

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    params_arg = DeclareLaunchArgument('params', default_value=PathJoinSubstitution([
        get_package_share_directory("pointcloud_detection"), "config", "params.yml"])
    )

    return LaunchDescription([
        params_arg,
        Node(package="pointcloud_detection",
             executable="pointcloud_detection",
             name="pointcloud_detection",
             output="screen",
             emulate_tty=True,
             parameters=[LaunchConfiguration('params')])
    ])
