import os
import sys

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    get_launch_description_from_python_launch_file,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _generate_launch_description(context: launch.LaunchContext, *args, **kwargs):
    use_sim_time = True
    tf_timeout = 0.0  #  tf buffer is always filled before processing any msg

    bag_filename = DeclareLaunchArgument(name="bag_filename", description="")
    file = LaunchConfiguration("bag_filename")
    output_dir_default = os.path.dirname(file.perform(context=context))
    output_dir = DeclareLaunchArgument(
        name="output_dir", default_value=output_dir_default, description=""
    )
    common_launch_args = get_launch_description_from_python_launch_file(
        get_package_share_directory("kinematic_icp") + "/launch/common_args.launch.py",
    )

    kinematic_icp_offline_node = Node(
        package="kinematic_icp",
        executable="kinematic_icp_offline_node",
        name="offline_node",
        namespace="kinematic_icp",
        output="screen",
        emulate_tty=True,
        remappings=[
            ("lidar_odometry", LaunchConfiguration("lidar_odometry_topic")),
        ],
        parameters=[
            # KISS-ICP configuration
            get_package_share_directory("kinematic_icp")
            + "/config/kinematic_icp_ros.yaml",
            {
                # Input topic, is not a remap to marry API with offline node
                "input": LaunchConfiguration("lidar_topic"),
                # ROS node configuration
                "lidar_odom_frame": LaunchConfiguration("lidar_odom_frame"),
                "wheel_odom_frame": LaunchConfiguration("wheel_odom_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "publish_odom_tf": LaunchConfiguration("publish_odom_tf"),
                "invert_odom_tf": LaunchConfiguration("invert_odom_tf"),
                "tf_timeout": tf_timeout,
                # ROS CLI arguments
                "use_sim_time": use_sim_time,
                # Offline node specific configuration
                "bag_filename": file,
                "output_dir": LaunchConfiguration("output_dir"),
                "max_num_threads": 0,
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
            "-d",
            get_package_share_directory("kinematic_icp") + "/rviz/kinematic_icp.rviz",
        ],
        condition=IfCondition(LaunchConfiguration("visualize")),
    )
    return [
        bag_filename,
        output_dir,
        common_launch_args,
        kinematic_icp_offline_node,
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_generate_launch_description)])
