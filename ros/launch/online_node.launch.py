import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import (
    get_launch_description_from_python_launch_file,
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions.find_package import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default=(os.getenv("SIMULATION") == "true"))
    tf_timeout = LaunchConfiguration(
        "tf_timeout",
        default=PythonExpression(["'0.1' if ", use_sim_time, " else '0.0'"]),
    )
    common_launch_args = get_launch_description_from_python_launch_file(
        get_package_share_directory("kinematic_icp") + "/launch/common_args.launch.py",
    )

    kinematic_icp_online_node = Node(
        package="kinematic_icp",
        executable="kinematic_icp_online_node",
        name="online_node",
        namespace="kinematic_icp",
        output="screen",
        remappings=[
            ("lidar_odometry", LaunchConfiguration("lidar_odometry_topic")),
        ],
        parameters=[
            # KISS-ICP configuration
            get_package_share_directory("kinematic_icp") + "/config/kinematic_icp_ros.yaml",
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
            },
        ],
    )

    return LaunchDescription(
        [
            common_launch_args,
            kinematic_icp_online_node,
        ]
    )
