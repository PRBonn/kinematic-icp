from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lidar_topic",
                description="",
            ),
            DeclareLaunchArgument(
                "lidar_odometry_topic",
                default_value="lidar_odometry",
                description="",
            ),
            DeclareLaunchArgument(
                "lidar_odom_frame",
                default_value="odom_lidar",
                description="",
            ),
            DeclareLaunchArgument(
                "wheel_odom_frame",
                default_value="odom",
                description="",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="base_footprint",
                description="",
            ),
            DeclareLaunchArgument(
                "publish_odom_tf",
                default_value="true",
                description="",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "invert_odom_tf",
                default_value="true",
                description="",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "visualize",
                default_value="true",
                description="",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "bag_filenames",
                default_value="",
                description="Comma-separated list of file paths",
            ),
        ]
    )
