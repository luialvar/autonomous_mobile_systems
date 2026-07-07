from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    map_topic = LaunchConfiguration("map_topic")
    output_map_path = LaunchConfiguration("output_map_path")
    use_rviz = LaunchConfiguration("use_rviz")

    mapper = Node(
        package="map_build",
        executable="map_build_node",
        name="map_build",
        output="screen",
        parameters=[{
            "output_map_path": output_map_path,
        }],
        remappings=[
            ("scan", scan_topic),
            ("odom", odom_topic),
            ("map_from_scan", map_topic),
        ],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "scan_topic",
            default_value="/scan",
            description="Input LaserScan topic.",
        ),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/odom",
            description="Input odometry topic used as known pose.",
        ),
        DeclareLaunchArgument(
            "map_topic",
            default_value="/map_from_scan",
            description="Output OccupancyGrid topic.",
        ),
        DeclareLaunchArgument(
            "output_map_path",
            default_value="map.ppm",
            description="Path where the current PPM map is written.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2 together with the mapper.",
        ),
        mapper,
        rviz2,
    ])
