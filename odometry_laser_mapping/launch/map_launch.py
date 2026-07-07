from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    map_yaml = LaunchConfiguration("map_yaml")
    use_rviz = LaunchConfiguration("use_rviz")

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_yaml}],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{
            "autostart": True,
            "node_names": ["map_server"],
        }],
    )

    simulator = Node(
        package="map_sim",
        executable="map_sim_node",
        name="map_sim",
        output="screen",
        parameters=[{
            "laser_pose_x_cells": ParameterValue(
                LaunchConfiguration("laser_pose_x_cells"),
                value_type=float,
            ),
            "laser_pose_y_cells": ParameterValue(
                LaunchConfiguration("laser_pose_y_cells"),
                value_type=float,
            ),
            "laser_pose_theta": ParameterValue(
                LaunchConfiguration("laser_pose_theta"),
                value_type=float,
            ),
            "angle_increment_deg": ParameterValue(
                LaunchConfiguration("angle_increment_deg"),
                value_type=float,
            ),
            "range_max_m": ParameterValue(
                LaunchConfiguration("range_max_m"),
                value_type=float,
            ),
        }],
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
            "map_yaml",
            description="Path to the occupancy-grid YAML map file.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2 together with the simulator.",
        ),
        DeclareLaunchArgument(
            "laser_pose_x_cells",
            default_value="992.0",
            description="Laser x position in occupancy-grid cell coordinates.",
        ),
        DeclareLaunchArgument(
            "laser_pose_y_cells",
            default_value="992.0",
            description="Laser y position in occupancy-grid cell coordinates.",
        ),
        DeclareLaunchArgument(
            "laser_pose_theta",
            default_value="0.0",
            description="Laser yaw angle in radians.",
        ),
        DeclareLaunchArgument(
            "angle_increment_deg",
            default_value="0.05",
            description="Angular spacing between simulated laser beams.",
        ),
        DeclareLaunchArgument(
            "range_max_m",
            default_value="100.0",
            description="Maximum simulated laser range in meters.",
        ),
        map_server,
        lifecycle_manager,
        simulator,
        rviz2,
    ])
