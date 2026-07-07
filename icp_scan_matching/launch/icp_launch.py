from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")

    icp = Node(
        package="icp_scan_matching",
        executable="icp_node",
        name="icp_scan_matching",
        output="screen",
        parameters=[{
            "max_iterations": ParameterValue(
                LaunchConfiguration("max_iterations"),
                value_type=int,
            ),
            "tolerance": ParameterValue(
                LaunchConfiguration("tolerance"),
                value_type=float,
            ),
            "max_correspondence_distance": ParameterValue(
                LaunchConfiguration("max_correspondence_distance"),
                value_type=float,
            ),
            "min_correspondences": ParameterValue(
                LaunchConfiguration("min_correspondences"),
                value_type=int,
            ),
            "match_every_n_scans": ParameterValue(
                LaunchConfiguration("match_every_n_scans"),
                value_type=int,
            ),
            "publish_every_n_scans": ParameterValue(
                LaunchConfiguration("publish_every_n_scans"),
                value_type=int,
            ),
            "beam_stride": ParameterValue(
                LaunchConfiguration("beam_stride"),
                value_type=int,
            ),
            "voxel_size": ParameterValue(
                LaunchConfiguration("voxel_size"),
                value_type=float,
            ),
            "max_meta_points": ParameterValue(
                LaunchConfiguration("max_meta_points"),
                value_type=int,
            ),
            "frame_id": LaunchConfiguration("frame_id"),
        }],
        remappings=[
            ("scan", LaunchConfiguration("scan_topic")),
            ("odom", LaunchConfiguration("odom_topic")),
            ("meta_scan", LaunchConfiguration("meta_scan_topic")),
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
            description="Input odometry topic used as the initial scan pose guess.",
        ),
        DeclareLaunchArgument(
            "meta_scan_topic",
            default_value="/meta_scan",
            description="Output accumulated PointCloud2 topic.",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="map",
            description="Frame id for the accumulated meta scan.",
        ),
        DeclareLaunchArgument(
            "max_iterations",
            default_value="60",
            description="Maximum ICP iterations per scan pair.",
        ),
        DeclareLaunchArgument(
            "tolerance",
            default_value="1e-5",
            description="RMSE change threshold for ICP convergence.",
        ),
        DeclareLaunchArgument(
            "max_correspondence_distance",
            default_value="0.5",
            description="Maximum accepted point-to-point match distance.",
        ),
        DeclareLaunchArgument(
            "min_correspondences",
            default_value="12",
            description="Minimum valid point pairs required to run ICP.",
        ),
        DeclareLaunchArgument(
            "match_every_n_scans",
            default_value="1",
            description="Only align every n-th incoming scan.",
        ),
        DeclareLaunchArgument(
            "publish_every_n_scans",
            default_value="5",
            description="Publish the accumulated meta scan every n accepted scans.",
        ),
        DeclareLaunchArgument(
            "beam_stride",
            default_value="2",
            description="Use every n-th laser beam before ICP.",
        ),
        DeclareLaunchArgument(
            "voxel_size",
            default_value="0.03",
            description="Voxel size for accumulated cloud downsampling.",
        ),
        DeclareLaunchArgument(
            "max_meta_points",
            default_value="200000",
            description="Maximum number of accumulated meta-scan points.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2 together with the ICP node.",
        ),
        icp,
        rviz2,
    ])
