from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # 声明 3 个路径参数
    node_params_arg = DeclareLaunchArgument(
        "node_params",
        default_value="/home/agv/wzb/src/icp_localization_ros2/config/node_params.yaml",
        description="Path to node_params.yaml"
    )

    icp_config_arg = DeclareLaunchArgument(
        "icp_config",
        default_value="/home/agv/wzb/src/icp_localization_ros2/config/icp.yaml",
        description="Path to icp.yaml"
    )

    input_filters_arg = DeclareLaunchArgument(
        "input_filters",
        default_value="/home/agv/wzb/src/icp_localization_ros2/config/input_filters_mid360.yaml",
        description="Path to input_filters.yaml"
    )

    node_params = LaunchConfiguration("node_params")
    icp_config = LaunchConfiguration("icp_config")
    input_filters = LaunchConfiguration("input_filters")

    icp_node = Node(
        package="icp_localization_ros2",
        executable="icp_localization",
        output="screen",
    parameters=[
        node_params,
        {"icp_config_path": icp_config},
        {"input_filters_config_path": input_filters},
    ]

    )

    return LaunchDescription([
        node_params_arg,
        icp_config_arg,
        input_filters_arg,
        icp_node,
    ])
