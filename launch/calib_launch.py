from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )

    # Get package directory
    pkg_share = get_package_share_directory('fast_calib')
    
    # Parameters file path
    params_file = os.path.join(pkg_share, 'config', 'qr_params.yaml')
    
    # RViz config file path
    rviz_config = os.path.join(pkg_share, 'rviz_cfg', 'fast_livo2.rviz')

    # Fast calib node
    fast_calib_node = Node(
        package='fast_calib',
        executable='fast_calib',
        name='fast_calib',
        parameters=[params_file],
        output='screen'
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        fast_calib_node,
        rviz_node
    ])