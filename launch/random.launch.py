from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('drone_visualization')
    base = os.path.join(share, 'config', 'sim.yaml')
    random_map = os.path.join(share, 'config', 'random.yaml')
    rviz = LaunchConfiguration('rviz')
    seed = LaunchConfiguration('seed')

    packages = [
        ('drone_dynamics', 'quadrotor_dynamics_node'),
        ('drone_dynamics', 'imu_gps_fusion_node'),
        ('drone_controller', 'position_controller_node'),
        ('drone_map', 'simulated_lidar_node'),
        ('drone_planner', 'ab_planner_node'),
        ('drone_visualization', 'visualization_node'),
    ]
    nodes = [
        Node(package=package, executable=executable, output='screen',
             parameters=[base, random_map])
        for package, executable in packages
    ]
    nodes.append(
        Node(package='drone_map', executable='voxel_map_node', output='screen',
             parameters=[base, random_map, {
                 'random_seed': ParameterValue(seed, value_type=int),
             }])
    )
    nodes.append(
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share, 'rviz', 'sim.rviz')],
             condition=IfCondition(rviz))
    )
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('seed', default_value='20260715',
                              description='Deterministic random-map seed'),
        *nodes,
    ])
