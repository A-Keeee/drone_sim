from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    share = get_package_share_directory('drone_visualization')
    params = os.path.join(share, 'config', 'sim.yaml')
    rviz = LaunchConfiguration('rviz')
    packages = [('drone_dynamics','quadrotor_dynamics_node'),('drone_dynamics','imu_gps_fusion_node'),
                ('drone_controller','position_controller_node'),('drone_map','voxel_map_node'),
                ('drone_map','simulated_lidar_node'),('drone_planner','ab_planner_node'),
                ('drone_visualization','visualization_node')]
    nodes = [Node(package=p, executable=n, output='screen', parameters=[params]) for p,n in packages]
    nodes.append(Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(share,'rviz','sim.rviz')], condition=__import__('launch.conditions').conditions.IfCondition(rviz)))
    return LaunchDescription([DeclareLaunchArgument('rviz', default_value='true')] + nodes)
