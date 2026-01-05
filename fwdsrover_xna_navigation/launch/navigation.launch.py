
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, LaunchConfigurationEquals


def generate_launch_description():
    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('fwdsrover_xna_navigation'), 'launch')
        
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('fwdsrover_xna_navigation'),
            'maps',
            'test.yaml'))  # change this to your own map for navigation

    x40a_param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('fwdsrover_xna_navigation'),
            'config',
            'x40a_nav2_params.yaml'))

    x120a_param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('fwdsrover_xna_navigation'),
            'config',
            'x120a_nav2_params.yaml'))
            
    rviz_config_dir = os.path.join(
        get_package_share_directory('fwdsrover_xna_navigation'),
        'rviz',
        'nav2.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rover',
            default_value='x40a',
            description='fwdsrover model',
            choices=['x40a', 'x120a']),
            
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),
            
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
            
        DeclareLaunchArgument(
            'params_file',
            condition=LaunchConfigurationEquals('rover', 'x40a'),
            default_value=x40a_param_dir,
            description='Full path to param file to load'),
            
        DeclareLaunchArgument(
            'params_file',
            condition=LaunchConfigurationEquals('rover', 'x120a'),
            default_value=x120a_param_dir,
            description='Full path to param file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch_file_dir, '/bringup_launch.py']),
            condition=LaunchConfigurationEquals('rover', 'x40a'),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': x40a_param_dir}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch_file_dir, '/bringup_launch.py']),
            condition=LaunchConfigurationEquals('rover', 'x120a'),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': x120a_param_dir}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
