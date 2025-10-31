#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time        = LaunchConfiguration('use_sim_time', default='true')
    cmd_topic           = LaunchConfiguration('cmd_topic', default='/rover_twist')
    odo_topic           = LaunchConfiguration('odo_topic', default='/rover_odo')
    sensor_topic        = LaunchConfiguration('sensor_topic', default='/rover_sensor')
    joint_states_topic  = LaunchConfiguration('joint_states_topic', default='/joint_states')
    steer_cmd_topic     = LaunchConfiguration('steer_cmd_topic', default='/steer_position_controller/commands')
    wheel_cmd_topic     = LaunchConfiguration('wheel_cmd_topic', default='/wheel_velocity_controller/commands')


    mode_arg            = LaunchConfiguration('mode', default='normal')  # normal / odo_driven（odoエイリアス可）

    rover_sim = Node(
        package='fwdsrover_xna_bringup', 
        executable='rover_sim_node',
        name='rover_sim_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,

            'cmd_topic': cmd_topic,
            'odo_topic': odo_topic,
            'sensor_topic': sensor_topic,
            'joint_states_topic': joint_states_topic,
            'steer_cmd_topic': steer_cmd_topic,
            'wheel_cmd_topic': wheel_cmd_topic,
            'operation_mode': mode_arg,
            'mode': mode_arg,
        }],
    )

    return LaunchDescription([

        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use /clock from simulator'),
        DeclareLaunchArgument('mode', default_value='normal',
                              description="Operation mode for rover_sim_node: 'normal' or 'odo_driven'"),
        DeclareLaunchArgument('cmd_topic', default_value='/rover_twist',
                              description='Twist command input topic'),
        DeclareLaunchArgument('odo_topic', default_value='/rover_odo',
                              description='Twist-like odometry input when mode=odo_driven'),
        DeclareLaunchArgument('sensor_topic', default_value='/rover_sensor',
                              description='Int16MultiArray sensor topic (published in normal mode)'),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states',
                              description='sensor_msgs/JointState publish topic'),
        DeclareLaunchArgument('steer_cmd_topic', default_value='/steer_position_controller/commands',
                              description='std_msgs/Float64MultiArray for steering controller'),
        DeclareLaunchArgument('wheel_cmd_topic', default_value='/wheel_velocity_controller/commands',
                              description='std_msgs/Float64MultiArray for wheel velocity controller'),

        rover_sim,
    ])

