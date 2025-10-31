from ament_index_python.packages import get_package_share_path
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_package_path = get_package_share_path('fwdsrover_description')
    bringup_package_path = get_package_share_path('fwdsrover_xna_bringup')
    default_model_path = description_package_path / 'urdf/x40a.xacro'
    default_rviz_config_path = bringup_package_path / 'rviz/fwdsrover_xna.rviz'
    
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time', default_value='false',
        description='Use simulation time if available'
    )
    
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='normal',
        description="Operation mode for rover_sim_node: 'normal' or 'odo_driven'"
    )
    
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig', default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file'
    )

    model_arg = DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),
        description='Absolute path to robot urdf file'
    )

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    rover_sim_node = Node(
        package='fwdsrover_xna_bringup',
        executable='rover_sim_node',
        name='rover_sim_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'cmd_topic': '/rover_twist',
            'joint_states_topic': '/joint_states', 
            'steer_cmd_topic': '/steer_position_controller/commands',
            'wheel_cmd_topic': '/wheel_velocity_controller/commands',
            
            #
            'mode': LaunchConfiguration('mode'),
            'operation_mode': LaunchConfiguration('mode'),
            
            }],
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description,}],
    )

    pub_odom_node = Node(
        package='fwdsrover_xna_bringup',
        executable='pub_odom',
        name='pub_odom',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )
    
    odom_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_static',
        arguments=['0','0','0','0','0','0','odom','base_footprint'],
        condition=None,  
    )

    return LaunchDescription([
        use_sim_time_arg,
        mode_arg,
        model_arg,
        rviz_arg,
        robot_state_publisher_node,
        rover_sim_node,
        pub_odom_node,
        odom_static_tf,
        rviz_node,
    ])
