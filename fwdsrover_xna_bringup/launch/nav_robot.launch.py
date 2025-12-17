from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # --------------------------------------------------
    # Package paths (convert Path -> str)
    # --------------------------------------------------
    desc_pkg = str(get_package_share_path('fwdsrover_description'))
    bringup_pkg = str(get_package_share_path('fwdsrover_xna_bringup'))

    # --------------------------------------------------
    # Launch arguments
    # --------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rover = LaunchConfiguration('rover', default='x40a')

    # --------------------------------------------------
    # URDF (xacro) selection by rover type
    # --------------------------------------------------
    model_path = PythonExpression([
        '"', desc_pkg, '/urdf/',
        '" + ("x40a.xacro" if "', rover, '" == "x40a" else "x120a.xacro")'
    ])

    # --------------------------------------------------
    # RViz config
    # --------------------------------------------------
    default_rviz = bringup_pkg + '/rviz/laser.rviz'
    rvizconfig = LaunchConfiguration('rvizconfig', default=default_rviz)

    # --------------------------------------------------
    # Robot description
    # --------------------------------------------------
    robot_description = ParameterValue(
        Command(['xacro ', model_path]),
        value_type=str
    )

    # --------------------------------------------------
    # Nodes
    # --------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
    )

    pub_odom = Node(
        package='fwdsrover_xna_bringup',
        executable='pub_odom',
        name='pub_odom',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizconfig],
    )

    # --------------------------------------------------
    # Launch description
    # --------------------------------------------------
    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            'rover',
            default_value='x40a',
            description='Rover type: x40a or x120a'
        ),

        DeclareLaunchArgument(
            'rvizconfig',
            default_value=default_rviz,
            description='Absolute path to rviz config file'
        ),

        robot_state_publisher,
        joint_state_publisher,
        pub_odom,
        rviz,
    ])

