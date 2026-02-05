from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import ( LaunchConfiguration, PathJoinSubstitution, PythonExpression,)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # --------------------------------------------------
    # Launch arguments
    # --------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui          = LaunchConfiguration('gui', default='true')
    verbose      = LaunchConfiguration('verbose', default='false')
    rover        = LaunchConfiguration('rover', default='x40a')
    map_file     = LaunchConfiguration('map')
    wall         = LaunchConfiguration('wall')

    # --------------------------------------------------
    # Package shares
    # --------------------------------------------------
    gazebo_pkg = FindPackageShare('fwdsrover_xna_gazebo')
    nav_pkg    = FindPackageShare('fwdsrover_xna_navigation')

    # --------------------------------------------------
    # Default map (FULL PATH)
    # --------------------------------------------------
    default_map = PathJoinSubstitution([
        nav_pkg,
        'maps',
        'test.yaml'
    ])

    # --------------------------------------------------
    # Auto-select Nav2 params by rover type
    # --------------------------------------------------
    auto_params_file = PythonExpression([
        '"', nav_pkg, '/config/',
        '" + ("x40a_nav2_params.yaml" if "', rover,
        '" == "x40a" else "x120a_nav2_params.yaml")'
    ])

    # --------------------------------------------------
    # Gazebo bringup (robot + world)
    # --------------------------------------------------
    gazebo_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                gazebo_pkg,
                'launch',
                'gazebo_bringup.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': gui,
            'verbose': verbose,
            'rover': rover,
        }.items()
    )

    # --------------------------------------------------
    # Wall spawn (static object)
    # --------------------------------------------------
    spawn_wall = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                gazebo_pkg,
                'launch',
                'spawn_wall.launch.py'
            ])
        ),
        launch_arguments={
            'mesh_uri': wall,
        }.items()
    )

    # --------------------------------------------------
    # Navigation (Nav2)
    # params_file is explicitly passed
    # --------------------------------------------------
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                nav_pkg,
                'launch',
                'navigation.launch.py'
            ])
        ),
        launch_arguments={
            'map': map_file,
            'params_file': auto_params_file,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # --------------------------------------------------
    # Launch description
    # --------------------------------------------------
    return LaunchDescription([

        DeclareLaunchArgument(
            'rover',
            default_value='x40a',
            description='Rover type: x40a or x120a'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'gui',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'verbose',
            default_value='false'
        ),

        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to map yaml'
        ),

        DeclareLaunchArgument(
            'wall',
            default_value='',
            description='Wall STL filename (e.g. Wall.stl, Wall2.stl)'
        ),

        gazebo_bringup,
        spawn_wall,
        navigation,
    ])

