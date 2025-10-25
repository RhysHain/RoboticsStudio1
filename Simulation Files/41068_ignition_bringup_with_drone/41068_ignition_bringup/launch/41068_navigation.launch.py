from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    config_path = PathJoinSubstitution([FindPackageShare('41068_ignition_bringup'), 'config'])

    # Additional command line arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )

    # Map argument
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='/home/zubayr/41068_ws/my_map_good.yaml',
        description='Full path to map file'
    )

    # Use SLAM mode (True) or localization mode with saved map (False)
    use_slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether to run SLAM (True) or use saved map with AMCL (False)'
    )

    # Start Simultaneous Localisation and Mapping (SLAM) - only if slam:=True
    slam = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('slam_toolbox'),
                             'launch', 'online_async_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': PathJoinSubstitution([config_path, 'slam_params.yaml'])
        }.items(),
        condition=IfCondition(LaunchConfiguration('slam'))
    )

    # Start Localization with saved map - only if slam:=False
    localization = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'localization_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': LaunchConfiguration('map'),
            'params_file': PathJoinSubstitution([config_path, 'nav2_params.yaml'])
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('slam'))
    )

    # Start Navigation Stack (always runs)
    navigation = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution([config_path, 'nav2_params.yaml'])
        }.items()
    )

    ld.add_action(use_sim_time_launch_arg)
    ld.add_action(map_arg)
    ld.add_action(use_slam_arg)
    ld.add_action(slam)
    ld.add_action(localization)
    ld.add_action(navigation)

    return ld
