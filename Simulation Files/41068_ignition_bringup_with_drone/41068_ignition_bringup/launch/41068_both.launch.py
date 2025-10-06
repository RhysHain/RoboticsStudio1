from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ld = LaunchDescription()

    # Get paths to directories
    pkg_path = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_path, 'config'])

    # --- Args ---
    use_sim_time_launch_arg = DeclareLaunchArgument('use_sim_time', default_value='True')
    ld.add_action(use_sim_time_launch_arg)
    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_launch_arg = DeclareLaunchArgument('rviz', default_value='False')
    ld.add_action(rviz_launch_arg)

    nav2_launch_arg = DeclareLaunchArgument('nav2', default_value='True')
    ld.add_action(nav2_launch_arg)

    world_launch_arg = DeclareLaunchArgument(
        'world', default_value='simple_trees', choices=['simple_trees', 'large_demo'])
    ld.add_action(world_launch_arg)

    # --- Gazebo world (unchanged) ---
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'),
                             'launch', 'ign_gazebo.launch.py']),
        launch_arguments={
            'ign_args': [PathJoinSubstitution([pkg_path, 'worlds',
                                               [LaunchConfiguration('world'), '.sdf']]),
                         ' -r']}.items()
    )
    ld.add_action(gazebo)

    
    # --- Global Clock Bridge ---
    clock_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge_clock.yaml']),
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    ld.add_action(clock_bridge)

    # =========================
    #        DRONE (/parrot)
    # =========================
    parrot_robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg_path, 'urdf_drone', 'parrot.urdf.xacro'])]),
        value_type=str
    )

    parrot_group = GroupAction([
        PushRosNamespace('parrot'),

        # robot_state_publisher with TF prefix so frames don’t collide
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': parrot_robot_description,
                'use_sim_time': use_sim_time,
                'frame_prefix': 'parrot/'     # <-- key for namespaced TF
            }],
            output='screen'
        ),

        # EKF with namespaced frames
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='robot_localization',
            output='screen',
            parameters=[
                PathJoinSubstitution([config_path, 'robot_localization.yaml']),
                {
                    'use_sim_time': use_sim_time,
                    'map_frame': 'map',
                    'odom_frame': 'parrot/odom',
                    'base_link_frame': 'parrot/base_link',
                    'world_frame': 'parrot/odom'
                }
            ]
        ),

        # DRONE bridge (run under /parrot so ROS topics become /parrot/*)
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            name='gazebo_bridge_drone',
            parameters=[{
                'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge_drone.yaml']),
                'use_sim_time': use_sim_time
            }],
            output='screen'
        ),

        # Spawn the drone model; publish its URDF from /parrot namespace
        Node(
            package='ros_ign_gazebo',
            executable='create',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-name', 'parrot',                 # <-- give the model a unique name
                '-topic', '/parrot/robot_description',
                '-z', '2.0'
            ]
        ),
    ])
    ld.add_action(parrot_group)

    # ---- RGBD color -> global coordinate node (subscribe to /parrot camera topics) ----
    camera_tree_node = Node(
        package='41068_ignition_bringup',
        executable='camera_tree_to_map',
        name='camera_tree_to_map',
        output='screen',
        parameters=[{
            'rgb_topic':   '/parrot/camera/color/image_rect_color',
            'depth_topic': '/parrot/camera/aligned_depth_to_color/image_raw',
            'info_topic':  '/parrot/camera/color/camera_info',
            'target_frame': 'map',
            'roi_px': 7,
            'min_m': 0.2,
            'max_m': 20.0,
        }]
    )
    ld.add_action(camera_tree_node)
    # -----------------------------------------------

    # =========================
    #      HUSKY ADD-ON (/husky)
    # =========================
    husky_description_content = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg_path, 'urdf', 'husky.urdf.xacro'])]),
        value_type=str
    )

    husky_group = GroupAction([
        PushRosNamespace('husky'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': husky_description_content,
                'use_sim_time': use_sim_time,
                'frame_prefix': 'husky/'
            }],
            output='screen'
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='robot_localization',
            output='screen',
            parameters=[
                PathJoinSubstitution([config_path, 'robot_localization.yaml']),
                {
                    'use_sim_time': use_sim_time,
                    'map_frame': 'map',
                    'odom_frame': 'husky/odom',
                    'base_link_frame': 'husky/base_link',
                    'world_frame': 'husky/odom'
                }
            ]
        ),

        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            name='gazebo_bridge_husky',
            parameters=[{
                'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge_husky.yaml']),
                'use_sim_time': use_sim_time
            }],
            output='screen'
        ),

        Node(
            package='ros_ign_gazebo',
            executable='create',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-name', 'husky',
                '-topic', '/husky/robot_description',
                '-x', '0.0', '-y', '0.0', '-z', '0.4'
            ]
        ),
    ])
    ld.add_action(husky_group)
    # =========================

    # rviz (unchanged)
    rviz_node = Node(
        package='rviz2', executable='rviz2', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path, '41068.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    # Nav2 (unchanged)
    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_path, 'launch', '41068_navigation.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(nav2)

    return ld
