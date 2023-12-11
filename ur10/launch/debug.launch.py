# #!/usr/bin/python3
# # -*- coding: utf-8 -*-
# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription,ExecuteProcess,RegisterEventHandler
# from launch.event_handlers import OnProcessExit
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.substitutions import FindPackageShare
# from launch.substitutions import  PathJoinSubstitution
# from launch_ros.actions import Node


# def generate_launch_description():

#     # Get Gazebo ROS interface package
#     pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

#     # Get the location for empty world
#     world = os.path.join(
#         get_package_share_directory('bot_car'),
#         'worlds',
#         'empty_world.world'
#     )

#     # Launch Description to run Gazebo Server
#     gzserver_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
#         ),
#         launch_arguments={'world': world}.items()
#     )

#     # Launch Description to run Gazebo Client
#     gzclient_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
#         )
#     )

#     # Get the package directory 
#     pkg_gazebo = get_package_share_directory('bot_car')

   

#     # Launch Decription to Spawn Robot Model 
#     spawn_robot_world = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_gazebo, 'launch',
#                          'spawn_robot_ros2.launch.py'),
#         )
#     )

#      # RVIZ Configuration
#     rviz_config_dir = PathJoinSubstitution(
#         [FindPackageShare("bot_car"), "rviz", "display_default.rviz"]
#     )

#     rviz_node = Node(
#         package='rviz2',
#         executable='rviz2',
#         output='screen',
#         name='rviz_node',
#         parameters=[{'use_sim_time': True}],
#         arguments=['-d', rviz_config_dir])

#     rviz_startup_event_handler = RegisterEventHandler(
#     event_handler=OnProcessExit(
#         target_action=spawn_robot_world,
#         on_exit=[rviz_node],
#     )
#     )


#     # Launch Description 
#     return LaunchDescription([
#         gzserver_cmd,
#         gzclient_cmd,
#         spawn_robot_world,
#         rviz_node
        
#     ])
#!/usr/bin/python3
# -*- coding: utf-8 -*-
#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction

def generate_launch_description():
    # Get Gazebo ROS interface package
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Get the location for the empty world
    world_file_path = os.path.join(get_package_share_directory('ur10'), 'worlds', 'empty_world.world')

    # Include Gazebo server launch
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file_path}.items(),
    )

    # Include Gazebo client launch
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    # RViz configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur10"), "rviz", "src/ur10/rviz/display_mybot.rviz"]
    )

    # Define the RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
    )

    rviz_start_delay = 5.0  # Delay in seconds

    rviz_node_with_delay = TimerAction(
        period=rviz_start_delay,
        actions=[rviz_node],
    )


    # Include robot spawn launch
    pkg_ur10 = get_package_share_directory('ur10')
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ur10, 'launch', 'spawn_robot_ros2.launch.py')
        )
    )

    return LaunchDescription([
        gzserver,
        gzclient,
        spawn_robot,
        rviz_node_with_delay,
    ])
