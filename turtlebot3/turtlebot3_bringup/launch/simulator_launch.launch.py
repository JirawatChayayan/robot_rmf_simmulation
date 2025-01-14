import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    launch_navigate_file_dir = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'launch')
    launch_gazebo_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    launch_bringup = os.path.join(get_package_share_directory('turtlebot3_bringup'), 'launch')

    launch_gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                        os.path.join(launch_gazebo_file_dir, 'turtlebot3_my_world.launch.py')
                    )
                )

    launch_navigate = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                        os.path.join(launch_navigate_file_dir, 'multi_navigate.launch.py')
                    )
                )
    
    launch_freefeet = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                        os.path.join(launch_bringup, 'ff_client.launch.py')
                    )
                )

    ld = LaunchDescription()
    ld.add_action(launch_navigate)
    ld.add_action(launch_gazebo)
    ld.add_action(launch_freefeet)
    return ld