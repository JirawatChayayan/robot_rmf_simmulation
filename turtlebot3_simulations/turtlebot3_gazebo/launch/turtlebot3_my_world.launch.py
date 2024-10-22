#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'my_world.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    
    poseList = [
        {
            'x' : '-4.4',
            'y' : '3.48'
        },
        {
            'x' : '4.475',
            'y' : '3.37'
        },
        {
            'x' : '-4.32',
            'y' : '-0.55'
        },
        {
            'x' : '4.28',
            'y' : '-0.59'
        },
        {
            'x' : '-5.07',
            'y' : '-4.96'
        },
    ]


    SPAWN_ROBOT = os.environ['SPAWN_ROBOT']
    if(SPAWN_ROBOT == None or SPAWN_ROBOT == ''):
        SPAWN_ROBOT = '1'

    spawn_count = int(SPAWN_ROBOT)
    if(spawn_count > 5):
        spawn_count = 5
    elif(spawn_count <1):
        spawn_count = 1
    robot_s = []
    ypos = 4.2
    for i in range(0,spawn_count):
        yp = ypos-(0.3*float(i))
        spawn_turtlebot_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': poseList[i]['x'],
                'y_pose': poseList[i]['y'],
                'ns': 'utac'+str(i+1),
                'use_sim_time' : use_sim_time
            }.items()
        )
        robot_s.append(spawn_turtlebot_cmd)


    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    for a in robot_s:
        ld.add_action(a)

    

    return ld
