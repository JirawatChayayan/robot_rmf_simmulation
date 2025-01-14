import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'launch')
    initList = [
        {
            'x' : '5.19',
            'y' : '-0.68',
            'rot':{
                'z' : '-0.9317268443466659',
                'w' : '0.36315986496831365'
            }
        },
        {
            'x' : '-1.63',
            'y' : '-6.23',
            'rot':{
                'z' : '-0.9317268443466659',
                'w' : '0.36315986496831365'
            }
        },
        {
            'x' : '2.58',
            'y' : '2.33',
            'rot':{
                'z' : '-0.9317268443466659',
                'w' : '0.36315986496831365'
            }
        },
        {
            'x' : '-3.59',
            'y' : '-3.04',
            'rot':{
                'z' : '-0.9317268443466659',
                'w' : '0.36315986496831365'
            }
        },
        {
            'x' : '-0.34',
            'y' : '5.61',
            'rot':{
                'z' : '-0.9317268443466659',
                'w' : '0.36315986496831365'
            }
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

    ld = LaunchDescription()
    for i in range(0,spawn_count):
            ns = 'utac'+str(i+1)
            data_init = initList[i]
            x = data_init['x']
            y = data_init['y']
            z = data_init['rot']['z']
            w = data_init['rot']['w']
            # print(data_init)

            spawn_turtlebot_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_file_dir, 'navigate.launch.py')
                ),
                launch_arguments={
                    'ns': ns,
                    'use_sim_time' : 'true',
                    'use_rviz' : 'false'
                }.items()
            )
            init_node = Node(
                  package='turtlebot3_navigation2',
                  executable='initpose.py',
                  name=f'init_{ns}',
                  namespace=ns,
                  arguments=[
                    '-x', x,
                    '-y', y,
                    '-z', z,
                    '-w', w,
                ],
            )
            delayed_node_init = TimerAction(
                period=5.0,
                actions=[init_node]
            )
            ld.add_action(spawn_turtlebot_cmd)
            ld.add_action(delayed_node_init)
    return ld