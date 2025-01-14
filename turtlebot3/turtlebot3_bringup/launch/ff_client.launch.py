import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = []

    SPAWN_ROBOT = os.environ['SPAWN_ROBOT']
    if(SPAWN_ROBOT == None or SPAWN_ROBOT == ''):
        SPAWN_ROBOT = '1'

    spawn_count = int(SPAWN_ROBOT)
    if(spawn_count > 5):
        spawn_count = 5
    elif(spawn_count <1):
        spawn_count = 1
    for j in range(0,spawn_count):
        i = j+1
        node_ff_client = Node(
            package='free_fleet_client_ros2',
            executable='free_fleet_client_ros2',
            name=f'turtlebot3_free_fleet_client_node_utac{i}',
            output='both',
            parameters=[
                {'fleet_name': 'utac_fleet'},
                {'robot_name': f'utac{i}'},
                {'robot_model': 'turtlebot3'},
                {'level_name': 'L1'},
                {'dds_domain': 42},
                {'max_dist_to_first_waypoint': 10.0},
                {'map_frame': 'map'},
                {'robot_frame': 'base_footprint'},
                {'nav2_server_name': f'utac{i}/navigate_to_pose'},
                {'use_sim_time': True},
                {'update_frequency': 10.0}
            ],
            remappings=[
                ('/battery_state', f'utac{i}/battery_state'),
                ('/tf', f'utac{i}/tf'),
                ('/tf_static', f'utac{i}/tf_static')
            ]
        )
        node_fake_batt = Node(
            package='rmf_setup',
            executable='fake_battery',
            name=f'fake_battery_utac{i}',
            output='both',
            namespace=f'utac{i}'
        )


        nodes.append(node_ff_client)
        nodes.append(node_fake_batt)

    node_fake_door = Node(
            package='rmf_setup',
            executable='fake_door',
            name=f'fake_door',
            output='both',
        )
    nodes.append(node_fake_door)
    return LaunchDescription(nodes)
