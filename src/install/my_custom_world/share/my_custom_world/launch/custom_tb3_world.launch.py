from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define robot names
    robot1_name = 'robot1'
    robot2_name = 'robot2'

    # Get paths to required packages
    world_path = os.path.join(get_package_share_directory('my_custom_world'), 'worlds', 'custom_world.world')

    # Launch Gazebo with custom world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'robot_name': robot1_name  # Main robot will use default topics
        }.items()
    )
    
    # Spawn robot2 with namespace
    spawn_robot2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot2_name,
            '-file', os.path.join(get_package_share_directory('turtlebot3_gazebo'), 
                       'models/turtlebot3_burger/model.sdf'),
            '-x', '1.0', '-y', '0.0', '-z', '0.01',
            '-robot_namespace', robot2_name
        ],
        output='screen'
    )

    # SLAM for robot1 (global map on /map)
    slam_robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'map_topic': 'map',  # Explicitly set to /map
            'scan_topic': 'scan',
            'base_frame': 'base_footprint',
            'odom_frame': 'odom'
        }.items()
    )
    
    # SLAM for robot2 (local map on /robot2/map)
    slam_robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'namespace': robot2_name,
            'map_topic': 'map',  # Will become /robot2/map due to namespace
            'scan_topic': 'scan',
            'base_frame': 'base_footprint',
            'odom_frame': 'odom'
        }.items()
    )


    # Map merger node
    map_merger_node = Node(
        package='my_custom_world',  
        executable='map_merger',  
        output='screen',
        parameters=[],
        remappings=[
            ('/map', '/map'),  # Ensure that the merged map is published on the /map topic
            ('/robot2/map', '/robot2/map')  # Mapping for robot2's map
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_robot2,
        slam_robot1,
        slam_robot2,
        map_merger_node  # Add the map merger node
    ])

