import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

"""
    Generate a LaunchDescription for the ROS2 package.

    Returns:
        LaunchDescription: The generated launch description.
"""
def generate_launch_description():
    
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

 
    world = os.path.join(
        get_package_share_directory('overfill_detection'),
        'worlds',
        'empty_world.world'
    )
    
    use_discovery = DeclareLaunchArgument('use_discovery', default_value='true', description='Enable/disable discovery service')


    # print(f"World path: {world}")
   
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

    lidar_listener_node = Node(
        package='overfill_detection',
        executable='trial',
        name='lidar_listener',
        output='screen'
        
    )
    
    ld = LaunchDescription()
    
 
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(lidar_listener_node)
   
    
    return ld
