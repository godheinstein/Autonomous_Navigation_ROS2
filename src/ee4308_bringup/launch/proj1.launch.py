from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_ee4308_bringup = get_package_share_directory('ee4308_bringup')
    
    yaml_map_file = os.path.join(pkg_ee4308_bringup, 'maps', 'my_map.yaml')
    yaml_params_file = os.path.join(pkg_ee4308_bringup, 'params', 'proj1.yaml') #from turtlebot3_navigation2 burger.yaml, changed 'robot_model_type: 'nav2_amcl::DifferentialMotionModel''

    # nav2 bring_up
    launch_nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments = {
            'use_sim_time': 'false',
            'autostart': 'true',
            'map': yaml_map_file,
            'params_file': yaml_params_file
        }.items(),
    )
    ld.add_action(launch_nav2_bringup)
    
    # rviz 
    rviz_file = os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        parameters=[yaml_params_file],
        output='screen'
    )
    ld.add_action(node_rviz)

    return ld