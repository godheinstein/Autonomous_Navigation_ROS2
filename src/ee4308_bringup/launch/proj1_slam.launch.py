from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # simply launches cartographer.
    pkg_turtlebot3_cartographer = get_package_share_directory('turtlebot3_cartographer')
    
    cartographer_launch_path = os.path.join(pkg_turtlebot3_cartographer, 'launch', 'cartographer.launch.py'),

    # cartographer (slam)
    launch_turtlebot3_cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cartographer_launch_path),
        launch_arguments = {
            'use_sim_time': 'false',
        }.items(),
    )
    ld.add_action(launch_turtlebot3_cartographer)

    return ld