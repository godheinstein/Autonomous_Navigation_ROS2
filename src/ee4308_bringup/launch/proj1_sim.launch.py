from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

import xacro
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_ee4308_bringup = get_package_share_directory('ee4308_bringup')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros') 
    
    yaml_map_file = os.path.join(pkg_ee4308_bringup, 'maps', 'my_map_sim.yaml')
    yaml_params_file = os.path.join(pkg_ee4308_bringup, 'params', 'proj1.yaml') #from turtlebot3_navigation2 burger.yaml, changed 'robot_model_type: 'nav2_amcl::DifferentialMotionModel''

    turtle_x = LaunchConfiguration('turtle_x')
    turtle_x_arg = DeclareLaunchArgument('turtle_x', default_value='0.0')
    ld.add_action(turtle_x_arg)

    turtle_y = LaunchConfiguration('turtle_y')
    turtle_y_arg = DeclareLaunchArgument('turtle_y', default_value='-1.0')
    ld.add_action(turtle_y_arg)

    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument('world', default_value=os.path.join(pkg_ee4308_bringup, 'worlds', 'turtlebot3_house.world'))
    ld.add_action(world_arg) # note the worlds must have a more relaxed contact coefficient (~100), and a less accurate solver to prevent erroneous collisions resulting in NAN values.
    
    gz_client = LaunchConfiguration('gz_client') # if true, gz_client is not launched
    gz_client = DeclareLaunchArgument('gz_client', default_value='false')
    ld.add_action(gz_client) 

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    ld.add_action(use_sim_time_arg)

    # get turtle description
    turtle_description_config = xacro.process_file(
        os.path.join(pkg_ee4308_bringup, 'urdf', 'turtle_proj1.urdf.xacro'))
    turtle_desc = turtle_description_config.toxml()

    # open gazebo server
    launch_gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world,
                          'verbose': 'true',
                          }.items()
    )
    ld.add_action(launch_gzserver)

    # open gazebo client
    launch_gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('gz_client')),
        launch_arguments={'verbose': 'true'}.items(),
    )
    ld.add_action(launch_gzclient)

    # turtle state publisher
    node_turtle_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': turtle_desc, 'frame_prefix': ''}],
        arguments=[turtle_desc]
    )
    ld.add_action(node_turtle_state_publisher)

    # nav2 bring_up
    launch_nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments = {
            'use_sim_time': use_sim_time,
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

    # spawn turtle
    node_spawn_turtle = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtle',
            '-topic', 'robot_description', # os.path.join(pkg_ee4308_bringup, 'models', 'turtlebot3_burger', 'model.sdf'),
            '-x', turtle_x,
            '-y', turtle_y,
            '-z', '0.01',
        ],
        output='screen',
        on_exit=[node_rviz]
    )
    ld.add_action(node_spawn_turtle)

    return ld