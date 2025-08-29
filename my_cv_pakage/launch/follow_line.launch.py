# my_cv_pakage/launch/follow_line.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    arena = os.path.join(
        get_package_share_directory('tb3_rwu_arena'),
        'launch', 'tb3_sim.launch.py'
    )

    return LaunchDescription([
        # Start RWU arena sim with robot pose
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(arena),
            launch_arguments={'x':'0.82','y':'-1.75','yaw':'0.0'}.items()
        ),

        # Start camera display node immediately
        Node(package='my_cv_pakage', executable='cv_view',
             name='cv_view', output='screen'),

        # Start line follower after 10s
        TimerAction(period=10.0, actions=[
            Node(package='my_cv_pakage', executable='follow_line_node',
                 name='follow_line', output='screen')
        ]),
    ])

