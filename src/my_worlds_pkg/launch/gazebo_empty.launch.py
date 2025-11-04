from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

import os
def generate_launch_description():
    # Path to your world file
    world_file = os.path.join(
        get_package_share_directory('my_worlds_pkg'),
        'worlds',
        'empty.sdf' # this is the world file that will be opened
        )

    # launch gazebo and load the world
    return LaunchDescription([

        ExecuteProcess(
            cmd=['gz', 'sim', world_file, '--render-engine', 'ogre'],
            output='screen'
        )
    ])
