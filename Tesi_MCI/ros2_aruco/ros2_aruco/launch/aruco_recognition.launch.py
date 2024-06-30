
import os

from ament_index_python.packages import get_package_share_directory

from irobot_create_common_bringup.namespace import GetNamespacedName
from irobot_create_common_bringup.offset import OffsetParser, RotationalOffsetX, RotationalOffsetY

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace


ARGUMENTS = [
    
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
]


# Rviz requires US locale to correctly display the wheels
os.environ['LC_NUMERIC'] = 'en_US.UTF-8'


def generate_launch_description():
    # Directories

    
    namespace = LaunchConfiguration('namespace')

    aruco_node = GroupAction([
        PushRosNamespace(namespace),

        Node(package='ros2_aruco',
             executable='aruco_node',
             name='aruco_node',
             namespace= LaunchConfiguration('namespace'),
             output='screen'),
    ])

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(aruco_node)
    return ld

